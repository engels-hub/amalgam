// Primitives, prism, revol and affine transforms (S02 §5, BREP-14). Declarations and categories
// are in occt_shim.hxx.
//
// Inputs are validated in amg-brep (sizes, angles, profile kinds); this file builds the shapes
// and reports each op's history from operand 0 in index terms: for every face, edge and vertex
// of the input (TopExp::MapShapes order, amg-brep's index), the result sub-shapes it became
// (`Modified`) and the ones it swept out (`Generated`), plus cap and lateral roles. Sweeps run on
// a private BRepBuilderAPI_Copy of the profile (category C), because BRepSweep stores pcurves of
// the lateral faces on the base edges it sweeps; the copy's history is composed in.

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepSweep_Revol.hxx>
#include <BRepTools_History.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <NCollection_List.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Mat.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>

namespace amg::occt {

namespace {

constexpr std::uint8_t MODIFIED = 0;
constexpr std::uint8_t GENERATED = 1;
constexpr std::uint8_t DELETED = 2;

constexpr std::uint8_t START_CAP = 0;
constexpr std::uint8_t END_CAP = 1;
constexpr std::uint8_t LATERAL = 2;

// `BRepOffsetAPI_DraftAngle` tilts a face by +angle towards the pull direction's side of its
// outward normal; amg-brep's positive taper grows the profile along the sweep, which leans the
// side faces outward, so the draft angle is the negated taper.
constexpr double DRAFT_SIGN = -1.0;

using Shapes = std::vector<TopoDS_Shape>;
// One step of an op: where a shape of the previous step's result went.
using Step = std::function<Shapes(const TopoDS_Shape &)>;

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

gp_Pnt pnt(const V3 &v) { return gp_Pnt(v.x, v.y, v.z); }
gp_Dir dir(const V3 &v) { return gp_Dir(v.x, v.y, v.z); }
gp_Vec vec(const V3 &v) { return gp_Vec(v.x, v.y, v.z); }
gp_Ax2 ax2(const OcFrame &f) { return gp_Ax2(pnt(f.origin), dir(f.normal), dir(f.x)); }

Shapes shapes_of(const NCollection_List<TopoDS_Shape> &list) {
  Shapes out;
  for (const TopoDS_Shape &s : list) {
    out.push_back(s);
  }
  return out;
}

// A sweep accessor that may raise or return null for a sub-shape without that image (a
// semi-infinite prism has no last shape).
template <typename F> Shapes guarded(F &&f) {
  try {
    TopoDS_Shape s = f();
    if (!s.IsNull()) {
      return {s};
    }
  } catch (const Standard_Failure &) {
  }
  return {};
}

// 0 face, 1 edge, 2 vertex; -1 for every other kind.
int kind_of(const TopoDS_Shape &s) {
  switch (s.ShapeType()) {
  case TopAbs_FACE:
    return 0;
  case TopAbs_EDGE:
    return 1;
  case TopAbs_VERTEX:
    return 2;
  default:
    return -1;
  }
}

// The face, edge and vertex maps of a shape, in amg-brep's index order.
struct Maps {
  ShapeMap m[3];
  explicit Maps(const TopoDS_Shape &s) {
    TopExp::MapShapes(s, TopAbs_FACE, m[0]);
    TopExp::MapShapes(s, TopAbs_EDGE, m[1]);
    TopExp::MapShapes(s, TopAbs_VERTEX, m[2]);
  }
};

// Shapes after the later steps of an op (draft, unify), in order.
Shapes through(const std::vector<Step> &steps, Shapes shapes) {
  for (const Step &step : steps) {
    Shapes next;
    for (const TopoDS_Shape &s : shapes) {
      for (TopoDS_Shape &image : step(s)) {
        next.push_back(std::move(image));
      }
    }
    shapes = std::move(next);
  }
  return shapes;
}

// Collects the op history in index terms of the final result.
class Recorder {
public:
  Recorder(OcOp &op, const Maps &out)
      : op_(op), out_(out), has_role_(static_cast<std::size_t>(out.m[0].Extent()), false) {}

  // Records one relation; returns whether any output survived into the result.
  bool relation(int kind, int index, std::uint8_t relation, const Shapes &outputs) {
    std::vector<std::pair<std::uint8_t, std::uint32_t>> found;
    for (const TopoDS_Shape &o : outputs) {
      const int k = kind_of(o);
      if (k < 0) {
        continue;
      }
      int i = out_.m[k].FindIndex(o);
      if (i == 0) {
        i = by_vertices(o, k);
      }
      if (i == 0) {
        continue;
      }
      const std::pair<std::uint8_t, std::uint32_t> key{static_cast<std::uint8_t>(k),
                                                        static_cast<std::uint32_t>(i - 1)};
      if (std::find(found.begin(), found.end(), key) == found.end()) {
        found.push_back(key);
      }
    }
    if (found.empty()) {
      return false;
    }
    push(kind, index, relation);
    for (const auto &[k, i] : found) {
      op_.out_kinds.push_back(k);
      op_.out_indices.push_back(i);
    }
    op_.out_starts.push_back(static_cast<std::uint32_t>(op_.out_kinds.size()));
    return true;
  }

  void deleted(int kind, int index) {
    push(kind, index, DELETED);
    op_.out_starts.push_back(static_cast<std::uint32_t>(op_.out_kinds.size()));
  }

  // Labels the result faces among `faces` that have no role yet.
  void role(const Shapes &faces, std::uint8_t role) {
    for (const TopoDS_Shape &f : faces) {
      if (kind_of(f) != 0) {
        continue;
      }
      int i = out_.m[0].FindIndex(f);
      if (i == 0) {
        i = by_vertices(f, 0);
      }
      if (i == 0 || has_role_[static_cast<std::size_t>(i - 1)]) {
        continue;
      }
      has_role_[static_cast<std::size_t>(i - 1)] = true;
      op_.role_faces.push_back(static_cast<std::uint32_t>(i - 1));
      op_.roles.push_back(role);
    }
  }

private:
  // The sorted vertex indices of `s` in the result, and its edge count; empty when a vertex is
  // not in the result.
  std::pair<std::vector<int>, int> signature(const TopoDS_Shape &s) const {
    ShapeMap vertices;
    TopExp::MapShapes(s, TopAbs_VERTEX, vertices);
    ShapeMap edges;
    TopExp::MapShapes(s, TopAbs_EDGE, edges);
    std::vector<int> out;
    for (int v = 1; v <= vertices.Extent(); ++v) {
      const int i = out_.m[2].FindIndex(vertices(v));
      if (i == 0) {
        return {};
      }
      out.push_back(i);
    }
    std::sort(out.begin(), out.end());
    return {out, edges.Extent()};
  }

  // BRepPrimAPI_MakeRevol replaces some swept faces and edges after the sweep (surfaces of
  // revolution canonised into planes, cones, ...) and `Generated` still returns the sweep's
  // originals. The vertices survive, so an image missing from the result is matched to the
  // one result face or edge (1-based; 0 when none or ambiguous) with the same vertices and
  // edge count.
  int by_vertices(const TopoDS_Shape &image, int kind) const {
    if (kind > 1) {
      return 0;
    }
    const auto want = signature(image);
    if (want.first.empty()) {
      return 0;
    }
    int found = 0;
    for (int i = 1; i <= out_.m[kind].Extent(); ++i) {
      if (signature(out_.m[kind](i)) == want) {
        if (found != 0) {
          return 0;
        }
        found = i;
      }
    }
    return found;
  }

  void push(int kind, int index, std::uint8_t relation) {
    op_.in_kinds.push_back(static_cast<std::uint8_t>(kind));
    op_.in_indices.push_back(static_cast<std::uint32_t>(index));
    op_.relations.push_back(relation);
  }

  OcOp &op_;
  const Maps &out_;
  std::vector<bool> has_role_;
};

// Where the copy put input sub-shape `s`.
TopoDS_Shape copied(BRepBuilderAPI_Copy &copier, const TopoDS_Shape &s) {
  const NCollection_List<TopoDS_Shape> &images = copier.Modified(s);
  if (images.IsEmpty()) {
    fail("sweep: a profile sub-shape has no copy");
  }
  return images.First();
}

// What the sweep itself made from `c`, where the API's `Generated` has nothing:
// BRepPrimAPI_MakeRevol canonises some swept faces after the sweep (a revolved radial line
// becomes a plane) and then reports their generating edges as deleted. The raw face is
// matched to its replacement by `Recorder::by_vertices`.
Shapes raw_swept(BRepPrimAPI_MakePrism &, const TopoDS_Shape &) { return {}; }

Shapes raw_swept(BRepPrimAPI_MakeRevol &sweep, const TopoDS_Shape &c) {
  if (c.ShapeType() != TopAbs_EDGE && c.ShapeType() != TopAbs_VERTEX) {
    return {};
  }
  return guarded([&] { return const_cast<BRepSweep_Revol &>(sweep.Revol()).Shape(c); });
}

// The history of a sweep (`Sweep` is BRepPrimAPI_MakePrism or _MakeRevol) of the copied
// profile, through the later `steps`.
template <typename Sweep>
void record_sweep(OcOp &op, const TopoDS_Shape &input, BRepBuilderAPI_Copy &copier,
                  Sweep &sweep, const std::vector<Step> &steps) {
  const Maps in(input);
  const Maps out(op.shape);
  Recorder rec(op, out);
  for (int kind = 0; kind < 3; ++kind) {
    for (int i = 1; i <= in.m[kind].Extent(); ++i) {
      const TopoDS_Shape c = copied(copier, in.m[kind](i));
      const Shapes first = through(steps, guarded([&] { return sweep.FirstShape(c); }));
      const Shapes last = through(steps, guarded([&] { return sweep.LastShape(c); }));
      Shapes images = shapes_of(sweep.Generated(c));
      if (images.empty()) {
        images = raw_swept(sweep, c);
      }
      Shapes swept = through(steps, std::move(images));
      const Shapes lateral = swept;
      swept.insert(swept.end(), last.begin(), last.end());
      const bool modified = rec.relation(kind, i - 1, MODIFIED, first);
      const bool generated = rec.relation(kind, i - 1, GENERATED, swept);
      if (!modified && !generated) {
        rec.deleted(kind, i - 1);
      }
      if (kind == 0) {
        rec.role(first, START_CAP);
        rec.role(last, END_CAP);
      } else if (kind == 1) {
        rec.role(lateral, LATERAL);
      }
    }
  }
}

// Unifies coplanar faces and collinear edges of `shape` (a new result; category N), and returns
// the step that maps `shape`'s sub-shapes into it.
Step unify(std::unique_ptr<ShapeUpgrade_UnifySameDomain> &holder, TopoDS_Shape &shape) {
  holder = std::make_unique<ShapeUpgrade_UnifySameDomain>(shape, true, true, false);
  holder->Build();
  shape = holder->Shape();
  ShapeUpgrade_UnifySameDomain *u = holder.get();
  return [u](const TopoDS_Shape &s) -> Shapes {
    const occ::handle<BRepTools_History> &history = u->History();
    if (history.IsNull()) {
      return {s};
    }
    if (history->IsRemoved(s)) {
      return {};
    }
    const NCollection_List<TopoDS_Shape> &modified = history->Modified(s);
    if (!modified.IsEmpty()) {
      return shapes_of(modified);
    }
    return {s};
  };
}

} // namespace

std::unique_ptr<OcShape> make_primitive(std::uint8_t kind, const OcFrame &frame,
                                        rust::Slice<const double> params) {
  static constexpr std::size_t COUNTS[5] = {3, 3, 4, 1, 3};
  if (kind >= 5) {
    fail("make_primitive: unknown kind");
  }
  if (params.size() != COUNTS[kind]) {
    fail("make_primitive: wrong parameter count");
  }
  const gp_Ax2 axes = ax2(frame);
  const double *p = params.data();
  TopoDS_Shape shape;
  switch (kind) {
  case 0:
    shape = BRepPrimAPI_MakeBox(axes, p[0], p[1], p[2]).Shape();
    break;
  case 1:
    shape = BRepPrimAPI_MakeCylinder(axes, p[0], p[1], p[2]).Shape();
    break;
  case 2:
    shape = BRepPrimAPI_MakeCone(axes, p[0], p[1], p[2], p[3]).Shape();
    break;
  case 3:
    shape = BRepPrimAPI_MakeSphere(axes, p[0]).Shape();
    break;
  default:
    shape = BRepPrimAPI_MakeTorus(axes, p[0], p[1], p[2]).Shape();
    break;
  }
  return std::make_unique<OcShape>(shape);
}

std::unique_ptr<OcOp> prism(const OcShape &profile, const V3 &v, std::uint8_t mode, double taper,
                            bool merge_coplanar) {
  const TopoDS_Shape &input = profile.get();
  if (input.IsNull()) {
    fail("prism: the profile is null");
  }
  if (mode > 2) {
    fail("prism: unknown extent mode");
  }
  BRepBuilderAPI_Copy copier(input, false, false);
  const TopoDS_Shape base = copier.Shape();
  std::unique_ptr<BRepPrimAPI_MakePrism> sweep;
  if (mode == 0) {
    sweep = std::make_unique<BRepPrimAPI_MakePrism>(base, vec(v), false, true);
  } else {
    sweep = std::make_unique<BRepPrimAPI_MakePrism>(base, dir(v), mode == 2, false, true);
  }
  if (!sweep->IsDone()) {
    fail("prism: BRepPrimAPI_MakePrism failed");
  }
  auto op = std::make_unique<OcOp>();
  TopoDS_Shape shape = sweep->Shape();
  std::vector<Step> steps;

  std::unique_ptr<BRepOffsetAPI_DraftAngle> draft;
  if (taper != 0.0) {
    if (mode != 0 || base.ShapeType() != TopAbs_FACE) {
      fail("prism: a taper needs a finite prism of one face");
    }
    BRepAdaptor_Surface surface(TopoDS::Face(base), false);
    if (surface.GetType() != GeomAbs_Plane) {
      fail("prism: a taper needs a planar profile face");
    }
    const gp_Pln neutral = surface.Plane();
    const gp_Dir pull = dir(v);
    draft = std::make_unique<BRepOffsetAPI_DraftAngle>(shape);
    ShapeMap done;
    ShapeMap base_edges;
    TopExp::MapShapes(base, TopAbs_EDGE, base_edges);
    for (int e = 1; e <= base_edges.Extent(); ++e) {
      for (const TopoDS_Shape &lateral : sweep->Generated(base_edges(e))) {
        if (lateral.ShapeType() != TopAbs_FACE || done.Contains(lateral)) {
          continue;
        }
        const TopoDS_Face &face = TopoDS::Face(lateral);
        draft->Add(face, pull, DRAFT_SIGN * taper, neutral);
        if (!draft->AddDone()) {
          fail("prism: the taper cannot tilt the side face of profile edge " +
               std::to_string(e - 1) +
               " (BRepOffsetAPI_DraftAngle takes planar, cylindrical and conical faces)");
        }
        // Add propagates the draft along tangent faces itself and ignores a face it already has.
        done.Add(face);
      }
    }
    try {
      draft->Build();
    } catch (const Standard_Failure &failure) {
      fail("prism: the taper failed (" + format_failure(failure) + ")");
    }
    if (!draft->IsDone()) {
      fail("prism: BRepOffsetAPI_DraftAngle failed (Draft_ErrorStatus " +
           std::to_string(static_cast<int>(draft->Status())) + ")");
    }
    shape = draft->Shape();
    BRepOffsetAPI_DraftAngle *d = draft.get();
    steps.push_back([d](const TopoDS_Shape &s) -> Shapes {
      const NCollection_List<TopoDS_Shape> &modified = d->Modified(s);
      if (!modified.IsEmpty()) {
        return shapes_of(modified);
      }
      return {s};
    });
  }

  std::unique_ptr<ShapeUpgrade_UnifySameDomain> unifier;
  if (merge_coplanar) {
    steps.push_back(unify(unifier, shape));
  }

  op->shape = shape;
  record_sweep(*op, input, copier, *sweep, steps);
  return op;
}

std::unique_ptr<OcOp> revol(const OcShape &profile, const V3 &origin, const V3 &axis,
                            double angle) {
  const TopoDS_Shape &input = profile.get();
  if (input.IsNull()) {
    fail("revol: the profile is null");
  }
  BRepBuilderAPI_Copy copier(input, false, false);
  const TopoDS_Shape base = copier.Shape();
  BRepPrimAPI_MakeRevol sweep(base, gp_Ax1(pnt(origin), dir(axis)), angle, false);
  if (!sweep.IsDone()) {
    fail("revol: BRepPrimAPI_MakeRevol failed");
  }
  auto op = std::make_unique<OcOp>();
  op->shape = sweep.Shape();
  record_sweep(*op, input, copier, sweep, {});
  return op;
}

std::unique_ptr<OcOp> shape_gtransformed(const OcShape &shape, const OcMat34 &m) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("gtransformed: the shape is null");
  }
  const gp_Mat linear(m.m[0], m.m[1], m.m[2], m.m[4], m.m[5], m.m[6], m.m[8], m.m[9], m.m[10]);
  if (std::abs(linear.Determinant()) < 1e-300) {
    fail("gtransformed: the transform is singular");
  }
  // A similarity (MᵀM = s²·I) keeps analytic geometry through gp_Trsf; anything else is a
  // general affine map.
  const gp_Mat gram = linear.Transposed() * linear;
  const double s2 = (gram(1, 1) + gram(2, 2) + gram(3, 3)) / 3.0;
  bool similarity = true;
  for (int r = 1; r <= 3; ++r) {
    for (int c = 1; c <= 3; ++c) {
      const double expected = r == c ? s2 : 0.0;
      if (std::abs(gram(r, c) - expected) > 1e-12 * s2) {
        similarity = false;
      }
    }
  }

  auto op = std::make_unique<OcOp>();
  std::unique_ptr<BRepBuilderAPI_ModifyShape> builder;
  if (similarity) {
    gp_Trsf trsf;
    trsf.SetValues(m.m[0], m.m[1], m.m[2], m.m[3], m.m[4], m.m[5], m.m[6], m.m[7], m.m[8],
                   m.m[9], m.m[10], m.m[11]);
    builder = std::make_unique<BRepBuilderAPI_Transform>(input, trsf, true, false);
  } else {
    const gp_GTrsf gtrsf(linear, gp_XYZ(m.m[3], m.m[7], m.m[11]));
    builder = std::make_unique<BRepBuilderAPI_GTransform>(input, gtrsf, true);
  }
  if (!builder->IsDone()) {
    fail("gtransformed: the transform failed");
  }
  op->shape = builder->Shape();

  const Maps in(input);
  const Maps out(op->shape);
  Recorder rec(*op, out);
  for (int kind = 0; kind < 3; ++kind) {
    for (int i = 1; i <= in.m[kind].Extent(); ++i) {
      if (!rec.relation(kind, i - 1, MODIFIED, shapes_of(builder->Modified(in.m[kind](i))))) {
        rec.deleted(kind, i - 1);
      }
    }
  }
  return op;
}

std::unique_ptr<OcShape> op_shape(const OcOp &op) { return std::make_unique<OcShape>(op.shape); }

OcOpHistory op_history(const OcOp &op) {
  OcOpHistory out;
  for (std::uint8_t k : op.in_kinds) {
    out.in_kinds.push_back(k);
  }
  for (std::uint32_t i : op.in_indices) {
    out.in_indices.push_back(i);
  }
  for (std::uint8_t r : op.relations) {
    out.relations.push_back(r);
  }
  for (std::uint32_t s : op.out_starts) {
    out.out_starts.push_back(s);
  }
  for (std::uint8_t k : op.out_kinds) {
    out.out_kinds.push_back(k);
  }
  for (std::uint32_t i : op.out_indices) {
    out.out_indices.push_back(i);
  }
  for (std::uint32_t f : op.role_faces) {
    out.role_faces.push_back(f);
  }
  for (std::uint8_t r : op.roles) {
    out.roles.push_back(r);
  }
  return out;
}

} // namespace amg::occt
