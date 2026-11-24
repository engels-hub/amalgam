// Primitives, prism, revol and affine transforms (S02 §5, BREP-14). Declarations and categories
// are in occt_shim.hxx.
//
// Inputs are validated in amg-brep (sizes, angles, profile kinds); this file builds the shapes
// and reports each op's history from operand 0 in index terms: for every face, edge and vertex
// of the input (TopExp::MapShapes order, amg-brep's index), the result sub-shapes it became
// (`Modified`) and the ones it swept out (`Generated`), plus cap and lateral roles. Sweeps run on
// a private BRepBuilderAPI_Copy of the profile (category C), because BRepSweep stores pcurves of
// the lateral faces on the base edges it sweeps; the copy's history is composed in.

#include "amg-occt-sys/cpp/occt_history.hxx"
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
#include <BRepSweep_Prism.hxx>
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

using namespace history;

// `BRepOffsetAPI_DraftAngle` tilts a face by +angle towards the pull direction's side of its
// outward normal; amg-brep's positive taper grows the profile along the sweep, which leans the
// side faces outward, so the draft angle is the negated taper.
constexpr double DRAFT_SIGN = -1.0;

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

gp_Pnt pnt(const V3 &v) { return gp_Pnt(v.x, v.y, v.z); }
gp_Dir dir(const V3 &v) { return gp_Dir(v.x, v.y, v.z); }
gp_Vec vec(const V3 &v) { return gp_Vec(v.x, v.y, v.z); }
gp_Ax2 ax2(const OcFrame &f) { return gp_Ax2(pnt(f.origin), dir(f.normal), dir(f.x)); }

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
// matched to its replacement by the `Recorder`'s vertex match (occt_history.hxx).
// A prism unbounded both ways reports nothing from `Generated` for its profile edges (the
// generating shapes are not used in the result, `GenIsUsed`); the sweep's own shape is the
// side face.
Shapes raw_swept(BRepPrimAPI_MakePrism &sweep, const TopoDS_Shape &c) {
  if (c.ShapeType() != TopAbs_EDGE && c.ShapeType() != TopAbs_VERTEX) {
    return {};
  }
  return guarded([&] { return const_cast<BRepSweep_Prism &>(sweep.Prism()).Shape(c); });
}

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
  Recorder rec(op, out, false, true);
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
      const bool modified = rec.relation(0, kind, i - 1, MODIFIED, first);
      const bool generated = rec.relation(0, kind, i - 1, GENERATED, swept);
      if (!modified && !generated) {
        rec.deleted(0, kind, i - 1);
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
// the step that maps `shape`'s sub-shapes into it. The options are the ones `prism`'s
// `merge_coplanar` and `unify_op` share: faces and edges, no B-spline concatenation.
Step unify(std::unique_ptr<ShapeUpgrade_UnifySameDomain> &holder, TopoDS_Shape &shape) {
  holder = std::make_unique<ShapeUpgrade_UnifySameDomain>(shape, true, true, false);
  holder->Build();
  shape = holder->Shape();
  return history_step(holder->History());
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
    // `Modified` lists only the faces whose geometry the draft changed through new history
    // entries; the tilted side faces themselves come back through `ModifiedShape` (BREP-05:
    // without it the side faces of a tapered prism had no source).
    steps.push_back([d](const TopoDS_Shape &s) -> Shapes {
      const NCollection_List<TopoDS_Shape> &modified = d->Modified(s);
      if (!modified.IsEmpty()) {
        return shapes_of(modified);
      }
      Shapes image = guarded([&] { return d->ModifiedShape(s); });
      if (!image.empty()) {
        return image;
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

  // Every sub-shape is copied into new geometry, so each is `Modified` 1:1.
  record_operands(*op, {input}, source_of(*builder), {}, false);
  return op;
}

std::unique_ptr<OcOp> unify_op(const OcShape &shape) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("unify: the shape is null");
  }
  // UnifySameDomain rebuilds the faces and edges it merges through a BRepTools_ReShape of its
  // input; it runs on a private copy (category C), whose map is the first step of the history.
  BRepBuilderAPI_Copy copier(input, false, false);
  TopoDS_Shape result = copier.Shape();
  std::unique_ptr<ShapeUpgrade_UnifySameDomain> unifier;
  const std::vector<Step> steps{unify(unifier, result)};
  auto op = std::make_unique<OcOp>();
  op->shape = result;
  record_operands(*op, {input}, source_of(copier), steps, false);
  return op;
}

std::unique_ptr<OcShape> op_shape(const OcOp &op) { return std::make_unique<OcShape>(op.shape); }

OcOpHistory op_history(const OcOp &op) {
  OcOpHistory out;
  for (std::uint16_t o : op.in_operands) {
    out.in_operands.push_back(o);
  }
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
