// Fillets and chamfers (S02 §5, BREP-06). Declarations and categories are in occt_shim.hxx.
//
// Both run BRepFilletAPI_MakeFillet / _MakeChamfer on a private BRepBuilderAPI_Copy of the input
// (category C): ChFi3d updates tolerances and adds pcurves on the faces and edges it blends, so
// the caller's TShapes are never handed to it. The copy keeps the input's sub-shape structure,
// and the history is reported from the input through the copy (amg-brep's index of the caller's
// shape).
//
// An edge spec names its edge (and, for asymmetric chamfers, its reference face) by index in the
// caller's shape. Edges are added in the caller's order; OCCT propagates each one along its
// tangent chain into a contour, numbered in the order contours are created. An edge already in a
// contour (added through tangency) is skipped when its spec equals the one that created the
// contour, and refused otherwise.
//
// A failed build is not an exception: the op reports `errors` (stripe status per faulty contour,
// faulty vertices, the OCCT exception if one was raised) and the offending input edges, and holds
// OCCT's partial result (HasResult / BadShape) as its shape when there is one, else a null shape.

#include "amg-occt-sys/cpp/occt_history.hxx"
#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRep_Tool.hxx>
#include <ChFi3d_FilBuilder.hxx>
#include <ChFi3d_FilletShape.hxx>
#include <ChFiDS_ErrorStatus.hxx>
#include <Geom2d_Curve.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

namespace amg::occt {

namespace {

using namespace history;

// Role codes of OcOp::roles beyond the sweep ones (amg-brep `ops::history_n`).
constexpr std::uint8_t FILLET_BLEND = 3;
constexpr std::uint8_t CHAMFER_FACE = 4;
constexpr std::uint8_t CORNER_PATCH = 5;

// Spec::kind.
constexpr std::uint8_t FILLET_CONSTANT = 0;
constexpr std::uint8_t FILLET_VARIABLE = 1;
constexpr std::uint8_t CHAMFER_EQUAL = 2;
constexpr std::uint8_t CHAMFER_TWO_DISTANCES = 3;
constexpr std::uint8_t CHAMFER_DISTANCE_ANGLE = 4;

// An error that amg-brep reports as `InvalidInput` (prefix checked in `blend::outcome`).
constexpr const char *INVALID = "invalid: ";

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

const char *stripe_status(ChFiDS_ErrorStatus status) {
  switch (status) {
  case ChFiDS_Ok:
    return "ChFiDS_Ok";
  case ChFiDS_WalkingFailure:
    return "ChFiDS_WalkingFailure";
  case ChFiDS_StartsolFailure:
    return "ChFiDS_StartsolFailure";
  case ChFiDS_TwistedSurface:
    return "ChFiDS_TwistedSurface";
  default:
    return "ChFiDS_Error";
  }
}

// ChFi3d's approximation defaults, read from a builder on the same shape (they are protected
// members with no getters), so an override of one parameter keeps OCCT's value for the others.
class DefaultParams : public ChFi3d_FilBuilder {
public:
  explicit DefaultParams(const TopoDS_Shape &s) : ChFi3d_FilBuilder(s) {}

  void apply(BRepFilletAPI_MakeFillet &maker, double app3d, double fl) const {
    maker.SetParams(tolappangle, tolesp, tol2d, app3d > 0.0 ? app3d : tolapp3d, tolapp2d,
                    fl > 0.0 ? fl : fleche);
  }
};

ChFi3d_FilletShape fillet_shape(std::uint8_t shape) {
  switch (shape) {
  case 1:
    return ChFi3d_QuasiAngular;
  case 2:
    return ChFi3d_Polynomial;
  default:
    return ChFi3d_Rational;
  }
}

// One edge spec with its own copy of the radius law (`OcBlendEdge` indexes the shared `laws`).
struct Spec {
  std::uint32_t edge;
  std::uint8_t kind;
  double a;
  double b;
  std::uint32_t face;
  std::vector<double> law; // (t, r) pairs
};

std::vector<Spec> specs_of(rust::Slice<const OcBlendEdge> edges, rust::Slice<const double> laws) {
  std::vector<Spec> out;
  out.reserve(edges.size());
  for (const OcBlendEdge &e : edges) {
    if (e.law_start > e.law_end || e.law_end > laws.size() || (e.law_end - e.law_start) % 2 != 0) {
      fail("blend: bad law range");
    }
    Spec spec{e.edge, e.kind, e.a, e.b, e.face, {}};
    spec.law.assign(laws.begin() + e.law_start, laws.begin() + e.law_end);
    out.push_back(std::move(spec));
  }
  return out;
}

bool same_spec(const Spec &a, const Spec &b) {
  if (a.kind != b.kind || a.a != b.a || a.b != b.b || a.law.size() != b.law.size()) {
    return false;
  }
  return std::equal(a.law.begin(), a.law.end(), b.law.begin());
}

// The input's faces and edges and their copies, both in the input's index order.
struct Copied {
  BRepBuilderAPI_Copy copier;
  TopoDS_Shape base;
  ShapeMap edges; // copy of input edge i at index i + 1
  ShapeMap faces;

  Copied(const TopoDS_Shape &input, const OcIndex &index) : copier(input, false, false) {
    base = copier.Shape();
    for (int i = 1; i <= index.edges.Extent(); ++i) {
      edges.Add(image(index.edges(i)));
    }
    for (int i = 1; i <= index.faces.Extent(); ++i) {
      faces.Add(image(index.faces(i)));
    }
  }

  TopoDS_Shape image(const TopoDS_Shape &s) {
    const NCollection_List<TopoDS_Shape> &images = copier.Modified(s);
    if (images.IsEmpty()) {
      fail("blend: an input sub-shape has no copy");
    }
    return images.First();
  }

  const TopoDS_Edge &edge(std::uint32_t e) const {
    if (e >= static_cast<std::uint32_t>(edges.Extent())) {
      fail("blend: edge index out of range");
    }
    return TopoDS::Edge(edges(static_cast<int>(e) + 1));
  }

  const TopoDS_Face &face(std::uint32_t f) const {
    if (f >= static_cast<std::uint32_t>(faces.Extent())) {
      fail("blend: face index out of range");
    }
    return TopoDS::Face(faces(static_cast<int>(f) + 1));
  }

  // The input index of a copied edge; -1 when it is not one.
  int input_edge(const TopoDS_Shape &e) const { return edges.FindIndex(e) - 1; }
};

// The shared interface of the two OCCT builders that this file uses.
template <typename Maker> struct Blend;

template <> struct Blend<BRepFilletAPI_MakeFillet> {
  static void add(BRepFilletAPI_MakeFillet &maker, const Copied &c, const Spec &spec) {
    const TopoDS_Edge &e = c.edge(spec.edge);
    if (spec.kind == FILLET_CONSTANT) {
      maker.Add(spec.a, e);
    } else if (spec.kind == FILLET_VARIABLE) {
      const int n = static_cast<int>(spec.law.size() / 2);
      NCollection_Array1<gp_Pnt2d> law(1, n);
      for (int i = 0; i < n; ++i) {
        law.SetValue(i + 1, gp_Pnt2d(spec.law[2 * i], spec.law[2 * i + 1]));
      }
      maker.Add(law, e);
    } else {
      fail("blend: a chamfer spec in a fillet");
    }
  }

  static void faults(BRepFilletAPI_MakeFillet &maker, const Copied &c,
                     std::vector<std::string> &errors, std::vector<int> &contours,
                     std::vector<TopoDS_Vertex> &vertices) {
    for (int i = 1; i <= maker.NbFaultyContours(); ++i) {
      const int ic = maker.FaultyContour(i);
      contours.push_back(ic);
      errors.push_back("contour " + std::to_string(ic - 1) + ": " +
                       stripe_status(maker.StripeStatus(ic)));
    }
    for (int i = 1; i <= maker.NbFaultyVertices(); ++i) {
      vertices.push_back(maker.FaultyVertex(i));
    }
    (void)c;
  }

  static bool has_partial(BRepFilletAPI_MakeFillet &maker) { return maker.HasResult(); }
  static TopoDS_Shape partial(BRepFilletAPI_MakeFillet &maker) { return maker.BadShape(); }
};

template <> struct Blend<BRepFilletAPI_MakeChamfer> {
  static void add(BRepFilletAPI_MakeChamfer &maker, const Copied &c, const Spec &spec) {
    const TopoDS_Edge &e = c.edge(spec.edge);
    if (spec.kind == CHAMFER_EQUAL) {
      maker.Add(spec.a, e);
    } else if (spec.kind == CHAMFER_TWO_DISTANCES) {
      maker.Add(spec.a, spec.b, e, c.face(spec.face));
    } else if (spec.kind == CHAMFER_DISTANCE_ANGLE) {
      maker.AddDA(spec.a, spec.b, e, c.face(spec.face));
    } else {
      fail("blend: a fillet spec in a chamfer");
    }
  }

  // BRepFilletAPI_MakeChamfer does not expose its builder's faulty contours: the caller probes.
  static void faults(BRepFilletAPI_MakeChamfer &, const Copied &, std::vector<std::string> &,
                     std::vector<int> &, std::vector<TopoDS_Vertex> &) {}

  static bool has_partial(BRepFilletAPI_MakeChamfer &) { return false; }
  static TopoDS_Shape partial(BRepFilletAPI_MakeChamfer &) { return TopoDS_Shape(); }
};

void push_offending(OcOp &op, std::uint32_t edge) {
  for (std::size_t j = 0; j < op.offending_indices.size(); ++j) {
    if (op.offending_kinds[j] == 1 && op.offending_indices[j] == edge) {
      return;
    }
  }
  op.offending_operands.push_back(0);
  op.offending_kinds.push_back(1);
  op.offending_indices.push_back(edge);
}

// Adds `specs` to `maker` in order. Returns, per contour (0-based), the specs that are in it;
// throws an `INVALID` error for an edge OCCT ignored or a conflicting spec on one tangent chain,
// after naming that edge in `op` (when given).
template <typename Maker>
std::vector<std::vector<std::size_t>> add_all(Maker &maker, const Copied &c,
                                              const std::vector<Spec> &specs, OcOp *op) {
  std::vector<std::vector<std::size_t>> contours;
  for (std::size_t k = 0; k < specs.size(); ++k) {
    const Spec &spec = specs[k];
    const TopoDS_Edge &e = c.edge(spec.edge);
    const int existing = maker.Contour(e);
    if (existing != 0) {
      const std::vector<std::size_t> &in = contours[static_cast<std::size_t>(existing - 1)];
      if (!same_spec(specs[in.front()], spec)) {
        if (op != nullptr) {
          push_offending(*op, spec.edge);
        }
        fail(std::string(INVALID) + "edge " + std::to_string(spec.edge) +
             " is on the tangent chain of edge " + std::to_string(specs[in.front()].edge) +
             " with a different spec");
      }
      contours[static_cast<std::size_t>(existing - 1)].push_back(k);
      continue;
    }
    Blend<Maker>::add(maker, c, spec);
    const int ic = maker.Contour(e);
    if (ic == 0) {
      if (op != nullptr) {
        push_offending(*op, spec.edge);
      }
      fail(std::string(INVALID) + "OCCT cannot blend edge " + std::to_string(spec.edge) +
           " (a smooth edge between tangent faces has no blend)");
    }
    if (static_cast<std::size_t>(ic) != contours.size() + 1) {
      fail("blend: unexpected contour numbering");
    }
    contours.push_back({k});
  }
  return contours;
}

int solid_count(const TopoDS_Shape &s) {
  ShapeMap solids;
  TopExp::MapShapes(s, TopAbs_SOLID, solids);
  return solids.Extent();
}

// Builds; returns whether it is done with as many solids as the input has (`solids`). ChFi3d can
// report done with a result that fell apart into loose faces (a variable radius that outgrows a
// face); that is a failure too. An exception from Build becomes an error message.
template <typename Maker>
bool build(Maker &maker, int solids, std::vector<std::string> &errors) {
  try {
    maker.Build();
    if (!maker.IsDone()) {
      return false;
    }
    const int got = solid_count(maker.Shape());
    if (got == solids) {
      return true;
    }
    errors.push_back("the result has " + std::to_string(got) + " solids, the input " +
                     std::to_string(solids));
  } catch (const Standard_Failure &failure) {
    errors.push_back(format_failure(failure));
  }
  return false;
}

template <typename Maker> std::unique_ptr<Maker> make(const Copied &c, const OcFilletParams &p);

template <>
std::unique_ptr<BRepFilletAPI_MakeFillet> make(const Copied &c, const OcFilletParams &p) {
  auto maker = std::make_unique<BRepFilletAPI_MakeFillet>(c.base, fillet_shape(p.shape));
  if (p.tol_app_3d > 0.0 || p.fleche > 0.0) {
    DefaultParams(c.base).apply(*maker, p.tol_app_3d, p.fleche);
  }
  return maker;
}

template <>
std::unique_ptr<BRepFilletAPI_MakeChamfer> make(const Copied &c, const OcFilletParams &) {
  return std::make_unique<BRepFilletAPI_MakeChamfer>(c.base);
}

// Fills the errors and the offending input edges of a failed build.
template <typename Maker>
void report_failure(OcOp &op, Maker &maker, const Copied &c,
                    const std::vector<Spec> &specs,
                    const std::vector<std::vector<std::size_t>> &contours,
                    const OcFilletParams &params) {
  std::vector<int> faulty;
  std::vector<TopoDS_Vertex> vertices;
  try {
    Blend<Maker>::faults(maker, c, op.errors, faulty, vertices);
  } catch (const Standard_Failure &failure) {
    op.errors.push_back("faults unavailable (" + format_failure(failure) + ")");
  }
  for (int ic : faulty) {
    if (ic >= 1 && static_cast<std::size_t>(ic) <= contours.size()) {
      for (std::size_t k : contours[static_cast<std::size_t>(ic - 1)]) {
        push_offending(op, specs[k].edge);
      }
    }
  }
  for (const TopoDS_Vertex &v : vertices) {
    bool any = false;
    for (const Spec &spec : specs) {
      const TopoDS_Edge &e = c.edge(spec.edge);
      if (TopExp::FirstVertex(e).IsSame(v) || TopExp::LastVertex(e).IsSame(v)) {
        push_offending(op, spec.edge);
        any = true;
      }
    }
    op.errors.push_back(any ? "a corner failed at a vertex of the blended edges"
                            : "a corner failed at a vertex");
  }
  if (op.offending_indices.empty()) {
    // No fault list (chamfers, or a failure the builder does not attribute): each contour that
    // fails on its own is offending; when none does, they fail together and all are.
    for (std::size_t ic = 0; ic < contours.size() && contours.size() > 1; ++ic) {
      std::vector<Spec> alone;
      for (std::size_t k : contours[ic]) {
        alone.push_back(specs[k]);
      }
      std::unique_ptr<Maker> probe = make<Maker>(c, params);
      std::vector<std::string> ignored;
      add_all(*probe, c, alone, nullptr);
      if (!build(*probe, solid_count(c.base), ignored)) {
        op.errors.push_back("contour " + std::to_string(ic) + " fails on its own");
        for (const Spec &spec : alone) {
          push_offending(op, spec.edge);
        }
      }
    }
    if (op.offending_indices.empty()) {
      for (const Spec &spec : specs) {
        push_offending(op, spec.edge);
      }
    }
  }
  if (op.errors.empty()) {
    op.errors.emplace_back("NotDone");
  }
  try {
    if (Blend<Maker>::has_partial(maker)) {
      op.shape = Blend<Maker>::partial(maker);
    } else if (maker.IsDone()) {
      op.shape = maker.Shape(); // done, but without its solids
    }
  } catch (const Standard_Failure &) {
    op.shape = TopoDS_Shape();
  }
}

// The history from the input through the copy and the blend, and the roles of the new faces:
// a face generated by an edge of contour `ic` is a blend face of that contour, every other face
// that no input face became is a corner patch.
template <typename Maker>
void record(OcOp &op, Maker &maker, Copied &c, const TopoDS_Shape &input,
            const std::vector<Spec> &specs,
            const std::vector<std::vector<std::size_t>> &contours, std::uint8_t role) {
  const auto modified = [&](const TopoDS_Shape &s) -> Shapes {
    const TopoDS_Shape copy = c.image(s);
    // IsDeleted is not used: it reports every sub-shape outside the builder's own map, vertices
    // that pass through unchanged included. A copy that is not in the result has no relation.
    const NCollection_List<TopoDS_Shape> &images = maker.Modified(copy);
    if (!images.IsEmpty()) {
      return shapes_of(images);
    }
    return {copy};
  };
  const auto generated = [&](const TopoDS_Shape &s) -> Shapes {
    return shapes_of(maker.Generated(c.image(s)));
  };
  record_operands(op, {input}, Source{modified, generated}, {}, false);

  const Maps out(op.shape);
  ShapeMap from_faces;
  const Maps in(input);
  for (int f = 1; f <= in.m[0].Extent(); ++f) {
    for (const TopoDS_Shape &image : modified(in.m[0](f))) {
      from_faces.Add(image);
    }
  }
  std::vector<std::int64_t> contour_of(static_cast<std::size_t>(out.m[0].Extent()), -1);
  for (std::size_t ic = 0; ic < contours.size(); ++ic) {
    // Every edge of the OCCT contour, the propagated ones included.
    const int n = maker.NbEdges(static_cast<int>(ic) + 1);
    for (int j = 1; j <= n; ++j) {
      const TopoDS_Edge &e = maker.Edge(static_cast<int>(ic) + 1, j);
      for (const TopoDS_Shape &face : maker.Generated(e)) {
        const int i = out.m[0].FindIndex(face);
        if (i != 0 && contour_of[static_cast<std::size_t>(i - 1)] < 0 &&
            !from_faces.Contains(face)) {
          contour_of[static_cast<std::size_t>(i - 1)] = static_cast<std::int64_t>(ic);
        }
      }
    }
  }
  (void)specs;
  for (int i = 1; i <= out.m[0].Extent(); ++i) {
    const TopoDS_Shape &face = out.m[0](i);
    if (from_faces.Contains(face)) {
      continue;
    }
    const std::int64_t ic = contour_of[static_cast<std::size_t>(i - 1)];
    op.role_faces.push_back(static_cast<std::uint32_t>(i - 1));
    if (ic >= 0) {
      op.roles.push_back(role);
      op.role_args.push_back(static_cast<std::uint32_t>(ic));
    } else {
      op.roles.push_back(CORNER_PATCH);
      op.role_args.push_back(0);
    }
  }
}

template <typename Maker>
std::unique_ptr<OcOp> run(const OcShape &shape, const OcIndex &index,
                          rust::Slice<const OcBlendEdge> edges, rust::Slice<const double> laws,
                          const OcFilletParams &params, bool with_history, std::uint8_t role) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("blend: the shape is null");
  }
  if (edges.empty()) {
    fail("blend: no edges");
  }
  const std::vector<Spec> specs = specs_of(edges, laws);
  Copied c(input, index);
  std::unique_ptr<Maker> maker = make<Maker>(c, params);
  auto op = std::make_unique<OcOp>();
  std::vector<std::vector<std::size_t>> contours;
  try {
    contours = add_all(*maker, c, specs, op.get());
  } catch (const Standard_ConstructionError &error) {
    const std::string what = error.what();
    if (what.rfind(INVALID, 0) == 0) {
      op->errors.push_back(what);
      return op;
    }
    throw;
  }
  if (!build(*maker, solid_count(c.base), op->errors)) {
    report_failure(*op, *maker, c, specs, contours, params);
    return op;
  }
  op->shape = maker->Shape();
  if (with_history) {
    record(*op, *maker, c, input, specs, contours, role);
  }
  return op;
}

} // namespace

std::unique_ptr<OcOp> blend_op(const OcShape &shape, const OcIndex &index, bool chamfer,
                               rust::Slice<const OcBlendEdge> edges, rust::Slice<const double> laws,
                               const OcFilletParams &params, bool with_history) {
  if (chamfer) {
    return run<BRepFilletAPI_MakeChamfer>(shape, index, edges, laws, params, with_history,
                                          CHAMFER_FACE);
  }
  return run<BRepFilletAPI_MakeFillet>(shape, index, edges, laws, params, with_history,
                                       FILLET_BLEND);
}

double edge_face_angle(const OcIndex &index, std::uint32_t e, std::uint32_t f1, std::uint32_t f2) {
  const auto n_edges = static_cast<std::uint32_t>(index.edges.Extent());
  const auto n_faces = static_cast<std::uint32_t>(index.faces.Extent());
  if (e >= n_edges || f1 >= n_faces || f2 >= n_faces) {
    fail("edge_face_angle: index out of range");
  }
  const TopoDS_Edge &edge = TopoDS::Edge(index.edges(static_cast<int>(e) + 1));
  double first = 0.0;
  double last = 0.0;
  BRep_Tool::Range(edge, first, last);
  const double mid = 0.5 * (first + last);
  gp_Vec normals[2];
  const std::uint32_t faces[2] = {f1, f2};
  for (int k = 0; k < 2; ++k) {
    const TopoDS_Face &face = TopoDS::Face(index.faces(static_cast<int>(faces[k]) + 1));
    double p0 = 0.0;
    double p1 = 0.0;
    const occ::handle<Geom2d_Curve> pcurve = BRep_Tool::CurveOnSurface(edge, face, p0, p1);
    if (pcurve.IsNull()) {
      fail("edge_face_angle: the edge has no pcurve on face " + std::to_string(faces[k]));
    }
    const gp_Pnt2d uv = pcurve->Value(mid);
    BRepAdaptor_Surface surface(face, false);
    gp_Pnt p;
    gp_Vec du;
    gp_Vec dv;
    surface.D1(uv.X(), uv.Y(), p, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-300) {
      fail("edge_face_angle: degenerate surface normal");
    }
    n.Normalize();
    if (face.Orientation() == TopAbs_REVERSED) {
      n.Reverse();
    }
    normals[k] = n;
  }
  const double c = std::max(-1.0, std::min(1.0, normals[0].Dot(normals[1])));
  return std::acos(c);
}

} // namespace amg::occt
