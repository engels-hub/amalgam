// Healing and assembly (S02 §9, BREP-10): heal, unify same domain, sew, make solid, make compound
// and the BOPAlgo_ArgumentAnalyzer pass of `check`. Declarations and categories are in
// occt_shim.hxx.
//
// Every op that changes topology runs on a private BRepBuilderAPI_Copy of its input (category C):
// ShapeFix and the sewing write tolerances into the sub-shapes they touch, and
// BRepBuilderAPI_MakeSolid sets the shell's `Closed` flag on its TShape. The history is reported
// from the caller's shapes through the copy (`Modified` 1:1 for everything the op leaves alone)
// and the op's own steps: the BRepTools_History of each ShapeFix / UnifySameDomain context, the
// sewing's modified sub-shapes.

#include "amg-occt-sys/cpp/occt_history.hxx"
#include "amg-occt-sys/cpp/occt_profile.hxx"
#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BOPAlgo_CheckResult.hxx>
#include <BOPAlgo_CheckStatus.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRep_Builder.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_TFace.hxx>
#include <BRep_TVertex.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Surface.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <Precision.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Shell.hxx>
#include <ShapeFix_Solid.hxx>
#include <ShapeFix_Wireframe.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_CompSolid.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

namespace amg::occt {

namespace {

using namespace history;

// An error that amg-brep reports as `InvalidInput` (same prefix as occt_blend.cpp).
constexpr const char *INVALID = "invalid: ";

// OcCheckProblem::kind of a problem on no indexed sub-shape (occt_shim.cpp's CHECK_WHOLE).
constexpr std::uint8_t CHECK_WHOLE = 3;

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

class SinkIndicator : public Message_ProgressIndicator {
public:
  explicit SinkIndicator(const ProgressSink &sink) : sink_(sink) {}

  bool UserBreak() override { return progress_user_break(sink_); }

  void Show(const Message_ProgressScope &, const bool) override {
    progress_show(sink_, GetPosition());
  }

private:
  const ProgressSink &sink_;
};

double tolerance_of(const TopoDS_Shape &s) {
  switch (s.ShapeType()) {
  case TopAbs_FACE:
    return BRep_Tool::Tolerance(TopoDS::Face(s));
  case TopAbs_EDGE:
    return BRep_Tool::Tolerance(TopoDS::Edge(s));
  case TopAbs_VERTEX:
    return BRep_Tool::Tolerance(TopoDS::Vertex(s));
  default:
    return 0.0;
  }
}

// The sewing as a history step: a sub-shape of the sewing's input becomes its sewn version (an
// edge merged with its neighbour, a face with sewn edges); a degenerated face is removed; an edge
// the sewing cut into sections becomes each section.
Step sewing_step(const BRepBuilderAPI_Sewing &sewer) {
  const BRepBuilderAPI_Sewing *s = &sewer;
  return [s](const TopoDS_Shape &x) -> Shapes {
    TopoDS_Shape image;
    if (s->IsModifiedSubShape(x)) {
      image = s->ModifiedSubShape(x);
    } else if (s->IsModified(x)) {
      image = s->Modified(x);
    } else {
      return {x};
    }
    if (image.IsNull()) {
      return {};
    }
    if (image.ShapeType() == x.ShapeType()) {
      return {image};
    }
    Shapes out;
    ShapeMap seen;
    for (TopExp_Explorer it(image, x.ShapeType()); it.More(); it.Next()) {
      if (seen.Add(it.Current())) {
        out.push_back(it.Current());
      }
    }
    return out;
  };
}

// Closed shells become solids; what `solids_of` made.
struct Solidified {
  TopoDS_Shape shape;
  int solids = 0;
  int open = 0; // open shells and free faces left as they are
};

// Every closed shell of `s` (outside any solid) becomes a solid, oriented so that it bounds a
// finite volume (ShapeFix_Solid::SolidFromShell: BRepBuilderAPI_MakeSolid, then reversed when
// the infinite point classifies inside). Open shells and faces outside shells stay as they are,
// solids of `s` too. One item comes back as itself, several as a compound in `s`'s order. The
// faces are kept (same TShapes), so the history needs no step for this.
Solidified solids_of(const TopoDS_Shape &s) {
  Solidified out;
  std::vector<TopoDS_Shape> items;
  ShapeMap in_shells;
  for (TopExp_Explorer it(s, TopAbs_SOLID); it.More(); it.Next()) {
    items.push_back(it.Current());
    TopExp::MapShapes(it.Current(), TopAbs_FACE, in_shells);
    ++out.solids;
  }
  const auto add_shell = [&](const TopoDS_Shell &shell, const TopoDS_Shape &as_is) {
    if (BRep_Tool::IsClosed(shell)) {
      ShapeFix_Solid fix;
      items.push_back(fix.SolidFromShell(shell));
      ++out.solids;
    } else {
      items.push_back(as_is);
      ++out.open;
    }
  };
  for (TopExp_Explorer it(s, TopAbs_SHELL, TopAbs_SOLID); it.More(); it.Next()) {
    const TopoDS_Shell &shell = TopoDS::Shell(it.Current());
    TopExp::MapShapes(shell, TopAbs_FACE, in_shells);
    add_shell(shell, shell);
  }
  // A face outside any shell may close on itself (a sphere, a torus: seams and degenerated
  // edges only); it then makes a solid of its own.
  BRep_Builder builder;
  for (TopExp_Explorer it(s, TopAbs_FACE, TopAbs_SHELL); it.More(); it.Next()) {
    if (in_shells.Add(it.Current())) {
      TopoDS_Shell shell;
      builder.MakeShell(shell);
      builder.Add(shell, it.Current());
      add_shell(shell, it.Current());
    }
  }
  if (items.size() == 1) {
    out.shape = items.front();
    return out;
  }
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const TopoDS_Shape &item : items) {
    builder.Add(compound, item);
  }
  out.shape = compound;
  return out;
}

// What a fix can change on a sub-shape it keeps: its tolerance, an edge's SameParameter and
// SameRange flags, a face's orientation in the shape (its first occurrence).
struct State {
  double tolerance = 0.0;
  bool same_parameter = true;
  bool same_range = true;
  TopAbs_Orientation orientation = TopAbs_EXTERNAL;

  // Whether `after` (this state after the fixes) differs from `before` by a fix: a grown
  // tolerance, a changed flag or a flipped face. A tolerance that shrank (SameParameter
  // recomputing it) is no fix.
  bool fixed_from(const State &before) const {
    return tolerance > before.tolerance || same_parameter != before.same_parameter ||
           same_range != before.same_range || orientation != before.orientation;
  }
};

// The state of every face, edge and vertex of `shape` (`maps` are its maps), in index order.
void collect_states(const TopoDS_Shape &shape, const Maps &maps, std::vector<State> (&out)[3]) {
  for (int kind = 0; kind < 3; ++kind) {
    out[kind].assign(static_cast<std::size_t>(maps.m[kind].Extent()), State{});
    for (int i = 1; i <= maps.m[kind].Extent(); ++i) {
      const TopoDS_Shape &s = maps.m[kind](i);
      State &st = out[kind][static_cast<std::size_t>(i - 1)];
      st.tolerance = tolerance_of(s);
      if (kind == 1) {
        st.same_parameter = BRep_Tool::SameParameter(TopoDS::Edge(s));
        st.same_range = BRep_Tool::SameRange(TopoDS::Edge(s));
      }
    }
  }
  for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
    const int i = maps.m[0].FindIndex(it.Current());
    if (i == 0) {
      continue;
    }
    State &st = out[0][static_cast<std::size_t>(i - 1)];
    if (st.orientation == TopAbs_EXTERNAL) {
      st.orientation = it.Current().Orientation();
    }
  }
}

// Copy sub-shapes the fixes replaced, removed or split, or whose state changed: the number of
// fixes a heal applied. `before` is the state of the copy (`maps`) before the fixes.
std::uint32_t count_fixes(const Maps &maps, const std::vector<State> (&before)[3],
                          const TopoDS_Shape &result, const std::vector<Step> &steps) {
  const Maps out(result);
  std::vector<State> after[3];
  collect_states(result, out, after);
  std::uint32_t fixes = 0;
  for (int kind = 0; kind < 3; ++kind) {
    for (int i = 1; i <= maps.m[kind].Extent(); ++i) {
      const TopoDS_Shape &s = maps.m[kind](i);
      const Shapes images = through(steps, {s});
      const int j = images.size() == 1 && images.front().IsSame(s)
                        ? out.m[kind].FindIndex(images.front())
                        : 0;
      if (j == 0 || after[kind][static_cast<std::size_t>(j - 1)].fixed_from(
                        before[kind][static_cast<std::size_t>(i - 1)])) {
        ++fixes;
      }
    }
  }
  return fixes;
}

// `s` under a new top-level TShape of its own type that holds the same children (and, for a
// face, edge or vertex, the same geometry, tolerance and flags), with `s`'s location and
// orientation. Everything below the top level is shared. TopoDS_Builder::Add freezes the
// component's TShape (its `Free` flag, written by BinTools): adding the copy leaves `s` as it is,
// and its children were frozen when they were added to `s`.
TopoDS_Shape fresh_top(const TopoDS_Shape &s) {
  BRep_Builder builder;
  TopoDS_Shape out;
  switch (s.ShapeType()) {
  case TopAbs_COMPOUND: {
    TopoDS_Compound c;
    builder.MakeCompound(c);
    out = c;
    break;
  }
  case TopAbs_COMPSOLID: {
    TopoDS_CompSolid c;
    builder.MakeCompSolid(c);
    out = c;
    break;
  }
  case TopAbs_SOLID: {
    TopoDS_Solid c;
    builder.MakeSolid(c);
    out = c;
    break;
  }
  case TopAbs_SHELL: {
    TopoDS_Shell c;
    builder.MakeShell(c);
    out = c;
    break;
  }
  case TopAbs_WIRE: {
    TopoDS_Wire c;
    builder.MakeWire(c);
    out = c;
    break;
  }
  case TopAbs_FACE: {
    const occ::handle<BRep_TFace> from = occ::down_cast<BRep_TFace>(s.TShape());
    TopoDS_Face f;
    builder.MakeFace(f, from->Surface(), from->Location(), from->Tolerance());
    builder.NaturalRestriction(f, from->NaturalRestriction());
    out = f;
    break;
  }
  case TopAbs_EDGE: {
    const occ::handle<BRep_TEdge> from = occ::down_cast<BRep_TEdge>(s.TShape());
    TopoDS_Edge e;
    builder.MakeEdge(e);
    const occ::handle<BRep_TEdge> to = occ::down_cast<BRep_TEdge>(e.TShape());
    to->ChangeCurves() = from->Curves();
    to->Tolerance(from->Tolerance());
    to->SameParameter(from->SameParameter());
    to->SameRange(from->SameRange());
    to->Degenerated(from->Degenerated());
    out = e;
    break;
  }
  case TopAbs_VERTEX: {
    const occ::handle<BRep_TVertex> from = occ::down_cast<BRep_TVertex>(s.TShape());
    TopoDS_Vertex v;
    builder.MakeVertex(v, from->Pnt(), from->Tolerance());
    occ::down_cast<BRep_TVertex>(v.TShape())->ChangePoints() = from->Points();
    out = v;
    break;
  }
  default:
    return s;
  }
  for (TopoDS_Iterator it(s.Located(TopLoc_Location()).Oriented(TopAbs_FORWARD), false, false);
       it.More(); it.Next()) {
    builder.Add(out, it.Value());
  }
  out.Closed(s.Closed());
  out.Orientable(s.Orientable());
  out.Infinite(s.Infinite());
  out.Convex(s.Convex());
  out.Checked(s.Checked());
  out.Location(s.Location());
  out.Orientation(s.Orientation());
  return out;
}

const char *check_status_name(BOPAlgo_CheckStatus status) {
  switch (status) {
  case BOPAlgo_BadType:
    return "BOPAlgo_BadType";
  case BOPAlgo_SelfIntersect:
    return "BOPAlgo_SelfIntersect";
  case BOPAlgo_TooSmallEdge:
    return "BOPAlgo_TooSmallEdge";
  case BOPAlgo_NonRecoverableFace:
    return "BOPAlgo_NonRecoverableFace";
  case BOPAlgo_IncompatibilityOfVertex:
    return "BOPAlgo_IncompatibilityOfVertex";
  case BOPAlgo_IncompatibilityOfEdge:
    return "BOPAlgo_IncompatibilityOfEdge";
  case BOPAlgo_IncompatibilityOfFace:
    return "BOPAlgo_IncompatibilityOfFace";
  case BOPAlgo_OperationAborted:
    return "BOPAlgo_OperationAborted";
  case BOPAlgo_GeomAbs_C0:
    return "BOPAlgo_GeomAbs_C0";
  case BOPAlgo_InvalidCurveOnSurface:
    return "BOPAlgo_InvalidCurveOnSurface";
  case BOPAlgo_NotValid:
    return "BOPAlgo_NotValid";
  default:
    return "BOPAlgo_CheckUnknown";
  }
}

void push_problem(std::vector<OcCheckProblem> &out, std::uint8_t kind, std::uint32_t index,
                  const char *status) {
  for (const OcCheckProblem &p : out) {
    if (p.kind == kind && p.index == index && std::string(p.status) == status) {
      return;
    }
  }
  out.push_back(OcCheckProblem{kind, index, rust::String(status)});
}

} // namespace

std::unique_ptr<OcOp> heal_op(const OcShape &shape, const OcHealParams &params,
                              const ProgressSink &sink) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("heal: the shape is null");
  }
  auto op = std::make_unique<OcOp>();
  // Geometry is copied too (S02 §9): the fixes trim and re-parameterise curves and surfaces.
  BRepBuilderAPI_Copy copier(input, true, false);
  const TopoDS_Shape base = copier.Shape();
  const Maps copy_maps(base);
  std::vector<State> before[3];
  collect_states(base, copy_maps, before);

  occ::handle<ShapeBuild_ReShape> shape_context = new ShapeBuild_ReShape;
  ShapeFix_Shape fixer;
  fixer.SetContext(shape_context);
  fixer.Init(base);
  fixer.SetPrecision(params.tol);
  fixer.SetMinTolerance(std::min(params.tol, Precision::Confusion()));
  fixer.SetMaxTolerance(params.max_tol);
  if (!params.fix_orientation) {
    fixer.FixSolidTool()->FixShellOrientationMode() = 0;
    fixer.FixShellTool()->FixOrientationMode() = 0;
    fixer.FixFaceTool()->FixOrientationMode() = 0;
  }
  occ::handle<SinkIndicator> indicator = new SinkIndicator(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg heal", 1);
    fixer.Perform(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  if (!op->errors.empty()) {
    return op;
  }
  std::vector<Step> steps{history_step(shape_context->History())};

  occ::handle<ShapeBuild_ReShape> wire_context = new ShapeBuild_ReShape;
  ShapeFix_Wireframe wireframe;
  wireframe.SetContext(wire_context);
  wireframe.Load(fixer.Shape());
  wireframe.SetPrecision(params.tol);
  wireframe.SetMaxTolerance(params.max_tol);
  try {
    wireframe.FixWireGaps();
    if (params.fix_small_edges) {
      wireframe.ModeDropSmallEdges() = true;
      wireframe.FixSmallEdges();
    }
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
    return op;
  }
  steps.push_back(history_step(wire_context->History()));

  op->shape = wireframe.Shape();
  if (op->shape.IsNull()) {
    op->errors.emplace_back("the healed shape is empty");
    return op;
  }
  op->fixes = count_fixes(copy_maps, before, op->shape, steps);

  // A fix may build faces it does not record (ShapeFix_Face splitting a face along a
  // self-intersecting wire): each keeps the surface of the face it comes from, so a result face
  // no relation reaches is `Modified` from the input face whose copy has its surface.
  const Maps in(input);
  const Maps out(op->shape);
  const auto images = [&](const TopoDS_Shape &s) {
    return through(steps, shapes_of(copier.Modified(s)));
  };
  ShapeMap reached;
  for (int i = 1; i <= in.m[0].Extent(); ++i) {
    for (const TopoDS_Shape &image : images(in.m[0](i))) {
      reached.Add(image);
    }
  }
  std::vector<Shapes> unrecorded(static_cast<std::size_t>(in.m[0].Extent()));
  for (int j = 1; j <= out.m[0].Extent(); ++j) {
    const TopoDS_Face &face = TopoDS::Face(out.m[0](j));
    if (reached.Contains(face)) {
      continue;
    }
    TopLoc_Location loc;
    const occ::handle<Geom_Surface> &surface = BRep_Tool::Surface(face, loc);
    for (int i = 1; i <= in.m[0].Extent(); ++i) {
      const NCollection_List<TopoDS_Shape> &copy = copier.Modified(in.m[0](i));
      if (copy.IsEmpty()) {
        continue;
      }
      TopLoc_Location copy_loc;
      const TopoDS_Face &copy_face = TopoDS::Face(copy.First());
      if (BRep_Tool::Surface(copy_face, copy_loc) == surface && copy_loc.IsEqual(loc)) {
        unrecorded[static_cast<std::size_t>(i - 1)].push_back(face);
        break;
      }
    }
  }
  Source source;
  source.modified = [&](const TopoDS_Shape &s) {
    Shapes all = images(s);
    if (s.ShapeType() == TopAbs_FACE) {
      const int i = in.m[0].FindIndex(s);
      if (i != 0) {
        for (const TopoDS_Shape &f : unrecorded[static_cast<std::size_t>(i - 1)]) {
          all.push_back(f);
        }
      }
    }
    return all;
  };
  source.generated = [](const TopoDS_Shape &) { return Shapes{}; };
  record_operands(*op, {input}, source, {}, false);
  return op;
}

namespace {

// Whether ShapeUpgrade_UnifySameDomain could merge two faces of these surface types: planes
// only when parallel within `max(1e3 x ang_tol, 1e-6)` rad (its own test is ang_tol), other
// elementary surfaces only with their own type; anything else (B-splines, offsets, extrusions,
// ...) always could.
bool same_domain_possible(const TopoDS_Face &a, const TopoDS_Face &b, double ang_tol) {
  const BRepAdaptor_Surface sa(a, false);
  const BRepAdaptor_Surface sb(b, false);
  const GeomAbs_SurfaceType ta = sa.GetType();
  const GeomAbs_SurfaceType tb = sb.GetType();
  const auto elementary = [](GeomAbs_SurfaceType t) {
    return t == GeomAbs_Plane || t == GeomAbs_Cylinder || t == GeomAbs_Cone ||
           t == GeomAbs_Sphere || t == GeomAbs_Torus;
  };
  if (!elementary(ta) || !elementary(tb)) {
    return true;
  }
  if (ta != tb) {
    return false;
  }
  if (ta != GeomAbs_Plane) {
    return true;
  }
  const double cos_max = std::cos(std::max(1e3 * ang_tol, 1e-6));
  return std::abs(sa.Plane().Axis().Direction().Dot(sb.Plane().Axis().Direction())) >= cos_max;
}

// KH-20: whether UnifySameDomain (safe input mode, no internal edges, as unify_same_domain_op
// runs it) could change `s` at all; a superset of what it merges. Faces merge only across an
// edge shared by exactly two different faces on the same domain (UnifyFaces skips every other
// edge), edges only through a vertex with exactly two edges (CheckSharedVertices keeps the
// others). Without such a pair the algorithm returns its input unchanged with an empty
// history, so the op can skip it.
bool could_unify(const TopoDS_Shape &s, const OcUnifyParams &params) {
  if (params.faces) {
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>,
                               TopTools_ShapeMapHasher>
        edge_faces;
    TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, edge_faces);
    for (int i = 1; i <= edge_faces.Extent(); ++i) {
      const TopoDS_Edge &edge = TopoDS::Edge(edge_faces.FindKey(i));
      const NCollection_List<TopoDS_Shape> &faces = edge_faces(i);
      if (BRep_Tool::Degenerated(edge) || faces.Extent() != 2 ||
          faces.First().IsSame(faces.Last())) {
        continue;
      }
      if (same_domain_possible(TopoDS::Face(faces.First()), TopoDS::Face(faces.Last()),
                               params.ang_tol)) {
        return true;
      }
    }
  }
  if (params.edges) {
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>,
                               TopTools_ShapeMapHasher>
        vertex_edges;
    TopExp::MapShapesAndUniqueAncestors(s, TopAbs_VERTEX, TopAbs_EDGE, vertex_edges);
    for (int i = 1; i <= vertex_edges.Extent(); ++i) {
      if (vertex_edges(i).Extent() == 2) {
        return true;
      }
    }
  }
  return false;
}

} // namespace

bool unify_candidates(const OcShape &shape, const OcUnifyParams &params) {
  if (shape.get().IsNull()) {
    fail("unify_candidates: the shape is null");
  }
  return could_unify(shape.get(), params);
}

std::unique_ptr<OcOp> unify_same_domain_op(const OcShape &shape, const OcUnifyParams &params) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("unify_same_domain: the shape is null");
  }
  // As unify_op (BREP-05): UnifySameDomain rebuilds what it merges through a BRepTools_ReShape
  // of its input, so it runs on a private copy whose map is the first step of the history.
  const profile::Phases phases("unify_same_domain");
  BRepBuilderAPI_Copy copier(input, false, false);
  phases.mark("copied");
  if (params.fast && !could_unify(copier.Shape(), params)) {
    // Nothing to merge: the copy is the result, as UnifySameDomain would return it.
    auto op = std::make_unique<OcOp>();
    op->shape = copier.Shape();
    record_operands(*op, {input}, source_of(copier), {}, false);
    phases.mark("nothing to unify, history recorded");
    return op;
  }
  ShapeUpgrade_UnifySameDomain unifier(copier.Shape(), params.edges, params.faces,
                                       params.concat_bsplines);
  unifier.SetLinearTolerance(params.lin_tol);
  unifier.SetAngularTolerance(params.ang_tol);
  unifier.Build();
  phases.mark("built");
  auto op = std::make_unique<OcOp>();
  op->shape = unifier.Shape();
  record_operands(*op, {input}, source_of(copier), {history_step(unifier.History())}, false);
  phases.mark("history recorded");
  return op;
}

std::unique_ptr<OcOp> sew_op(const OcShapeList &shapes, const OcSewParams &params,
                             const ProgressSink &sink) {
  if (shapes.shapes.empty()) {
    fail("sew: no shapes");
  }
  // One copy per input (geometry too: the sewing updates the curves of the edges it merges).
  // The inputs are not put into a compound: TopoDS_Builder::Add freezes the component's TShape
  // (its `Free` flag, which BinTools writes), and the callers' shapes must stay as they are.
  // Sub-shapes two inputs share become two coincident copies, which the sewing merges again.
  std::vector<std::unique_ptr<BRepBuilderAPI_Copy>> copies;
  BRepBuilderAPI_Sewing sewer(params.tol, true, true, true, params.non_manifold);
  for (const TopoDS_Shape &s : shapes.shapes) {
    if (s.IsNull()) {
      fail("sew: a shape is null");
    }
    copies.push_back(std::make_unique<BRepBuilderAPI_Copy>(s, true, false));
    sewer.Add(copies.back()->Shape());
  }
  auto op = std::make_unique<OcOp>();
  occ::handle<SinkIndicator> indicator = new SinkIndicator(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg sew", 1);
    sewer.Perform(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  if (!op->errors.empty()) {
    return op;
  }
  TopoDS_Shape result = sewer.SewedShape();
  if (result.IsNull()) {
    op->errors.emplace_back("the sewing gave no shape");
    return op;
  }
  if (params.make_solid) {
    const Solidified made = solids_of(result);
    if (made.open > 0) {
      op->errors.push_back("the sewn shape is not closed: " + std::to_string(made.open) +
                           " open shell(s) or free face(s), " +
                           std::to_string(sewer.NbFreeEdges()) + " free edge(s)");
      op->shape = result;
      return op;
    }
    result = made.shape;
  }
  op->shape = result;

  // The history per operand: its own copy, then the sewing.
  const std::vector<Step> steps{sewing_step(sewer)};
  const Maps out(op->shape);
  Recorder rec(*op, out, true);
  for (std::size_t o = 0; o < shapes.shapes.size(); ++o) {
    const auto operand = static_cast<std::uint16_t>(o);
    const Maps in(shapes.shapes[o]);
    for (int kind = 0; kind < 3; ++kind) {
      for (int i = 1; i <= in.m[kind].Extent(); ++i) {
        const Shapes images = through(steps, shapes_of(copies[o]->Modified(in.m[kind](i))));
        if (!rec.relation(operand, kind, i - 1, MODIFIED, images)) {
          rec.deleted(operand, kind, i - 1);
        }
      }
    }
  }
  return op;
}

std::unique_ptr<OcOp> make_solid_op(const OcShape &shape) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("make_solid: the shape is null");
  }
  auto op = std::make_unique<OcOp>();
  BRepBuilderAPI_Copy copier(input, false, false);
  const Solidified made = solids_of(copier.Shape());
  if (made.open > 0) {
    op->errors.push_back(std::string(INVALID) + std::to_string(made.open) +
                         " shell(s) or face(s) are not closed");
    return op;
  }
  if (made.solids == 0) {
    op->errors.push_back(std::string(INVALID) + "the shape has no shell");
    return op;
  }
  op->shape = made.shape;
  record_operands(*op, {input}, source_of(copier), {}, false);
  return op;
}

std::unique_ptr<OcShape> make_compound_of(const OcShapeList &shapes) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const TopoDS_Shape &s : shapes.shapes) {
    if (s.IsNull()) {
      fail("make_compound: a shape is null");
    }
    builder.Add(compound, fresh_top(s));
  }
  return std::make_unique<OcShape>(compound);
}

rust::Vec<OcCheckProblem> analyze_shape(const OcShape &shape, const OcIndex &index) {
  rust::Vec<OcCheckProblem> out;
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    return out;
  }
  BRepBuilderAPI_Copy copier(input, false, false);
  // copies[k](i + 1) is the copy of the input's sub-shape i of kind k.
  ShapeMap copies[3];
  const ShapeMap *const maps[3] = {&index.faces, &index.edges, &index.vertices};
  for (int kind = 0; kind < 3; ++kind) {
    for (int i = 1; i <= maps[kind]->Extent(); ++i) {
      const NCollection_List<TopoDS_Shape> &images = copier.Modified((*maps[kind])(i));
      copies[kind].Add(images.IsEmpty() ? (*maps[kind])(i) : images.First());
    }
  }
  BOPAlgo_ArgumentAnalyzer analyzer;
  analyzer.SetShape1(copier.Shape());
  analyzer.ArgumentTypeMode() = false;
  analyzer.SelfInterMode() = true;
  analyzer.SmallEdgeMode() = true;
  analyzer.CurveOnSurfaceMode() = true;
  analyzer.Perform();
  std::vector<OcCheckProblem> problems;
  for (const BOPAlgo_CheckResult &result : analyzer.GetCheckResult()) {
    const char *status = check_status_name(result.GetCheckStatus());
    bool placed = false;
    for (const TopoDS_Shape &faulty : result.GetFaultyShapes1()) {
      const int kind = kind_of(faulty);
      const int i = kind < 0 ? 0 : copies[kind].FindIndex(faulty);
      if (i != 0) {
        push_problem(problems, static_cast<std::uint8_t>(kind), static_cast<std::uint32_t>(i - 1),
                     status);
        placed = true;
      }
    }
    if (!placed) {
      push_problem(problems, CHECK_WHOLE, 0, status);
    }
  }
  for (OcCheckProblem &problem : problems) {
    out.push_back(std::move(problem));
  }
  return out;
}

std::uint32_t op_fix_count(const OcOp &op) { return op.fixes; }

} // namespace amg::occt
