// Draft, defeature, the trim step of replace face, and split faces (S02 §5, BREP-16).
// Declarations and categories are in occt_shim.hxx.
//
// Every op runs on a private BRepBuilderAPI_Copy of the input (category C), as the BREP-06/07
// ops do: BRepOffsetAPI_DraftAngle and the BOPAlgo tools write tolerances and pcurves into the
// sub-shapes they touch. The history is reported from the input through the copy, in amg-brep's
// index of the caller's shape.
//
// - draft_op: BRepOffsetAPI_DraftAngle (planar, cylindrical and conical faces).
// - defeature_op: BRepAlgoAPI_Defeaturing (BOPAlgo_RemoveFeatures); a feature OCCT skips is a
//   failure naming its faces, not a silent partial result.
// - trim_face_op: the second step of replace face. The input already reaches past the new
//   surface at the replaced face (amg-brep extends it with offset_faces); it is split by a face on
//   the new surface and the pieces beyond it are dropped. The replaced face is `Modified` into
//   the faces on the new surface.
// - face_reach: how far the new surface lies beyond the face along its outward normals.
// - split_faces_op: the tool's curves on the chosen faces (edges of an edge or wire tool lying on
//   them, or the section of a tool with faces) imprinted with BRepAlgoAPI_Splitter; solids are
//   not split.

#include "amg-occt-sys/cpp/occt_history.hxx"
#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <BOPAlgo_Alerts.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Draft_ErrorStatus.hxx>
#include <GProp_GProps.hxx>
#include <GeomAPI_IntCS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Line.hxx>
#include <Geom_Surface.hxx>
#include <Message_Alert.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <Message_Report.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_AlertWithShape.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

namespace amg::occt {

namespace {

using namespace history;

// An error that amg-brep reports as `InvalidInput` (same prefix as the other op files).
constexpr const char *INVALID = "invalid: ";

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

class LocalProgress : public Message_ProgressIndicator {
public:
  explicit LocalProgress(const ProgressSink &sink) : sink_(sink) {}

  bool UserBreak() override { return progress_user_break(sink_); }

  void Show(const Message_ProgressScope &, const bool) override {
    progress_show(sink_, GetPosition());
  }

private:
  const ProgressSink &sink_;
};

gp_Pnt pnt(const V3 &v) { return gp_Pnt(v.x, v.y, v.z); }
gp_Dir dir(const V3 &v) { return gp_Dir(v.x, v.y, v.z); }

// The input's sub-shapes and their copies, in the input's index order.
struct Copied {
  BRepBuilderAPI_Copy copier;
  TopoDS_Shape base;
  ShapeMap faces; // copy of input face i at index i + 1

  Copied(const TopoDS_Shape &input, const OcIndex &index) : copier(input, false, false) {
    base = copier.Shape();
    for (int i = 1; i <= index.faces.Extent(); ++i) {
      faces.Add(image(index.faces(i)));
    }
  }

  // The copy of `s` in the orientation of `s` (BREP-07: Modified drops the orientation).
  TopoDS_Shape image(const TopoDS_Shape &s) const {
    const NCollection_List<TopoDS_Shape> &images =
        const_cast<BRepBuilderAPI_Copy &>(copier).Modified(s);
    if (images.IsEmpty()) {
      fail("local op: an input sub-shape has no copy");
    }
    TopoDS_Shape out = images.First();
    out.Orientation(s.Orientation());
    return out;
  }

  const TopoDS_Face &face(std::uint32_t f) const {
    if (f >= static_cast<std::uint32_t>(faces.Extent())) {
      fail("local op: face index out of range");
    }
    return TopoDS::Face(faces(static_cast<int>(f) + 1));
  }

  // The input face index of a copied face; -1 when it is none.
  int face_index(const TopoDS_Shape &s) const { return faces.FindIndex(s) - 1; }
};

void push_offending_face(OcOp &op, std::uint32_t face) {
  for (std::size_t j = 0; j < op.offending_indices.size(); ++j) {
    if (op.offending_kinds[j] == 0 && op.offending_indices[j] == face) {
      return;
    }
  }
  op.offending_operands.push_back(0);
  op.offending_kinds.push_back(0);
  op.offending_indices.push_back(face);
}

// Every copied face inside `shape` becomes offending.
void offending_faces_in(OcOp &op, const Copied &c, const TopoDS_Shape &shape) {
  if (shape.IsNull()) {
    return;
  }
  ShapeMap faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  for (int i = 1; i <= faces.Extent(); ++i) {
    const int f = c.face_index(faces(i));
    if (f >= 0) {
      push_offending_face(op, static_cast<std::uint32_t>(f));
    }
  }
}

// Alert names (deduplicated) of a BOPAlgo-based algorithm, and the copied faces they name.
void collect_alerts(const occ::handle<Message_Report> &report, Message_Gravity gravity,
                    std::vector<std::string> &names, OcOp *op, const Copied *c) {
  if (report.IsNull()) {
    return;
  }
  for (const occ::handle<Message_Alert> &alert : report->GetAlerts(gravity)) {
    if (alert.IsNull()) {
      continue;
    }
    const std::string name = alert->GetMessageKey();
    if (std::find(names.begin(), names.end(), name) == names.end()) {
      names.push_back(name);
    }
    if (op != nullptr && c != nullptr) {
      occ::handle<TopoDS_AlertWithShape> with_shape = occ::down_cast<TopoDS_AlertWithShape>(alert);
      if (!with_shape.IsNull()) {
        offending_faces_in(*op, *c, with_shape->GetShape());
      }
    }
  }
}

int solid_count(const TopoDS_Shape &s) {
  ShapeMap solids;
  TopExp::MapShapes(s, TopAbs_SOLID, solids);
  return solids.Extent();
}

// The one child of a compound with one child, else `s`.
TopoDS_Shape unwrap(const TopoDS_Shape &s) {
  if (s.IsNull() || s.ShapeType() != TopAbs_COMPOUND) {
    return s;
  }
  TopoDS_Iterator it(s);
  if (!it.More()) {
    return s;
  }
  TopoDS_Shape first = it.Value();
  it.Next();
  return it.More() ? s : first;
}

// The outward unit normal of `face` at (u, v); false when it is not defined there.
bool outward_normal(const TopoDS_Face &face, double u, double v, gp_Dir &out) {
  const BRepAdaptor_Surface surface(face, false);
  BRepLProp_SLProps props(surface, u, v, 1, 1e-9);
  if (!props.IsNormalDefined()) {
    return false;
  }
  out = props.Normal();
  if (face.Orientation() == TopAbs_REVERSED) {
    out.Reverse();
  }
  return true;
}

// Sample points of `face` with their outward normals: its vertices and the cell centres of a
// `n` × `n` grid of its parameter box that lie inside it.
struct Sample {
  gp_Pnt p;
  gp_Dir n;
};

std::vector<Sample> samples_of(const TopoDS_Face &face, int n) {
  std::vector<Sample> out;
  const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(face);
  double u0 = 0.0;
  double u1 = 0.0;
  double v0 = 0.0;
  double v1 = 0.0;
  BRepTools::UVBounds(face, u0, u1, v0, v1);
  const double tol = BRep_Tool::Tolerance(face);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double u = u0 + (u1 - u0) * (i + 0.5) / n;
      const double v = v0 + (v1 - v0) * (j + 0.5) / n;
      BRepClass_FaceClassifier inside(face, gp_Pnt2d(u, v), tol);
      if (inside.State() != TopAbs_IN) {
        continue;
      }
      gp_Dir normal;
      if (outward_normal(face, u, v, normal)) {
        out.push_back({surface->Value(u, v), normal});
      }
    }
  }
  ShapeMap vertices;
  TopExp::MapShapes(face, TopAbs_VERTEX, vertices);
  for (int i = 1; i <= vertices.Extent(); ++i) {
    const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vertices(i)));
    GeomAPI_ProjectPointOnSurf project(p, surface);
    if (project.NbPoints() == 0) {
      continue;
    }
    double u = 0.0;
    double v = 0.0;
    project.LowerDistanceParameters(u, v);
    gp_Dir normal;
    if (outward_normal(face, u, v, normal)) {
      out.push_back({p, normal});
    }
  }
  return out;
}

// The signed distance along `n` from `p` to the nearest point where the line through `p` along
// `n` meets `surface`; NaN when it does not.
double ray_hit(const gp_Pnt &p, const gp_Dir &n, const occ::handle<Geom_Surface> &surface,
               gp_Pnt *hit = nullptr) {
  occ::handle<Geom_Line> line = new Geom_Line(gp_Lin(p, n));
  GeomAPI_IntCS intersect(line, surface);
  double best = std::numeric_limits<double>::quiet_NaN();
  if (!intersect.IsDone()) {
    return best;
  }
  for (int i = 1; i <= intersect.NbPoints(); ++i) {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    intersect.Parameters(i, u, v, w);
    if (std::isnan(best) || std::abs(w) < std::abs(best)) {
      best = w;
      if (hit != nullptr) {
        *hit = intersect.Point(i);
      }
    }
  }
  return best;
}

// The unit normal of `surface` at its point nearest `p`; false when it has none there.
bool surface_normal_near(const occ::handle<Geom_Surface> &surface, const gp_Pnt &p, gp_Dir &out) {
  GeomAPI_ProjectPointOnSurf project(p, surface);
  if (project.NbPoints() == 0) {
    return false;
  }
  double u = 0.0;
  double v = 0.0;
  project.LowerDistanceParameters(u, v);
  gp_Pnt q;
  gp_Vec du;
  gp_Vec dv;
  surface->D1(u, v, q, du, dv);
  const gp_Vec n = du.Crossed(dv);
  if (n.Magnitude() < 1e-12) {
    return false;
  }
  out = gp_Dir(n);
  return true;
}

const char *draft_error(Draft_ErrorStatus status) {
  switch (status) {
  case Draft_NoError:
    return "Draft_NoError";
  case Draft_FaceRecomputation:
    return "Draft_FaceRecomputation";
  case Draft_EdgeRecomputation:
    return "Draft_EdgeRecomputation";
  case Draft_VertexRecomputation:
    return "Draft_VertexRecomputation";
  default:
    return "Draft_UnknownError";
  }
}

// A source through the copy: each copied sub-shape goes to its images in `modified` (itself
// when it has none and is not `deleted`) and `generated`.
template <typename M, typename G, typename D>
Source through_copy(const Copied &c, M modified, G generated, D deleted) {
  return Source{[&c, modified, deleted](const TopoDS_Shape &s) -> Shapes {
                  const TopoDS_Shape copy = c.image(s);
                  Shapes images = modified(copy);
                  if (images.empty() && !deleted(copy)) {
                    return {copy};
                  }
                  return images;
                },
                [&c, generated](const TopoDS_Shape &s) -> Shapes {
                  return generated(c.image(s));
                }};
}

} // namespace

// -- draft --

std::unique_ptr<OcOp> draft_op(const OcShape &shape, const OcIndex &index,
                               rust::Slice<const std::uint32_t> faces,
                               const OcDraftParams &params, const ProgressSink &sink) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("draft: the shape is null");
  }
  auto op = std::make_unique<OcOp>();
  Copied c(input, index);
  const gp_Dir pull = dir(params.pull);
  const gp_Pln neutral(gp_Ax3(pnt(params.neutral.origin), dir(params.neutral.normal),
                              dir(params.neutral.x)));
  BRepOffsetAPI_DraftAngle draft(c.base);
  for (std::uint32_t f : faces) {
    const TopoDS_Face &face = c.face(f);
    const GeomAbs_SurfaceType type = BRepAdaptor_Surface(face, false).GetType();
    if (type != GeomAbs_Plane && type != GeomAbs_Cylinder && type != GeomAbs_Cone) {
      op->errors.push_back(std::string(INVALID) + "face " + std::to_string(f) +
                           " is not planar, cylindrical or conical");
      push_offending_face(*op, f);
      return op;
    }
    try {
      draft.Add(face, pull, params.angle, neutral, true);
    } catch (const Standard_Failure &failure) {
      op->errors.push_back("face " + std::to_string(f) + ": " + format_failure(failure));
      push_offending_face(*op, f);
      return op;
    }
    if (!draft.AddDone()) {
      op->errors.push_back("face " + std::to_string(f) + ": " + draft_error(draft.Status()));
      push_offending_face(*op, f);
      try {
        offending_faces_in(*op, c, draft.ProblematicShape());
      } catch (const Standard_Failure &) {
      }
      return op;
    }
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  occ::handle<LocalProgress> indicator = new LocalProgress(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg draft", 1);
    draft.Build(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  if (op->errors.empty() && !draft.IsDone()) {
    op->errors.emplace_back(draft_error(draft.Status()));
    try {
      offending_faces_in(*op, c, draft.ProblematicShape());
    } catch (const Standard_Failure &) {
    }
  }
  if (!op->errors.empty()) {
    return op;
  }
  op->shape = draft.Shape();
  if (op->shape.IsNull()) {
    op->errors.emplace_back("the result is empty");
    return op;
  }
  if (solid_count(op->shape) != solid_count(c.base)) {
    op->errors.emplace_back("the draft changed the number of solids");
    return op;
  }
  BRepOffsetAPI_DraftAngle *d = &draft;
  // As the taper prism (BREP-05): `Modified` lists only some faces; the tilted faces come back
  // through `ModifiedShape`.
  const auto modified = [d](const TopoDS_Shape &copy) -> Shapes {
    const NCollection_List<TopoDS_Shape> &images = d->Modified(copy);
    if (!images.IsEmpty()) {
      return shapes_of(images);
    }
    try {
      const TopoDS_Shape image = d->ModifiedShape(copy);
      if (!image.IsNull()) {
        return {image};
      }
    } catch (const Standard_Failure &) {
    }
    return {};
  };
  const auto generated = [d](const TopoDS_Shape &copy) -> Shapes {
    return shapes_of(d->Generated(copy));
  };
  const auto deleted = [](const TopoDS_Shape &) { return false; };
  record_operands(*op, {input}, through_copy(c, modified, generated, deleted), {}, false);
  return op;
}

// -- defeature --

std::unique_ptr<OcOp> defeature_op(const OcShape &shape, const OcIndex &index,
                                   rust::Slice<const std::uint32_t> faces, bool parallel,
                                   const ProgressSink &sink) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("defeature: the shape is null");
  }
  auto op = std::make_unique<OcOp>();
  Copied c(input, index);
  NCollection_List<TopoDS_Shape> remove;
  for (std::uint32_t f : faces) {
    remove.Append(c.face(f));
  }
  BRepAlgoAPI_Defeaturing algo;
  algo.SetShape(c.base);
  algo.AddFacesToRemove(remove);
  algo.SetRunParallel(parallel);
  algo.SetToFillHistory(true);
  occ::handle<LocalProgress> indicator = new LocalProgress(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg defeature", 1);
    algo.Build(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  std::vector<std::string> names;
  collect_alerts(algo.GetReport(), Message_Fail, names, op.get(), &c);
  // A feature the algorithm cannot remove is skipped with a warning: here it is a failure.
  std::vector<std::string> warnings;
  {
    const occ::handle<Message_Report> &report = algo.GetReport();
    if (!report.IsNull()) {
      for (const occ::handle<Message_Alert> &alert : report->GetAlerts(Message_Warning)) {
        if (alert.IsNull()) {
          continue;
        }
        const std::string name = alert->GetMessageKey();
        if (name.find("UnableToRemoveTheFeature") != std::string::npos) {
          if (std::find(names.begin(), names.end(), name) == names.end()) {
            names.push_back(name);
          }
          occ::handle<TopoDS_AlertWithShape> with_shape =
              occ::down_cast<TopoDS_AlertWithShape>(alert);
          if (!with_shape.IsNull()) {
            offending_faces_in(*op, c, with_shape->GetShape());
          }
        } else if (std::find(warnings.begin(), warnings.end(), name) == warnings.end()) {
          warnings.push_back(name);
        }
      }
    }
  }
  for (std::string &name : names) {
    op->errors.push_back(std::move(name));
  }
  if (op->errors.empty() && !algo.IsDone()) {
    op->errors.emplace_back("BRepAlgoAPI_Defeaturing: NotDone");
  }
  if (!op->errors.empty()) {
    if (op->offending_indices.empty()) {
      for (std::uint32_t f : faces) {
        push_offending_face(*op, f);
      }
    }
    return op;
  }
  op->shape = algo.Shape();
  if (op->shape.IsNull() || solid_count(op->shape) == 0) {
    op->errors.emplace_back("the result has no solid");
    op->shape = TopoDS_Shape();
    return op;
  }
  for (std::string &w : warnings) {
    op->warnings.push_back(std::move(w));
  }
  BRepAlgoAPI_Defeaturing *a = &algo;
  const auto modified = [a](const TopoDS_Shape &copy) -> Shapes {
    return shapes_of(a->Modified(copy));
  };
  const auto generated = [a](const TopoDS_Shape &copy) -> Shapes {
    return shapes_of(a->Generated(copy));
  };
  const auto deleted = [a](const TopoDS_Shape &copy) { return a->IsDeleted(copy); };
  record_operands(*op, {input}, through_copy(c, modified, generated, deleted), {}, false);
  return op;
}

// -- replace face --

double face_reach(const OcIndex &index, std::uint32_t face, const OcShape &tool) {
  if (face >= static_cast<std::uint32_t>(index.faces.Extent())) {
    fail("face_reach: face index out of range");
  }
  const TopoDS_Shape &t = tool.get();
  if (t.IsNull() || t.ShapeType() != TopAbs_FACE) {
    fail("face_reach: the tool is not a face");
  }
  const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(TopoDS::Face(t));
  const TopoDS_Face &f = TopoDS::Face(index.faces(static_cast<int>(face) + 1));
  double reach = std::numeric_limits<double>::quiet_NaN();
  for (const Sample &s : samples_of(f, 5)) {
    const double w = ray_hit(s.p, s.n, surface);
    if (std::isnan(w)) {
      continue;
    }
    reach = std::isnan(reach) ? w : std::max(reach, w);
  }
  return reach;
}

std::unique_ptr<OcOp> trim_face_op(const OcShape &shape, const OcIndex &index, std::uint32_t face,
                                   const OcShape &tool, const ProgressSink &sink) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("replace_face: the shape is null");
  }
  const TopoDS_Shape &tool_face = tool.get();
  if (tool_face.IsNull() || tool_face.ShapeType() != TopAbs_FACE) {
    fail("replace_face: the tool is not a face");
  }
  auto op = std::make_unique<OcOp>();
  Copied c(input, index);
  const TopoDS_Face &old_face = c.face(face);
  const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(TopoDS::Face(tool_face));

  // The side of the new surface the material stays on: against the replaced face's outward
  // normal where the normals from its samples meet the surface (`away` is the surface normal
  // turned to point out of the material there).
  double sense = 0.0;
  for (const Sample &s : samples_of(old_face, 5)) {
    gp_Pnt hit;
    if (std::isnan(ray_hit(s.p, s.n, surface, &hit))) {
      continue;
    }
    gp_Dir ns;
    if (surface_normal_near(surface, hit, ns)) {
      sense = ns.Dot(s.n) >= 0.0 ? 1.0 : -1.0;
      break;
    }
  }
  if (sense == 0.0) {
    op->errors.push_back(std::string(INVALID) +
                         "the new surface does not lie across the face's normals");
    push_offending_face(*op, face);
    return op;
  }

  BRepAlgoAPI_Splitter splitter;
  NCollection_List<TopoDS_Shape> args;
  args.Append(c.base);
  NCollection_List<TopoDS_Shape> tools;
  tools.Append(tool_face);
  splitter.SetArguments(args);
  splitter.SetTools(tools);
  splitter.SetNonDestructive(true);
  occ::handle<LocalProgress> indicator = new LocalProgress(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg replace face", 1);
    splitter.Build(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  std::vector<std::string> names;
  collect_alerts(splitter.GetReport(), Message_Fail, names, nullptr, nullptr);
  for (std::string &name : names) {
    op->errors.push_back(std::move(name));
  }
  if (op->errors.empty() && !splitter.IsDone()) {
    op->errors.emplace_back("BRepAlgoAPI_Splitter: NotDone");
  }
  if (!op->errors.empty()) {
    return op;
  }

  // The faces on the new surface: images of the tool face.
  ShapeMap on_surface;
  for (const TopoDS_Shape &s : splitter.Modified(tool_face)) {
    on_surface.Add(s);
  }
  on_surface.Add(tool_face);
  const double tol = 1e-7;
  const auto keep = [&](const TopoDS_Shape &solid) -> bool {
    ShapeMap solid_faces;
    TopExp::MapShapes(solid, TopAbs_FACE, solid_faces);
    for (int i = 1; i <= solid_faces.Extent(); ++i) {
      const TopoDS_Face &f = TopoDS::Face(solid_faces(i));
      if (!on_surface.Contains(f)) {
        continue;
      }
      double u0 = 0.0;
      double u1 = 0.0;
      double v0 = 0.0;
      double v1 = 0.0;
      BRepTools::UVBounds(f, u0, u1, v0, v1);
      const double u = 0.5 * (u0 + u1);
      const double v = 0.5 * (v0 + v1);
      gp_Dir out;
      gp_Dir ns;
      const gp_Pnt p = BRep_Tool::Surface(f)->Value(u, v);
      if (outward_normal(f, u, v, out) && surface_normal_near(surface, p, ns)) {
        // The piece is on the material side when its face on the surface looks away.
        return out.Dot(ns) * sense > 0.0;
      }
    }
    // A piece the surface does not bound: by the side of a vertex off the surface.
    ShapeMap vertices;
    TopExp::MapShapes(solid, TopAbs_VERTEX, vertices);
    for (int i = 1; i <= vertices.Extent(); ++i) {
      const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vertices(i)));
      GeomAPI_ProjectPointOnSurf project(p, surface);
      if (project.NbPoints() == 0 || project.LowerDistance() <= tol) {
        continue;
      }
      gp_Dir ns;
      if (!surface_normal_near(surface, p, ns)) {
        continue;
      }
      return gp_Vec(project.NearestPoint(), p).Dot(gp_Vec(ns)) * sense < 0.0;
    }
    return true;
  };
  ShapeMap solids;
  TopExp::MapShapes(splitter.Shape(), TopAbs_SOLID, solids);
  std::vector<TopoDS_Shape> kept;
  for (int i = 1; i <= solids.Extent(); ++i) {
    if (keep(solids(i))) {
      kept.push_back(solids(i));
    }
  }
  if (kept.empty()) {
    op->errors.emplace_back("nothing is left on the material side of the new surface");
    push_offending_face(*op, face);
    return op;
  }
  if (kept.size() == 1) {
    op->shape = kept.front();
  } else {
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (const TopoDS_Shape &s : kept) {
      builder.Add(compound, s);
    }
    op->shape = compound;
  }
  const Maps out(op->shape);
  Shapes new_faces; // the faces on the new surface that are in the result
  for (int i = 1; i <= on_surface.Extent(); ++i) {
    if (out.m[0].Contains(on_surface(i))) {
      new_faces.push_back(on_surface(i));
    }
  }
  // An edge of the replaced face that is gone goes to the result edges between the new faces
  // and the images of its other face.
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
      in_edge_faces;
  TopExp::MapShapesAndUniqueAncestors(c.base, TopAbs_EDGE, TopAbs_FACE, in_edge_faces);
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
      out_edge_faces;
  TopExp::MapShapesAndUniqueAncestors(op->shape, TopAbs_EDGE, TopAbs_FACE, out_edge_faces);
  ShapeMap new_face_map;
  for (const TopoDS_Shape &f : new_faces) {
    new_face_map.Add(f);
  }
  BRepAlgoAPI_Splitter *sp = &splitter;
  const auto images_of = [sp](const TopoDS_Shape &copy) -> Shapes {
    Shapes images = shapes_of(sp->Modified(copy));
    if (images.empty() && !sp->IsDeleted(copy)) {
      images.push_back(copy);
    }
    return images;
  };
  const auto modified = [&](const TopoDS_Shape &copy) -> Shapes {
    Shapes images = images_of(copy);
    if (copy.IsSame(old_face)) {
      for (const TopoDS_Shape &f : new_faces) {
        images.push_back(f);
      }
      return images;
    }
    if (copy.ShapeType() != TopAbs_EDGE) {
      return images;
    }
    for (const TopoDS_Shape &image : images) {
      if (out.m[1].Contains(image)) {
        return images;
      }
    }
    const int e = in_edge_faces.FindIndex(copy);
    if (e == 0) {
      return images;
    }
    bool on_old = false;
    ShapeMap others;
    for (const TopoDS_Shape &f : in_edge_faces(e)) {
      if (f.IsSame(old_face)) {
        on_old = true;
      } else {
        for (const TopoDS_Shape &image : images_of(f)) {
          others.Add(image);
        }
      }
    }
    if (!on_old) {
      return images;
    }
    for (int i = 1; i <= out_edge_faces.Extent(); ++i) {
      bool by_new = false;
      bool by_other = false;
      for (const TopoDS_Shape &f : out_edge_faces(i)) {
        by_new = by_new || new_face_map.Contains(f);
        by_other = by_other || others.Contains(f);
      }
      if (by_new && by_other) {
        images.push_back(out_edge_faces.FindKey(i));
      }
    }
    return images;
  };
  const auto generated = [sp](const TopoDS_Shape &copy) -> Shapes {
    return shapes_of(sp->Generated(copy));
  };
  const auto deleted = [](const TopoDS_Shape &) { return true; };
  record_operands(*op, {input}, through_copy(c, modified, generated, deleted), {}, false);
  return op;
}

// -- split faces --

std::unique_ptr<OcOp> split_faces_op(const OcShape &shape, const OcIndex &index,
                                     rust::Slice<const std::uint32_t> faces, const OcShape &tool,
                                     const ProgressSink &sink) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("split_faces: the shape is null");
  }
  if (tool.get().IsNull()) {
    fail("split_faces: the tool is null");
  }
  auto op = std::make_unique<OcOp>();
  Copied c(input, index);
  BRepBuilderAPI_Copy tool_copier(tool.get(), false, false);
  const TopoDS_Shape tool_copy = tool_copier.Shape();

  BRep_Builder builder;
  TopoDS_Compound selected;
  builder.MakeCompound(selected);
  for (std::uint32_t f : faces) {
    builder.Add(selected, c.face(f));
  }

  // The tool's curves on the chosen faces.
  std::vector<TopoDS_Shape> curves;
  ShapeMap tool_faces;
  TopExp::MapShapes(tool_copy, TopAbs_FACE, tool_faces);
  try {
    if (tool_faces.Extent() > 0) {
      BRepAlgoAPI_Section section(selected, tool_copy, false);
      section.ComputePCurveOn1(true);
      section.Approximation(true);
      section.Build();
      if (!section.IsDone()) {
        op->errors.emplace_back("BRepAlgoAPI_Section: NotDone");
        return op;
      }
      ShapeMap edges;
      TopExp::MapShapes(section.Shape(), TopAbs_EDGE, edges);
      for (int i = 1; i <= edges.Extent(); ++i) {
        curves.push_back(edges(i));
      }
    } else {
      // Tool edges split where they cross the faces' boundaries, kept where they lie on a face.
      BRepAlgoAPI_Splitter cut;
      NCollection_List<TopoDS_Shape> args;
      args.Append(tool_copy);
      NCollection_List<TopoDS_Shape> by;
      by.Append(selected);
      cut.SetArguments(args);
      cut.SetTools(by);
      cut.Build();
      const TopoDS_Shape pieces = cut.IsDone() ? cut.Shape() : tool_copy;
      ShapeMap edges;
      TopExp::MapShapes(pieces, TopAbs_EDGE, edges);
      for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge &e = TopoDS::Edge(edges(i));
        if (BRep_Tool::Degenerated(e)) {
          continue;
        }
        double t0 = 0.0;
        double t1 = 0.0;
        const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(e, t0, t1);
        if (curve.IsNull()) {
          continue;
        }
        const TopoDS_Vertex mid = BRepBuilderAPI_MakeVertex(curve->Value(0.5 * (t0 + t1)));
        BRepExtrema_DistShapeShape dist(mid, selected);
        const double tol = std::max(1e-5, 2.0 * BRep_Tool::Tolerance(e));
        if (dist.IsDone() && dist.NbSolution() > 0 && dist.Value() <= tol) {
          curves.push_back(e);
        }
      }
    }
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
    return op;
  }
  if (curves.empty()) {
    op->errors.push_back(std::string(INVALID) + "the tool does not meet the faces");
    for (std::uint32_t f : faces) {
      push_offending_face(*op, f);
    }
    return op;
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }

  BRepAlgoAPI_Splitter splitter;
  NCollection_List<TopoDS_Shape> args;
  args.Append(c.base);
  NCollection_List<TopoDS_Shape> tools;
  for (const TopoDS_Shape &e : curves) {
    tools.Append(e);
  }
  splitter.SetArguments(args);
  splitter.SetTools(tools);
  splitter.SetNonDestructive(true);
  occ::handle<LocalProgress> indicator = new LocalProgress(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg split faces", 1);
    splitter.Build(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  std::vector<std::string> names;
  collect_alerts(splitter.GetReport(), Message_Fail, names, nullptr, nullptr);
  for (std::string &name : names) {
    op->errors.push_back(std::move(name));
  }
  if (op->errors.empty() && !splitter.IsDone()) {
    op->errors.emplace_back("BRepAlgoAPI_Splitter: NotDone");
  }
  if (!op->errors.empty()) {
    return op;
  }
  op->shape = unwrap(splitter.Shape());
  // The pieces are only those of the chosen faces: a face that is not chosen and was split has
  // the tool on it as well, which the caller did not ask for.
  ShapeMap chosen;
  for (std::uint32_t f : faces) {
    chosen.Add(c.face(f));
  }
  bool any_split = false;
  for (int i = 1; i <= c.faces.Extent(); ++i) {
    const TopoDS_Shape &f = c.faces(i);
    const int pieces = splitter.Modified(f).Extent();
    if (pieces > 1) {
      if (chosen.Contains(f)) {
        any_split = true;
      } else {
        op->warnings.push_back("face " + std::to_string(i - 1) + " was split as well");
      }
    }
  }
  if (!any_split) {
    op->errors.push_back(std::string(INVALID) + "the tool does not split the faces");
    for (std::uint32_t f : faces) {
      push_offending_face(*op, f);
    }
    op->shape = TopoDS_Shape();
    return op;
  }
  BRepAlgoAPI_Splitter *sp = &splitter;
  const auto modified = [sp](const TopoDS_Shape &copy) -> Shapes {
    return shapes_of(sp->Modified(copy));
  };
  const auto generated = [sp](const TopoDS_Shape &copy) -> Shapes {
    return shapes_of(sp->Generated(copy));
  };
  const auto deleted = [sp](const TopoDS_Shape &copy) { return sp->IsDeleted(copy); };
  record_operands(*op, {input}, through_copy(c, modified, generated, deleted), {}, false);
  return op;
}

} // namespace amg::occt
