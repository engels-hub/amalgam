// Pipe-shell sweeps and the raw ThruSections loft (S02 §5, BREP-15). Declarations and categories
// are in occt_shim.hxx.
//
// Both run on private BRepBuilderAPI_Copy copies of their operands (category C): the sweep and
// the loft use the section edges as boundary edges of the faces they build and add pcurves to
// them. An edge operand is made into a one-edge wire of its copy (never of the caller's edge:
// TopoDS_Builder::Add would set the caller's TShape flags). The history is reported from every
// operand through its copy, in amg-brep's index of the caller's shapes.
//
// Inputs are validated in amg-brep (`sweep.rs`); a bad input found here is an error starting with
// `invalid: `, an OCCT failure any other error, both reported through `op_bool_status`.

#include "amg-occt-sys/cpp/occt_history.hxx"
#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Adaptor3d_Curve.hxx>
#include <BRepAdaptor_CompCurve.hxx>
#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_PipeError.hxx>
#include <BRepBuilderAPI_TransitionMode.hxx>
#include <BRepFill_TypeOfContact.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <Bnd_Box.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomFill_ConstantBiNormal.hxx>
#include <GeomFill_CorrectedFrenet.hxx>
#include <GeomFill_Frenet.hxx>
#include <GeomFill_TrihedronLaw.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Law_Function.hxx>
#include <Law_Interpol.hxx>
#include <Law_Linear.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_HArray1.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

namespace amg::occt {

namespace {

using namespace history;

// An error that amg-brep reports as `InvalidInput` (same prefix as occt_blend.cpp).
constexpr const char *INVALID = "invalid: ";

// OcPipeParams::mode.
constexpr std::uint8_t MODE_CORRECTED_FRENET = 0;
constexpr std::uint8_t MODE_FRENET = 1;
constexpr std::uint8_t MODE_FIXED = 2;
constexpr std::uint8_t MODE_BINORMAL = 3;
constexpr std::uint8_t MODE_GUIDE = 4;
constexpr std::uint8_t MODE_SUPPORT = 5;

// OcLoftSection::wire of a point section.
constexpr std::uint32_t POINT_SECTION = UINT32_MAX;

// Samples of the twist guide per spine edge, and at least this many in all.
constexpr int TWIST_SAMPLES_PER_EDGE = 32;
constexpr int TWIST_SAMPLES_MIN = 64;

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

gp_Pnt pnt(const V3 &v) { return gp_Pnt(v.x, v.y, v.z); }
gp_Dir dir(const V3 &v) { return gp_Dir(v.x, v.y, v.z); }

class SweepProgress : public Message_ProgressIndicator {
public:
  explicit SweepProgress(const ProgressSink &sink) : sink_(sink) {}

  bool UserBreak() override { return progress_user_break(sink_); }

  void Show(const Message_ProgressScope &, const bool) override {
    progress_show(sink_, GetPosition());
  }

private:
  const ProgressSink &sink_;
};

// One operand's private copy. `shape` is what the algorithm gets: the copy, or a one-edge wire
// of the copied edge.
struct Operand {
  std::unique_ptr<BRepBuilderAPI_Copy> copier;
  TopoDS_Shape shape;
  // The copy that holds this operand's sub-shapes instead of `copier` (a support's spine).
  BRepBuilderAPI_Copy *alias = nullptr;

  // Where the copy put sub-shape `s` of the caller's operand (null if nowhere).
  TopoDS_Shape copy_of(const TopoDS_Shape &s) const {
    const NCollection_List<TopoDS_Shape> &images = (alias ? alias : copier.get())->Modified(s);
    return images.IsEmpty() ? TopoDS_Shape() : images.First();
  }
};

// The spine rebuilt from the support's copy, when every spine edge is an edge of the support
// (OCCT's support mode needs the spine edges' pcurves on the support faces, which only the
// support's own edges carry): `spine` then aliases `support`. False when an edge is not one of
// the support's.
bool spine_on_support(Operand &spine, const TopoDS_Shape &input, const Operand &support) {
  BRepBuilderAPI_MakeWire wire;
  ShapeMap edges;
  TopExp::MapShapes(input, TopAbs_EDGE, edges);
  for (int e = 1; e <= edges.Extent(); ++e) {
    TopoDS_Shape image;
    try {
      // Raises Standard_NoSuchObject for a shape that is not in the support.
      const NCollection_List<TopoDS_Shape> &images = support.copier->Modified(edges(e));
      if (!images.IsEmpty()) {
        image = images.First();
      }
    } catch (const Standard_Failure &) {
    }
    if (image.IsNull()) {
      return false;
    }
    wire.Add(TopoDS::Edge(image));
    if (!wire.IsDone()) {
      return false;
    }
  }
  spine.shape = wire.Wire();
  spine.alias = support.copier.get();
  return true;
}

// Copies `input`; an edge becomes a one-edge wire of its copy when `as_wire`.
Operand copy_operand(const TopoDS_Shape &input, bool as_wire) {
  if (input.IsNull()) {
    fail("sweep: an operand is null");
  }
  Operand o;
  o.copier = std::make_unique<BRepBuilderAPI_Copy>(input, false, false);
  o.shape = o.copier->Shape();
  if (as_wire && o.shape.ShapeType() == TopAbs_EDGE) {
    BRepBuilderAPI_MakeWire wire(TopoDS::Edge(o.shape));
    if (!wire.IsDone()) {
      fail("sweep: cannot make a wire of an edge operand");
    }
    o.shape = wire.Wire();
  }
  return o;
}

// The caller's operands and their copies.
std::vector<Operand> copy_all(const OcShapeList &list, const std::vector<bool> &as_wire) {
  std::vector<Operand> out;
  for (std::size_t i = 0; i < list.shapes.size(); ++i) {
    out.push_back(copy_operand(list.shapes[i], as_wire[i]));
  }
  return out;
}

// A law through `(t, value)` pairs, `t` in [0, 1] mapped onto [first, last].
occ::handle<Law_Function> make_law(rust::Slice<const double> pairs, double first, double last) {
  const std::size_t n = pairs.size() / 2;
  if (n < 2 || pairs.size() % 2 != 0) {
    fail("sweep: a law needs at least two (t, value) samples");
  }
  const auto at = [&](std::size_t k) { return first + pairs[2 * k] * (last - first); };
  if (n == 2) {
    occ::handle<Law_Linear> law = new Law_Linear();
    law->Set(at(0), pairs[1], at(1), pairs[3]);
    return law;
  }
  NCollection_Array1<gp_Pnt2d> points(1, static_cast<int>(n));
  for (std::size_t k = 0; k < n; ++k) {
    points(static_cast<int>(k) + 1) = gp_Pnt2d(at(k), pairs[2 * k + 1]);
  }
  occ::handle<Law_Interpol> law = new Law_Interpol();
  law->Set(points, false);
  return law;
}

// Linear interpolation of the twist samples at `t` in [0, 1] (piecewise linear between samples;
// the guide curve through the sampled points then smooths it).
double twist_at(rust::Slice<const double> pairs, double t) {
  const std::size_t n = pairs.size() / 2;
  if (t <= pairs[0]) {
    return pairs[1];
  }
  for (std::size_t k = 1; k < n; ++k) {
    const double t0 = pairs[2 * (k - 1)];
    const double t1 = pairs[2 * k];
    if (t <= t1) {
      const double a = t1 > t0 ? (t - t0) / (t1 - t0) : 1.0;
      return pairs[2 * k - 1] + a * (pairs[2 * k + 1] - pairs[2 * k - 1]);
    }
  }
  return pairs[2 * n - 1];
}

// The trihedron law of a twisted sweep's base mode.
occ::handle<GeomFill_TrihedronLaw> base_trihedron(const OcPipeParams &p) {
  switch (p.mode) {
  case MODE_FRENET:
    return new GeomFill_Frenet();
  case MODE_BINORMAL:
    return new GeomFill_ConstantBiNormal(dir(p.binormal));
  default:
    return new GeomFill_CorrectedFrenet();
  }
}

// Half the diagonal of the profiles' bounding box: the distance of the twist guide from the
// spine.
double guide_radius(const std::vector<Operand> &operands, std::size_t profiles) {
  Bnd_Box box;
  for (std::size_t i = 0; i < profiles; ++i) {
    BRepBndLib::Add(operands[i].shape, box);
  }
  if (box.IsVoid()) {
    return 1.0;
  }
  return std::max(0.5 * std::sqrt(box.SquareExtent()), 1e-3);
}

// The auxiliary spine that realises a twist law: at each spine sample, the point at `radius`
// from the spine in the base trihedron's normal plane, at angle twist(t) from its normal. The
// guide trihedron (plane mode) then points the section's normal at it, which turns the section
// by twist(t) − twist(0) about the tangent relative to the base mode.
TopoDS_Wire twist_guide(const TopoDS_Wire &spine, const OcPipeParams &p,
                        rust::Slice<const double> twist, double radius) {
  occ::handle<BRepAdaptor_CompCurve> curve = new BRepAdaptor_CompCurve(spine, true);
  occ::handle<GeomFill_TrihedronLaw> law = base_trihedron(p);
  law->SetCurve(curve);
  ShapeMap edges;
  TopExp::MapShapes(spine, TopAbs_EDGE, edges);
  const int n = std::max(TWIST_SAMPLES_MIN, TWIST_SAMPLES_PER_EDGE * edges.Extent());
  const double first = curve->FirstParameter();
  const double last = curve->LastParameter();
  occ::handle<NCollection_HArray1<gp_Pnt>> points = new NCollection_HArray1<gp_Pnt>(1, n + 1);
  occ::handle<NCollection_HArray1<double>> params = new NCollection_HArray1<double>(1, n + 1);
  for (int k = 0; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    const double s = first + t * (last - first);
    gp_Vec tangent;
    gp_Vec normal;
    gp_Vec binormal;
    if (!law->D0(s, tangent, normal, binormal)) {
      fail("pipe: the base trihedron is undefined along the spine (twist law)");
    }
    const double a = twist_at(twist, t);
    const gp_Vec offset = (normal * std::cos(a) + binormal * std::sin(a)) * radius;
    points->SetValue(k + 1, curve->Value(s).Translated(offset));
    params->SetValue(k + 1, s);
  }
  GeomAPI_Interpolate interpolate(points, params, false, 1e-9);
  interpolate.Perform();
  if (!interpolate.IsDone()) {
    fail("pipe: cannot build the twist guide");
  }
  BRepBuilderAPI_MakeEdge edge(interpolate.Curve());
  if (!edge.IsDone()) {
    fail("pipe: cannot build the twist guide edge");
  }
  return BRepBuilderAPI_MakeWire(edge.Edge()).Wire();
}

// Whether every edge of `spine` has a stored pcurve on some face of `support` (OCCT's SetMode
// silently ignores a support that carries nothing).
bool carries(const TopoDS_Shape &support, const TopoDS_Wire &spine) {
  ShapeMap faces;
  TopExp::MapShapes(support, TopAbs_FACE, faces);
  ShapeMap edges;
  TopExp::MapShapes(spine, TopAbs_EDGE, edges);
  for (int e = 1; e <= edges.Extent(); ++e) {
    bool found = false;
    for (int f = 1; f <= faces.Extent() && !found; ++f) {
      double first = 0.0;
      double last = 0.0;
      // Only a stored pcurve: on planes BRep_Tool projects any edge.
      bool stored = false;
      const bool has = !BRep_Tool::CurveOnSurface(TopoDS::Edge(edges(e)),
                                                  TopoDS::Face(faces(f)), first, last, &stored)
                            .IsNull();
      found = has && stored;
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

const char *pipe_status(BRepBuilderAPI_PipeError status) {
  switch (status) {
  case BRepBuilderAPI_PipeDone:
    return "BRepBuilderAPI_PipeDone";
  case BRepBuilderAPI_PipeNotDone:
    return "BRepBuilderAPI_PipeNotDone";
  case BRepBuilderAPI_PlaneNotIntersectGuide:
    return "BRepBuilderAPI_PlaneNotIntersectGuide";
  case BRepBuilderAPI_ImpossibleContact:
    return "BRepBuilderAPI_ImpossibleContact";
  default:
    return "BRepBuilderAPI_PipeError";
  }
}

// The result faces bounded only by edges of `section` (a cap; `section` may be a face or a
// wire).
Shapes cap_faces(const TopoDS_Shape &result, const TopoDS_Shape &section) {
  Shapes out;
  if (section.IsNull()) {
    return out;
  }
  ShapeMap section_edges;
  TopExp::MapShapes(section, TopAbs_EDGE, section_edges);
  if (section_edges.IsEmpty()) {
    return out;
  }
  ShapeMap faces;
  TopExp::MapShapes(result, TopAbs_FACE, faces);
  for (int f = 1; f <= faces.Extent(); ++f) {
    ShapeMap face_edges;
    TopExp::MapShapes(faces(f), TopAbs_EDGE, face_edges);
    bool all = !face_edges.IsEmpty();
    for (int e = 1; e <= face_edges.Extent() && all; ++e) {
      all = section_edges.Contains(face_edges(e));
    }
    if (all) {
      out.push_back(faces(f));
    }
  }
  return out;
}

using EdgeFaces =
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

// A loft section edge's images narrowed to the faces it bounds (`edge_faces`: the result's
// edge → faces map), when the edge itself is a boundary in the result (ruled lofts, the end
// sections of smooth ones): OCCT reports the whole column of faces for every section's edge. A
// middle section of a smooth loft is no edge of the result and keeps OCCT's images.
Shapes bounded_by(const EdgeFaces &edge_faces, const TopoDS_Shape &edge, Shapes images) {
  if (edge.IsNull() || edge.ShapeType() != TopAbs_EDGE || !edge_faces.Contains(edge)) {
    return images;
  }
  const NCollection_List<TopoDS_Shape> &adjacent = edge_faces.FindFromKey(edge);
  Shapes kept;
  for (const TopoDS_Shape &image : images) {
    bool keep = image.ShapeType() != TopAbs_FACE;
    for (const TopoDS_Shape &f : adjacent) {
      keep = keep || f.IsSame(image);
    }
    if (keep) {
      kept.push_back(image);
    }
  }
  return kept.empty() ? images : kept;
}

Shapes all_faces(const TopoDS_Shape &shape) {
  Shapes out;
  ShapeMap faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  for (int f = 1; f <= faces.Extent(); ++f) {
    out.push_back(faces(f));
  }
  return out;
}

// A builder's `Generated`, empty where it raises.
template <typename Builder> Shapes generated_by(Builder &builder, const TopoDS_Shape &s) {
  if (s.IsNull()) {
    return {};
  }
  try {
    return shapes_of(builder.Generated(s));
  } catch (const Standard_Failure &) {
    return {};
  }
}

// The history of every operand through its copy and `builder` (`Generated`), in index terms of
// `op.shape`: a copied sub-shape that is in the result is `Modified` into it, the builder's
// images are `Generated`, anything else is `Deleted`. `extra(copy)` adds generated images the
// builder does not report; `narrow(copy, images)` may then restrict them.
template <typename Builder, typename Extra, typename Narrow>
void record(const OcShapeList &inputs, const std::vector<Operand> &operands, Builder &builder,
            Extra &&extra, Narrow &&narrow, Recorder &rec) {
  for (std::size_t o = 0; o < operands.size(); ++o) {
    const auto operand = static_cast<std::uint16_t>(o);
    const Maps in(inputs.shapes[o]);
    for (int kind = 0; kind < 3; ++kind) {
      for (int i = 1; i <= in.m[kind].Extent(); ++i) {
        const TopoDS_Shape c = operands[o].copy_of(in.m[kind](i));
        const bool modified = !c.IsNull() && rec.relation(operand, kind, i - 1, MODIFIED, {c});
        Shapes images = generated_by(builder, c);
        Shapes more = extra(c);
        images.insert(images.end(), more.begin(), more.end());
        images = narrow(c, std::move(images));
        const bool generated = rec.relation(operand, kind, i - 1, GENERATED, images);
        if (!modified && !generated) {
          rec.deleted(operand, kind, i - 1);
        }
      }
    }
  }
}

} // namespace

std::unique_ptr<OcOp> pipe_op(const OcShapeList &operands, const OcPipeParams &params,
                              rust::Slice<const double> twist, rust::Slice<const double> scale,
                              const ProgressSink &sink) {
  const std::size_t n = operands.shapes.size();
  const std::size_t profiles = params.profiles;
  if (profiles == 0 || profiles >= n) {
    fail("pipe: needs profiles and a spine");
  }
  const bool has_aux = params.mode == MODE_GUIDE || params.mode == MODE_SUPPORT;
  if (has_aux && (params.aux <= profiles || params.aux >= n)) {
    fail("pipe: the guide or support operand is out of range");
  }
  std::vector<bool> as_wire(n, true);
  for (std::size_t i = 0; i < profiles; ++i) {
    // Profiles stay vertices (punctual sections); edges become wires.
    as_wire[i] = operands.shapes[i].ShapeType() != TopAbs_VERTEX;
  }
  if (params.mode == MODE_SUPPORT) {
    as_wire[params.aux] = false;
  }
  std::vector<Operand> ops = copy_all(operands, as_wire);
  auto op = std::make_unique<OcOp>();
  if (params.mode == MODE_SUPPORT &&
      !spine_on_support(ops[profiles], operands.shapes[profiles], ops[params.aux])) {
    op->errors.push_back(std::string(INVALID) +
                         "in support mode the spine's edges must be edges of the support");
    return op;
  }
  const TopoDS_Shape &spine_shape = ops[profiles].shape;
  if (spine_shape.ShapeType() != TopAbs_WIRE) {
    op->errors.push_back(std::string(INVALID) + "the spine is not a wire or an edge");
    return op;
  }
  const TopoDS_Wire spine = TopoDS::Wire(spine_shape);
  BRepOffsetAPI_MakePipeShell pipe(spine);

  switch (params.mode) {
  case MODE_CORRECTED_FRENET:
    pipe.SetMode(false);
    break;
  case MODE_FRENET:
    pipe.SetMode(true);
    break;
  case MODE_FIXED:
    // A constant trihedron only translates the sections, whatever its axes.
    pipe.SetMode(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), dir(params.binormal)));
    break;
  case MODE_BINORMAL:
    pipe.SetMode(dir(params.binormal));
    break;
  case MODE_GUIDE: {
    const TopoDS_Shape &guide = ops[params.aux].shape;
    if (guide.ShapeType() != TopAbs_WIRE) {
      op->errors.push_back(std::string(INVALID) + "the guide is not a wire or an edge");
      return op;
    }
    pipe.SetMode(TopoDS::Wire(guide), false,
                 params.contact ? BRepFill_Contact : BRepFill_NoContact);
    break;
  }
  case MODE_SUPPORT:
    if (!carries(ops[params.aux].shape, spine) || !pipe.SetMode(ops[params.aux].shape)) {
      op->errors.push_back(std::string(INVALID) +
                           "the spine's edges have no representation on the support's faces");
      return op;
    }
    break;
  default:
    fail("pipe: unknown mode");
  }
  if (!twist.empty()) {
    if (params.mode != MODE_CORRECTED_FRENET && params.mode != MODE_FRENET &&
        params.mode != MODE_BINORMAL) {
      op->errors.push_back(std::string(INVALID) +
                           "a twist law needs the corrected Frenet, Frenet or binormal mode");
      return op;
    }
    pipe.SetMode(twist_guide(spine, params, twist, guide_radius(ops, profiles)), false,
                 BRepFill_NoContact);
  }

  if (!scale.empty()) {
    if (profiles != 1 || ops[0].shape.ShapeType() != TopAbs_WIRE) {
      op->errors.push_back(std::string(INVALID) + "a scale law needs exactly one wire profile");
      return op;
    }
    BRepAdaptor_CompCurve curve(spine, false);
    pipe.SetLaw(ops[0].shape, make_law(scale, curve.FirstParameter(), curve.LastParameter()),
                false, false);
  } else {
    for (std::size_t i = 0; i < profiles; ++i) {
      pipe.Add(ops[i].shape, false, false);
    }
  }
  switch (params.transition) {
  case 1:
    pipe.SetTransitionMode(BRepBuilderAPI_RightCorner);
    break;
  case 2:
    pipe.SetTransitionMode(BRepBuilderAPI_RoundCorner);
    break;
  default:
    pipe.SetTransitionMode(BRepBuilderAPI_Transformed);
    break;
  }

  occ::handle<SweepProgress> indicator = new SweepProgress(sink);
  try {
    if (!pipe.IsReady()) {
      op->errors.emplace_back("the pipe has no section");
      return op;
    }
    Message_ProgressScope scope(indicator->Start(), "amg pipe", 1);
    pipe.Build(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  if (op->errors.empty() && !pipe.IsDone()) {
    op->errors.emplace_back(pipe_status(pipe.GetStatus()));
  }
  if (!op->errors.empty()) {
    return op;
  }
  if (params.solid) {
    bool made = false;
    try {
      made = pipe.MakeSolid();
    } catch (const Standard_Failure &failure) {
      op->errors.push_back("MakeSolid: " + format_failure(failure));
      return op;
    }
    if (!made) {
      op->errors.emplace_back("MakeSolid failed (the sections must be closed)");
      return op;
    }
  }
  op->shape = pipe.Shape();
  if (op->shape.IsNull()) {
    op->errors.emplace_back("the pipe is empty");
    return op;
  }

  const Maps out(op->shape);
  Recorder rec(*op, out, true);
  // A profile edge sweeps its faces along the whole spine: OCCT's images are kept as they are.
  // `Generated` also covers the spine: its edges give the faces along them, its vertices the
  // section edges there.
  record(
      operands, ops, pipe, [](const TopoDS_Shape &) { return Shapes{}; },
      [](const TopoDS_Shape &, Shapes images) { return images; }, rec);
  TopoDS_Shape first;
  TopoDS_Shape last;
  try {
    first = pipe.FirstShape();
    last = pipe.LastShape();
  } catch (const Standard_Failure &) {
  }
  rec.role(cap_faces(op->shape, first), START_CAP);
  rec.role(cap_faces(op->shape, last), END_CAP);
  rec.role(all_faces(op->shape), LATERAL);
  return op;
}

std::unique_ptr<OcOp> thru_sections_op(const OcShapeList &wires,
                                       rust::Slice<const OcLoftSection> sections,
                                       const OcLoftParams &params, const ProgressSink &sink) {
  if (sections.size() < 2) {
    fail("thru_sections: needs two sections");
  }
  const std::vector<bool> as_wire(wires.shapes.size(), true);
  const std::vector<Operand> ops = copy_all(wires, as_wire);
  auto op = std::make_unique<OcOp>();
  BRepOffsetAPI_ThruSections loft(params.solid, params.ruled, 1e-6);
  loft.CheckCompatibility(params.check_compatibility);
  loft.SetSmoothing(params.smoothing);
  if (params.max_degree > 0) {
    loft.SetMaxDegree(params.max_degree);
  }
  // Without the compatibility check every wire section needs the same edge count; OCCT builds
  // a broken (open) shape otherwise, so this is the op's failure (incompatible sections).
  if (!params.check_compatibility) {
    int first_count = -1;
    for (std::size_t k = 0; k < sections.size(); ++k) {
      if (sections[k].wire == POINT_SECTION || sections[k].wire >= ops.size()) {
        continue;
      }
      ShapeMap edges;
      TopExp::MapShapes(ops[sections[k].wire].shape, TopAbs_EDGE, edges);
      if (first_count < 0) {
        first_count = edges.Extent();
      } else if (edges.Extent() != first_count) {
        op->errors.push_back("incompatible sections: section " + std::to_string(k) + " has " +
                             std::to_string(edges.Extent()) + " edge(s), the first wire " +
                             std::to_string(first_count) +
                             " (without the compatibility check the counts must match)");
        return op;
      }
    }
  }
  // Which copied wire each section is (the first wire repeated last closes the loft).
  for (std::size_t k = 0; k < sections.size(); ++k) {
    const OcLoftSection &s = sections[k];
    if (s.wire == POINT_SECTION) {
      loft.AddVertex(BRepBuilderAPI_MakeVertex(pnt(s.point)).Vertex());
      continue;
    }
    if (s.wire >= ops.size()) {
      fail("thru_sections: a section's wire is out of range");
    }
    const TopoDS_Shape &w = ops[s.wire].shape;
    if (w.ShapeType() != TopAbs_WIRE) {
      op->errors.push_back(std::string(INVALID) + "section " + std::to_string(k) +
                           " is not a wire or an edge");
      return op;
    }
    loft.AddWire(TopoDS::Wire(w));
  }

  occ::handle<SweepProgress> indicator = new SweepProgress(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg thru_sections", 1);
    loft.Build(scope.Next());
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink)) {
    op->cancelled = true;
    return op;
  }
  if (op->errors.empty() && !loft.IsDone()) {
    op->errors.emplace_back("BRepOffsetAPI_ThruSections NotDone");
  }
  if (op->errors.empty()) {
    op->shape = loft.Shape();
    if (op->shape.IsNull()) {
      op->errors.emplace_back("the loft is empty");
    }
  }
  if (!op->errors.empty()) {
    op->shape = TopoDS_Shape();
    return op;
  }

  const Maps out(op->shape);
  Recorder rec(*op, out, true);
  EdgeFaces edge_faces;
  TopExp::MapShapesAndAncestors(op->shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  record(
      wires, ops, loft,
      [&loft](const TopoDS_Shape &c) -> Shapes {
        if (c.IsNull() || c.ShapeType() != TopAbs_EDGE) {
          return {};
        }
        try {
          TopoDS_Shape f = loft.GeneratedFace(c);
          if (!f.IsNull()) {
            return {f};
          }
        } catch (const Standard_Failure &) {
        }
        return {};
      },
      [&edge_faces](const TopoDS_Shape &c, Shapes images) {
        return bounded_by(edge_faces, c, std::move(images));
      },
      rec);
  TopoDS_Shape first;
  TopoDS_Shape last;
  try {
    first = loft.FirstShape();
    last = loft.LastShape();
  } catch (const Standard_Failure &) {
  }
  rec.role(cap_faces(op->shape, first), LOFT_SECTION, 0);
  rec.role(cap_faces(op->shape, last), LOFT_SECTION,
           static_cast<std::uint32_t>(sections.size() - 1));
  rec.role(all_faces(op->shape), LATERAL);
  return op;
}

} // namespace amg::occt
