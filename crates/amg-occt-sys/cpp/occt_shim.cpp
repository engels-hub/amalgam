#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <iterator>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BinTools.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>
#include <gp_Trsf.hxx>

namespace amg::occt {

namespace {

// Pinned by FND-13 (vcpkg baseline 2026.10.16). Changing OCCT must be a deliberate edit here.
constexpr char PINNED_VERSION[] = "8.0.1";
constexpr char SHIM_VERSION[] = "2";

// Marks an edge end that has no vertex (infinite edges).
constexpr std::uint32_t NO_VERTEX = 0xFFFFFFFFu;

constexpr bool str_eq(const char *a, const char *b) {
  while (*a != '\0' && *a == *b) {
    ++a;
    ++b;
  }
  return *a == *b;
}

static_assert(str_eq(OCC_VERSION_COMPLETE, PINNED_VERSION),
              "OCCT version differs from the pin in occt_shim.cpp (S02 section 1)");

V3 to_v3(const gp_XYZ &p) { return V3{p.X(), p.Y(), p.Z()}; }

// Keeps the (x, y) parameter axes. A left-handed gp_Ax3 therefore yields normal = x × y, the
// opposite of Ax3.Direction(); see geom::Surface in amg-brep.
OcFrame to_frame(const gp_Ax3 &ax) {
  const gp_Dir x = ax.XDirection();
  const gp_Dir y = ax.YDirection();
  const gp_Dir normal = x.Crossed(y);
  return OcFrame{to_v3(ax.Location().XYZ()), to_v3(x.XYZ()), to_v3(y.XYZ()), to_v3(normal.XYZ())};
}

const TopoDS_Face &face_at(const OcIndex &index, std::uint32_t f) {
  return TopoDS::Face(index.faces.FindKey(static_cast<int>(f) + 1));
}

const TopoDS_Edge &edge_at(const OcIndex &index, std::uint32_t e) {
  return TopoDS::Edge(index.edges.FindKey(static_cast<int>(e) + 1));
}

const TopoDS_Vertex &vertex_at(const OcIndex &index, std::uint32_t v) {
  return TopoDS::Vertex(index.vertices.FindKey(static_cast<int>(v) + 1));
}

const TopoDS_Shape &sub_shape_at(const OcIndex &index, std::uint8_t kind, std::uint32_t i) {
  switch (kind) {
  case 0:
    return face_at(index, i);
  case 1:
    return edge_at(index, i);
  case 2:
    return vertex_at(index, i);
  default:
    throw std::invalid_argument("sub-shape kind must be 0, 1 or 2");
  }
}

std::uint32_t index_of(const ShapeMap &map, const TopoDS_Shape &shape, const char *what) {
  const int found = map.FindIndex(shape);
  if (found == 0) {
    throw std::logic_error(std::string(what) + " is not in the shape's index");
  }
  return static_cast<std::uint32_t>(found - 1);
}

// Edges of one wire in traversal order. Falls back to stored order when BRepTools_WireExplorer
// fails or does not visit every edge (disconnected or non-manifold wires).
std::vector<TopoDS_Edge> wire_edges(const TopoDS_Face &face, const TopoDS_Wire &wire) {
  std::vector<TopoDS_Edge> stored;
  for (TopoDS_Iterator it(wire); it.More(); it.Next()) {
    if (it.Value().ShapeType() == TopAbs_EDGE) {
      stored.push_back(TopoDS::Edge(it.Value()));
    }
  }
  std::vector<TopoDS_Edge> walked;
  try {
    for (BRepTools_WireExplorer exp(wire, face); exp.More(); exp.Next()) {
      walked.push_back(exp.Current());
    }
  } catch (const Standard_Failure &) {
    return stored;
  }
  return walked.size() == stored.size() ? walked : stored;
}

TopoDS_Wire outer_wire(const TopoDS_Face &face) {
  try {
    return BRepTools::OuterWire(face);
  } catch (const Standard_Failure &) {
    for (TopoDS_Iterator it(face); it.More(); it.Next()) {
      if (it.Value().ShapeType() == TopAbs_WIRE) {
        return TopoDS::Wire(it.Value());
      }
    }
    return TopoDS_Wire();
  }
}

void copy_bspline_curve(const Handle(Geom_BSplineCurve) & curve, OcCurve &out) {
  out.kind = OcCurveKind::BSpline;
  out.degree = static_cast<std::uint32_t>(curve->Degree());
  out.periodic = curve->IsPeriodic();
  for (int i = 1; i <= curve->NbKnots(); ++i) {
    out.knots.push_back(curve->Knot(i));
    out.mults.push_back(static_cast<std::uint32_t>(curve->Multiplicity(i)));
  }
  const bool rational = curve->IsRational();
  for (int i = 1; i <= curve->NbPoles(); ++i) {
    out.poles.push_back(to_v3(curve->Pole(i).XYZ()));
    if (rational) {
      out.weights.push_back(curve->Weight(i));
    }
  }
}

void copy_bspline_surface(const Handle(Geom_BSplineSurface) & surface, OcSurface &out) {
  out.kind = OcSurfaceKind::BSpline;
  out.u_degree = static_cast<std::uint32_t>(surface->UDegree());
  out.v_degree = static_cast<std::uint32_t>(surface->VDegree());
  out.u_periodic = surface->IsUPeriodic();
  out.v_periodic = surface->IsVPeriodic();
  for (int i = 1; i <= surface->NbUKnots(); ++i) {
    out.u_knots.push_back(surface->UKnot(i));
    out.u_mults.push_back(static_cast<std::uint32_t>(surface->UMultiplicity(i)));
  }
  for (int i = 1; i <= surface->NbVKnots(); ++i) {
    out.v_knots.push_back(surface->VKnot(i));
    out.v_mults.push_back(static_cast<std::uint32_t>(surface->VMultiplicity(i)));
  }
  out.nu = static_cast<std::uint32_t>(surface->NbUPoles());
  out.nv = static_cast<std::uint32_t>(surface->NbVPoles());
  const bool rational = surface->IsURational() || surface->IsVRational();
  for (int i = 1; i <= surface->NbUPoles(); ++i) {
    for (int j = 1; j <= surface->NbVPoles(); ++j) {
      out.poles.push_back(to_v3(surface->Pole(i, j).XYZ()));
      if (rational) {
        out.weights.push_back(surface->Weight(i, j));
      }
    }
  }
}

std::unique_ptr<std::string> stream_bytes(const std::ostringstream &stream) {
  if (!stream.good()) {
    throw std::runtime_error("writing to the memory stream failed");
  }
  return std::make_unique<std::string>(stream.str());
}

std::string to_string(rust::Slice<const std::uint8_t> data) {
  return std::string(reinterpret_cast<const char *>(data.data()), data.size());
}

std::unique_ptr<OcShape> checked_read(TopoDS_Shape shape) {
  if (shape.IsNull()) {
    throw std::runtime_error("the data holds no shape");
  }
  return std::make_unique<OcShape>(std::move(shape));
}

} // namespace

std::string format_failure(const Standard_Failure &failure) {
#if OCC_VERSION_HEX >= 0x080000
  const char *type = failure.ExceptionType();
  const char *message = failure.what();
#else
  const char *type = failure.DynamicType()->Name();
  const char *message = failure.GetMessageString();
#endif
  return std::string(type) + ": " + (message != nullptr ? message : "");
}

rust::String kernel_version() {
  return rust::String(std::string("occt-") + OCC_VERSION_COMPLETE + ";shim-" + SHIM_VERSION);
}

std::unique_ptr<OcShape> make_box(double dx, double dy, double dz) {
  BRepPrimAPI_MakeBox maker(dx, dy, dz);
  return std::make_unique<OcShape>(maker.Shape());
}

double volume(const OcShape &shape) {
  GProp_GProps props;
  BRepGProp::VolumeProperties(shape.get(), props);
  return props.Mass();
}

void gp_dir_from_zero_vector() {
  // volatile: the zero must reach the constructor at run time, not in a constant expression.
  volatile double zero = 0.0;
  const gp_Dir direction(zero, zero, zero);
  (void)direction;
}

bool shape_is_null(const OcShape &shape) { return shape.get().IsNull(); }

std::uint8_t shape_kind(const OcShape &shape) {
  if (shape.get().IsNull()) {
    throw std::invalid_argument("shape_kind: null shape");
  }
  return static_cast<std::uint8_t>(shape.get().ShapeType());
}

bool shape_is_same(const OcShape &a, const OcShape &b) { return a.get().IsSame(b.get()); }

std::unique_ptr<OcShape> shape_moved(const OcShape &shape, const OcMat34 &m) {
  gp_Trsf transform;
  transform.SetValues(m.m[0], m.m[1], m.m[2], m.m[3], m.m[4], m.m[5], m.m[6], m.m[7], m.m[8],
                      m.m[9], m.m[10], m.m[11]);
  // SetValues derives the scale from the determinant; drop its round-off, the input is rigid.
  transform.SetScaleFactor(1.0);
  // An identity location would still make a distinct, non-`IsSame` shape. `SetValues` never
  // sets the `gp_Identity` form, so compare the values.
  static constexpr double IDENTITY[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  if (std::equal(std::begin(m.m), std::end(m.m), std::begin(IDENTITY))) {
    return std::make_unique<OcShape>(shape.get());
  }
  return std::make_unique<OcShape>(shape.get().Moved(TopLoc_Location(transform)));
}

OcBox shape_bbox(const OcShape &shape) {
  Bnd_Box box;
  BRepBndLib::AddOptimal(shape.get(), box, false, false);
  OcBox out{};
  out.is_void = box.IsVoid();
  if (!out.is_void) {
    out.min = to_v3(box.CornerMin().XYZ());
    out.max = to_v3(box.CornerMax().XYZ());
  }
  return out;
}

std::unique_ptr<OcIndex> build_index(const OcShape &shape) {
  auto index = std::make_unique<OcIndex>();
  TopExp::MapShapes(shape.get(), TopAbs_FACE, index->faces);
  TopExp::MapShapes(shape.get(), TopAbs_EDGE, index->edges);
  TopExp::MapShapes(shape.get(), TopAbs_VERTEX, index->vertices);
  TopExp::MapShapes(shape.get(), TopAbs_SOLID, index->solids);
  return index;
}

OcTopology index_topology(const OcIndex &index) {
  OcTopology out{};
  out.n_faces = static_cast<std::uint32_t>(index.faces.Extent());
  out.n_edges = static_cast<std::uint32_t>(index.edges.Extent());
  out.n_vertices = static_cast<std::uint32_t>(index.vertices.Extent());

  const auto append_wire = [&](const TopoDS_Face &face, const TopoDS_Wire &wire) {
    for (const TopoDS_Edge &edge : wire_edges(face, wire)) {
      out.wire_edges.push_back(index_of(index.edges, edge, "wire edge"));
    }
    out.wire_edge_starts.push_back(static_cast<std::uint32_t>(out.wire_edges.size()));
  };

  out.wire_edge_starts.push_back(0);
  std::uint32_t n_wires = 0;
  for (std::uint32_t f = 0; f < out.n_faces; ++f) {
    out.face_wire_starts.push_back(n_wires);
    const TopoDS_Face &face = face_at(index, f);
    const TopoDS_Wire outer = outer_wire(face);
    if (!outer.IsNull()) {
      append_wire(face, outer);
      ++n_wires;
    }
    for (TopoDS_Iterator it(face); it.More(); it.Next()) {
      if (it.Value().ShapeType() == TopAbs_WIRE && !it.Value().IsSame(outer)) {
        append_wire(face, TopoDS::Wire(it.Value()));
        ++n_wires;
      }
    }
  }
  out.face_wire_starts.push_back(n_wires);

  std::vector<std::vector<std::uint32_t>> vertex_edges(out.n_vertices);
  for (std::uint32_t e = 0; e < out.n_edges; ++e) {
    const TopoDS_Edge &edge = edge_at(index, e);
    TopoDS_Vertex first;
    TopoDS_Vertex last;
    TopExp::Vertices(edge, first, last, false);
    out.edge_vertices.push_back(first.IsNull() ? NO_VERTEX
                                               : index_of(index.vertices, first, "edge vertex"));
    out.edge_vertices.push_back(last.IsNull() ? NO_VERTEX
                                              : index_of(index.vertices, last, "edge vertex"));
    for (TopoDS_Iterator it(edge); it.More(); it.Next()) {
      if (it.Value().ShapeType() != TopAbs_VERTEX) {
        continue;
      }
      auto &list = vertex_edges[index_of(index.vertices, it.Value(), "edge vertex")];
      // A closed edge lists its one vertex twice.
      if (list.empty() || list.back() != e) {
        list.push_back(e);
      }
    }
  }
  out.vertex_edge_starts.push_back(0);
  for (const auto &list : vertex_edges) {
    for (const std::uint32_t e : list) {
      out.vertex_edges.push_back(e);
    }
    out.vertex_edge_starts.push_back(static_cast<std::uint32_t>(out.vertex_edges.size()));
  }
  return out;
}

std::uint32_t index_solid_count(const OcIndex &index) {
  return static_cast<std::uint32_t>(index.solids.Extent());
}

std::unique_ptr<OcShape> index_solid(const OcIndex &index, std::uint32_t i) {
  return std::make_unique<OcShape>(index.solids.FindKey(static_cast<int>(i) + 1));
}

std::unique_ptr<OcShape> index_sub_shape(const OcIndex &index, std::uint8_t kind,
                                         std::uint32_t i) {
  return std::make_unique<OcShape>(sub_shape_at(index, kind, i));
}

double index_tolerance(const OcIndex &index, std::uint8_t kind, std::uint32_t i) {
  switch (kind) {
  case 0:
    return BRep_Tool::Tolerance(face_at(index, i));
  case 1:
    return BRep_Tool::Tolerance(edge_at(index, i));
  case 2:
    return BRep_Tool::Tolerance(vertex_at(index, i));
  default:
    throw std::invalid_argument("sub-shape kind must be 0, 1 or 2");
  }
}

std::uint64_t index_estimated_bytes(const OcIndex &index) {
  // Rough per-object sizes of the BRep_T* records and their analytic geometry; shared geometry
  // is counted once per user.
  constexpr std::uint64_t FACE_BYTES = 400;
  constexpr std::uint64_t EDGE_BYTES = 500;
  constexpr std::uint64_t VERTEX_BYTES = 150;
  constexpr std::uint64_t POLE_BYTES = 32;

  std::uint64_t total = FACE_BYTES * static_cast<std::uint64_t>(index.faces.Extent()) +
                        EDGE_BYTES * static_cast<std::uint64_t>(index.edges.Extent()) +
                        VERTEX_BYTES * static_cast<std::uint64_t>(index.vertices.Extent());
  TopLoc_Location location;
  for (int i = 1; i <= index.faces.Extent(); ++i) {
    const Handle(Geom_Surface) &surface =
        BRep_Tool::Surface(TopoDS::Face(index.faces.FindKey(i)), location);
    const Handle(Geom_BSplineSurface) spline = Handle(Geom_BSplineSurface)::DownCast(surface);
    if (!spline.IsNull()) {
      total += POLE_BYTES * static_cast<std::uint64_t>(spline->NbUPoles()) *
               static_cast<std::uint64_t>(spline->NbVPoles());
    }
  }
  for (int i = 1; i <= index.edges.Extent(); ++i) {
    double first = 0.0;
    double last = 0.0;
    const Handle(Geom_Curve) &curve =
        BRep_Tool::Curve(TopoDS::Edge(index.edges.FindKey(i)), location, first, last);
    const Handle(Geom_BSplineCurve) spline = Handle(Geom_BSplineCurve)::DownCast(curve);
    if (!spline.IsNull()) {
      total += POLE_BYTES * static_cast<std::uint64_t>(spline->NbPoles());
    }
  }
  return total;
}

OcSurface face_surface(const OcIndex &index, std::uint32_t f) {
  OcSurface out{};
  out.kind = OcSurfaceKind::Other;
  const Handle(Geom_Surface) surface = BRep_Tool::Surface(face_at(index, f));
  if (surface.IsNull()) {
    return out;
  }
  const GeomAdaptor_Surface adaptor(surface);
  switch (adaptor.GetType()) {
  case GeomAbs_Plane:
    out.kind = OcSurfaceKind::Plane;
    out.frame = to_frame(adaptor.Plane().Position());
    break;
  case GeomAbs_Cylinder: {
    const gp_Cylinder cylinder = adaptor.Cylinder();
    out.kind = OcSurfaceKind::Cylinder;
    out.frame = to_frame(cylinder.Position());
    out.a = cylinder.Radius();
    break;
  }
  case GeomAbs_Cone: {
    const gp_Cone cone = adaptor.Cone();
    out.kind = OcSurfaceKind::Cone;
    out.frame = to_frame(cone.Position());
    out.a = cone.RefRadius();
    out.b = cone.SemiAngle();
    break;
  }
  case GeomAbs_Sphere: {
    const gp_Sphere sphere = adaptor.Sphere();
    out.kind = OcSurfaceKind::Sphere;
    out.frame = to_frame(sphere.Position());
    out.a = sphere.Radius();
    break;
  }
  case GeomAbs_Torus: {
    const gp_Torus torus = adaptor.Torus();
    out.kind = OcSurfaceKind::Torus;
    out.frame = to_frame(torus.Position());
    out.a = torus.MajorRadius();
    out.b = torus.MinorRadius();
    break;
  }
  case GeomAbs_BSplineSurface:
    copy_bspline_surface(adaptor.BSpline(), out);
    break;
  case GeomAbs_BezierSurface:
    copy_bspline_surface(GeomConvert::SurfaceToBSplineSurface(adaptor.Bezier()), out);
    break;
  default:
    break;
  }
  return out;
}

bool face_is_reversed(const OcIndex &index, std::uint32_t f) {
  return face_at(index, f).Orientation() == TopAbs_REVERSED;
}

OcUvBounds face_uv_bounds(const OcIndex &index, std::uint32_t f) {
  OcUvBounds out{};
  BRepTools::UVBounds(face_at(index, f), out.umin, out.umax, out.vmin, out.vmax);
  return out;
}

double face_area(const OcIndex &index, std::uint32_t f) {
  GProp_GProps props;
  BRepGProp::SurfaceProperties(face_at(index, f), props);
  return props.Mass();
}

OcCurve edge_curve(const OcIndex &index, std::uint32_t e) {
  OcCurve out{};
  out.kind = OcCurveKind::Absent;
  const TopoDS_Edge &edge = edge_at(index, e);
  if (BRep_Tool::Degenerated(edge)) {
    return out;
  }
  const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, out.first, out.last);
  if (curve.IsNull()) {
    return out;
  }
  out.kind = OcCurveKind::Other;
  const GeomAdaptor_Curve adaptor(curve);
  switch (adaptor.GetType()) {
  case GeomAbs_Line: {
    const gp_Lin line = adaptor.Line();
    out.kind = OcCurveKind::Line;
    out.frame.origin = to_v3(line.Location().XYZ());
    out.frame.normal = to_v3(line.Direction().XYZ());
    break;
  }
  case GeomAbs_Circle: {
    const gp_Circ circle = adaptor.Circle();
    out.kind = OcCurveKind::Circle;
    out.frame = to_frame(gp_Ax3(circle.Position()));
    out.a = circle.Radius();
    break;
  }
  case GeomAbs_Ellipse: {
    const gp_Elips ellipse = adaptor.Ellipse();
    out.kind = OcCurveKind::Ellipse;
    out.frame = to_frame(gp_Ax3(ellipse.Position()));
    out.a = ellipse.MajorRadius();
    out.b = ellipse.MinorRadius();
    break;
  }
  case GeomAbs_BSplineCurve:
    copy_bspline_curve(adaptor.BSpline(), out);
    break;
  case GeomAbs_BezierCurve:
    copy_bspline_curve(GeomConvert::CurveToBSplineCurve(adaptor.Bezier()), out);
    break;
  default:
    break;
  }
  return out;
}

bool edge_is_seam(const OcIndex &index, std::uint32_t e, std::uint32_t f) {
  return BRep_Tool::IsClosed(edge_at(index, e), face_at(index, f));
}

V3 vertex_point(const OcIndex &index, std::uint32_t v) {
  return to_v3(BRep_Tool::Pnt(vertex_at(index, v)).XYZ());
}

std::unique_ptr<std::string> shape_write_bin(const OcShape &shape) {
  std::ostringstream stream(std::ios::out | std::ios::binary);
  BinTools::Write(shape.get(), stream, false, false, BinTools_FormatVersion_VERSION_4);
  return stream_bytes(stream);
}

std::unique_ptr<OcShape> shape_read_bin(rust::Slice<const std::uint8_t> data) {
  std::istringstream stream(to_string(data), std::ios::in | std::ios::binary);
  TopoDS_Shape shape;
  BinTools::Read(shape, stream);
  return checked_read(std::move(shape));
}

std::unique_ptr<std::string> shape_write_text(const OcShape &shape) {
  std::ostringstream stream(std::ios::out);
  stream.imbue(std::locale::classic());
  BRepTools::Write(shape.get(), stream, false, false, TopTools_FormatVersion_VERSION_3);
  return stream_bytes(stream);
}

std::unique_ptr<OcShape> shape_read_text(rust::Slice<const std::uint8_t> data) {
  std::istringstream stream(to_string(data), std::ios::in);
  stream.imbue(std::locale::classic());
  BRep_Builder builder;
  TopoDS_Shape shape;
  BRepTools::Read(shape, stream, builder);
  return checked_read(std::move(shape));
}

} // namespace amg::occt
