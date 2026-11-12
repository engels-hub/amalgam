// Face and wire builders (S02 §4, BREP-03). Declarations and categories are in occt_shim.hxx.
//
// Every function here takes plain data only and builds new topology, so all are category R.
// Inputs are validated in amg-brep (closure, orientation, containment); this file turns them
// into OCCT geometry, shares one vertex per loop junction (tolerance = the junction's gap), and
// reports, for every input edge, the index of the edge it became in the result's
// TopExp::MapShapes order (the order of amg-brep's sub-shape index).

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeSegment2d.hxx>
#include <Geom2dAPI_Interpolate.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_HArray1.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Face.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopExp.hxx>
#include <NCollection_List.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>
#include <gp_Vec.hxx>

namespace amg::occt {

namespace {

constexpr std::uint32_t LOST_EDGE = 0xFFFFFFFFu;

// A turn sharper than this between consecutive polyline segments starts a new edge.
constexpr double CORNER_ANGLE = 0.7;

// At most this many polyline segments per edge.
constexpr std::size_t MAX_EDGE_SEGMENTS = 12;

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

gp_Pnt pnt(const V3 &v) { return gp_Pnt(v.x, v.y, v.z); }
gp_Dir dir(const V3 &v) { return gp_Dir(v.x, v.y, v.z); }
gp_Ax2 ax2(const OcFrame &f) { return gp_Ax2(pnt(f.origin), dir(f.normal), dir(f.x)); }
gp_Ax3 ax3(const OcFrame &f) { return gp_Ax3(pnt(f.origin), dir(f.normal), dir(f.x)); }

occ::handle<Geom_Curve> curve_in(const OcCurve &c, const std::string &at) {
  switch (c.kind) {
  case OcCurveKind::Line:
    return new Geom_Line(pnt(c.frame.origin), dir(c.frame.normal));
  case OcCurveKind::Circle:
    return new Geom_Circle(ax2(c.frame), c.a);
  case OcCurveKind::Ellipse:
    return new Geom_Ellipse(ax2(c.frame), c.a, c.b);
  case OcCurveKind::BSpline: {
    const int np = static_cast<int>(c.poles.size());
    const int nk = static_cast<int>(c.knots.size());
    NCollection_Array1<gp_Pnt> poles(1, np);
    for (int i = 0; i < np; ++i) {
      poles.SetValue(i + 1, pnt(c.poles[i]));
    }
    NCollection_Array1<double> knots(1, nk);
    NCollection_Array1<int> mults(1, nk);
    for (int i = 0; i < nk; ++i) {
      knots.SetValue(i + 1, c.knots[i]);
      mults.SetValue(i + 1, static_cast<int>(c.mults[i]));
    }
    const int degree = static_cast<int>(c.degree);
    if (c.weights.empty()) {
      return new Geom_BSplineCurve(poles, knots, mults, degree, c.periodic);
    }
    NCollection_Array1<double> weights(1, np);
    for (int i = 0; i < np; ++i) {
      weights.SetValue(i + 1, c.weights[i]);
    }
    return new Geom_BSplineCurve(poles, weights, knots, mults, degree, c.periodic);
  }
  default:
    fail(at + ": the curve kind cannot be built");
  }
}

occ::handle<Geom_Surface> surface_in(const OcSurface &s) {
  switch (s.kind) {
  case OcSurfaceKind::Plane:
    return new Geom_Plane(ax3(s.frame));
  case OcSurfaceKind::Cylinder:
    return new Geom_CylindricalSurface(ax3(s.frame), s.a);
  case OcSurfaceKind::Cone:
    return new Geom_ConicalSurface(ax3(s.frame), s.b, s.a);
  case OcSurfaceKind::Sphere:
    return new Geom_SphericalSurface(ax3(s.frame), s.a);
  case OcSurfaceKind::Torus:
    return new Geom_ToroidalSurface(ax3(s.frame), s.a, s.b);
  case OcSurfaceKind::BSpline: {
    const int nu = static_cast<int>(s.nu);
    const int nv = static_cast<int>(s.nv);
    NCollection_Array2<gp_Pnt> poles(1, nu, 1, nv);
    NCollection_Array2<double> weights(1, nu, 1, nv);
    for (int i = 0; i < nu; ++i) {
      for (int j = 0; j < nv; ++j) {
        const std::size_t k = static_cast<std::size_t>(i) * s.nv + j;
        poles.SetValue(i + 1, j + 1, pnt(s.poles[k]));
        weights.SetValue(i + 1, j + 1, s.weights.empty() ? 1.0 : s.weights[k]);
      }
    }
    const auto array = [](const rust::Vec<double> &v) {
      NCollection_Array1<double> out(1, static_cast<int>(v.size()));
      for (std::size_t i = 0; i < v.size(); ++i) {
        out.SetValue(static_cast<int>(i) + 1, v[i]);
      }
      return out;
    };
    const auto int_array = [](const rust::Vec<std::uint32_t> &v) {
      NCollection_Array1<int> out(1, static_cast<int>(v.size()));
      for (std::size_t i = 0; i < v.size(); ++i) {
        out.SetValue(static_cast<int>(i) + 1, static_cast<int>(v[i]));
      }
      return out;
    };
    const int du = static_cast<int>(s.u_degree);
    const int dv = static_cast<int>(s.v_degree);
    if (s.weights.empty()) {
      return new Geom_BSplineSurface(poles, array(s.u_knots), array(s.v_knots),
                                     int_array(s.u_mults), int_array(s.v_mults), du, dv,
                                     s.u_periodic, s.v_periodic);
    }
    return new Geom_BSplineSurface(poles, weights, array(s.u_knots), array(s.v_knots),
                                   int_array(s.u_mults), int_array(s.v_mults), du, dv,
                                   s.u_periodic, s.v_periodic);
  }
  default:
    fail("the surface kind cannot be built");
  }
}

// The edges of one loop or open wire, in input order, sharing a vertex at each junction.
struct BuiltLoop {
  TopoDS_Wire wire;
  std::vector<TopoDS_Edge> edges;
};

BuiltLoop build_loop(const rust::Vec<OcEdgeIn> &edges, std::size_t begin, std::size_t end,
                     bool closed, const std::string &where) {
  const std::size_t n = end - begin;
  if (n == 0) {
    fail(where + ": the loop has no edges");
  }
  std::vector<occ::handle<Geom_Curve>> curves(n);
  std::vector<gp_Pnt> starts(n);
  std::vector<gp_Pnt> ends(n);
  for (std::size_t k = 0; k < n; ++k) {
    const OcEdgeIn &in = edges[begin + k];
    curves[k] = curve_in(in.curve, where + ", edge " + std::to_string(k));
    const gp_Pnt first = curves[k]->Value(in.curve.first);
    const gp_Pnt last = curves[k]->Value(in.curve.last);
    starts[k] = in.reversed ? last : first;
    ends[k] = in.reversed ? first : last;
  }
  BRep_Builder builder;
  const std::size_t n_vertices = closed ? n : n + 1;
  std::vector<TopoDS_Vertex> vertices(n_vertices);
  for (std::size_t k = 0; k < n; ++k) {
    double gap = 0.0;
    if (k > 0) {
      gap = ends[k - 1].Distance(starts[k]);
    } else if (closed) {
      gap = ends[n - 1].Distance(starts[0]);
    }
    builder.MakeVertex(vertices[k], starts[k], std::max(1.001 * gap, Precision::Confusion()));
  }
  if (!closed) {
    builder.MakeVertex(vertices[n], ends[n - 1], Precision::Confusion());
  }
  BuiltLoop out;
  builder.MakeWire(out.wire);
  for (std::size_t k = 0; k < n; ++k) {
    const OcEdgeIn &in = edges[begin + k];
    const TopoDS_Vertex &v_start = vertices[k];
    const TopoDS_Vertex &v_end = vertices[closed ? (k + 1) % n : k + 1];
    // Vertices in the curve's parameter order.
    const TopoDS_Vertex &v_first = in.reversed ? v_end : v_start;
    const TopoDS_Vertex &v_last = in.reversed ? v_start : v_end;
    BRepBuilderAPI_MakeEdge make(curves[k], v_first, v_last, in.curve.first, in.curve.last);
    if (!make.IsDone()) {
      fail(where + ", edge " + std::to_string(k) + ": BRepBuilderAPI_MakeEdge error " +
           std::to_string(static_cast<int>(make.Error())));
    }
    TopoDS_Edge edge = make.Edge();
    if (in.reversed) {
      edge.Reverse();
    }
    builder.Add(out.wire, edge);
    out.edges.push_back(edge);
  }
  out.wire.Closed(closed);
  return out;
}

// Index of each of `edges` in `shape`'s edge map, LOST_EDGE where absent.
std::vector<std::uint32_t> edge_map(const TopoDS_Shape &shape,
                                    const std::vector<TopoDS_Shape> &edges) {
  ShapeMap map;
  TopExp::MapShapes(shape, TopAbs_EDGE, map);
  std::vector<std::uint32_t> out;
  out.reserve(edges.size());
  for (const TopoDS_Shape &edge : edges) {
    const int found = edge.IsNull() ? 0 : map.FindIndex(edge);
    out.push_back(found == 0 ? LOST_EDGE : static_cast<std::uint32_t>(found - 1));
  }
  return out;
}

std::string loop_name(std::size_t face, std::size_t loop) {
  return "face " + std::to_string(face) + ", loop " + std::to_string(loop);
}

// ---- polylines on a surface ----------------------------------------------------------------

// Twice the signed area of a closed UV polygon.
double uv_area2(const std::vector<gp_Pnt2d> &uv) {
  double sum = 0.0;
  for (std::size_t i = 0; i < uv.size(); ++i) {
    const gp_Pnt2d &a = uv[i];
    const gp_Pnt2d &b = uv[(i + 1) % uv.size()];
    sum += a.X() * b.Y() - b.X() * a.Y();
  }
  return sum;
}

// A pcurve through the points of `run` (at least 2): a segment, or an interpolating B-spline.
occ::handle<Geom2d_Curve> fit_run(const std::vector<gp_Pnt2d> &run) {
  if (run.size() == 2) {
    return GC_MakeSegment2d(run[0], run[1]).Value();
  }
  occ::handle<NCollection_HArray1<gp_Pnt2d>> held =
      new NCollection_HArray1<gp_Pnt2d>(1, static_cast<int>(run.size()));
  for (std::size_t i = 0; i < run.size(); ++i) {
    held->SetValue(static_cast<int>(i) + 1, run[i]);
  }
  Geom2dAPI_Interpolate interp(held, false, Precision::PConfusion());
  interp.Perform();
  if (!interp.IsDone()) {
    fail("could not fit a pcurve through a polyline run");
  }
  return interp.Curve();
}

// Moves `value` by whole periods to the one nearest `reference`.
double nearest_period(double value, double reference, double period) {
  return value + period * std::round((reference - value) / period);
}

// A wire on `surface` through the closed polyline `pts` (no repeated closing point), oriented so
// that `outer` loops run counter-clockwise in (u, v) and holes clockwise. Each run between
// corners (at most MAX_EDGE_SEGMENTS segments) becomes one edge whose pcurve interpolates the
// projected points.
TopoDS_Wire polyline_wire(const occ::handle<Geom_Surface> &surface, ShapeAnalysis_Surface &sas,
                          std::vector<gp_Pnt> pts, bool outer, double tol,
                          const std::string &where) {
  const std::size_t n = pts.size();
  std::vector<gp_Pnt2d> uv(n);
  // Projection, then unwrapping across periodic seams so (u, v) is continuous along the loop.
  const bool u_periodic = surface->IsUPeriodic();
  const bool v_periodic = surface->IsVPeriodic();
  for (std::size_t i = 0; i < n; ++i) {
    gp_Pnt2d q = sas.ValueOfUV(pts[i], tol);
    if (i > 0) {
      if (u_periodic) {
        q.SetX(nearest_period(q.X(), uv[i - 1].X(), surface->UPeriod()));
      }
      if (v_periodic) {
        q.SetY(nearest_period(q.Y(), uv[i - 1].Y(), surface->VPeriod()));
      }
    }
    uv[i] = q;
  }
  const double gap_u = uv[n - 1].X() - uv[0].X();
  const double gap_v = uv[n - 1].Y() - uv[0].Y();
  if ((u_periodic && std::abs(gap_u) > 0.5 * surface->UPeriod()) ||
      (v_periodic && std::abs(gap_v) > 0.5 * surface->VPeriod())) {
    fail(where + ": the polyline winds around the surface's period; split the face");
  }
  const double area2 = uv_area2(uv);
  if (area2 == 0.0) {
    fail(where + ": the polyline encloses no area in (u, v)");
  }
  if ((area2 > 0.0) != outer) {
    std::reverse(pts.begin(), pts.end());
    std::reverse(uv.begin(), uv.end());
  }
  // Corners: a turn sharper than CORNER_ANGLE in 3D.
  std::vector<std::size_t> corners;
  for (std::size_t i = 0; i < n; ++i) {
    const gp_Vec in(pts[(i + n - 1) % n], pts[i]);
    const gp_Vec out(pts[i], pts[(i + 1) % n]);
    if (in.Magnitude() > 0.0 && out.Magnitude() > 0.0 && in.Angle(out) > CORNER_ANGLE) {
      corners.push_back(i);
    }
  }
  if (corners.empty()) {
    corners.push_back(0);
  }
  // Split long runs: OCCT's fixed-order integrators (BRepGProp without Eps, as in face_area)
  // under-integrate one pcurve with very many spans.
  std::vector<std::size_t> splits;
  for (std::size_t c = 0; c < corners.size(); ++c) {
    const std::size_t from = corners[c];
    const std::size_t to = c + 1 < corners.size() ? corners[c + 1] : corners[0] + n;
    const std::size_t pieces = (to - from + MAX_EDGE_SEGMENTS - 1) / MAX_EDGE_SEGMENTS;
    for (std::size_t k = 0; k < pieces; ++k) {
      splits.push_back((from + k * (to - from) / pieces) % n);
    }
  }
  std::sort(splits.begin(), splits.end());
  corners = splits;
  BRep_Builder builder;
  TopoDS_Wire wire;
  builder.MakeWire(wire);
  const double vertex_tol = std::max(2.0 * tol, 1e-7);
  std::vector<TopoDS_Vertex> vertices(corners.size());
  for (std::size_t c = 0; c < corners.size(); ++c) {
    const gp_Pnt2d &q = uv[corners[c]];
    builder.MakeVertex(vertices[c], surface->Value(q.X(), q.Y()), vertex_tol);
  }
  for (std::size_t c = 0; c < corners.size(); ++c) {
    const std::size_t from = corners[c];
    const std::size_t to = c + 1 < corners.size() ? corners[c + 1] : corners[0] + n;
    std::vector<gp_Pnt2d> run;
    for (std::size_t i = from; i <= to; ++i) {
      gp_Pnt2d q = uv[i % n];
      if (i >= n && !run.empty()) {
        // Past the closing step: keep continuity with the previous point.
        if (u_periodic) {
          q.SetX(nearest_period(q.X(), run.back().X(), surface->UPeriod()));
        }
        if (v_periodic) {
          q.SetY(nearest_period(q.Y(), run.back().Y(), surface->VPeriod()));
        }
      }
      run.push_back(q);
    }
    const occ::handle<Geom2d_Curve> pcurve = fit_run(run);
    BRepBuilderAPI_MakeEdge make(pcurve, surface, vertices[c],
                                 vertices[(c + 1) % corners.size()], pcurve->FirstParameter(),
                                 pcurve->LastParameter());
    if (!make.IsDone()) {
      fail(where + ", edge " + std::to_string(c) + ": BRepBuilderAPI_MakeEdge error " +
           std::to_string(static_cast<int>(make.Error())));
    }
    builder.Add(wire, make.Edge());
  }
  wire.Closed(true);
  return wire;
}

// Twice the signed (u, v) area of `wire` on `face`, from its pcurves in wire order.
double wire_uv_area2(const TopoDS_Wire &wire, const TopoDS_Face &face) {
  std::vector<gp_Pnt2d> pts;
  for (TopoDS_Iterator it(wire); it.More(); it.Next()) {
    const TopoDS_Edge &edge = TopoDS::Edge(it.Value());
    double f = 0.0;
    double l = 0.0;
    const occ::handle<Geom2d_Curve> pcurve = BRep_Tool::CurveOnSurface(edge, face, f, l);
    if (pcurve.IsNull()) {
      continue;
    }
    constexpr int SAMPLES = 32;
    const bool reversed = edge.Orientation() == TopAbs_REVERSED;
    for (int k = 0; k < SAMPLES; ++k) {
      const double s = static_cast<double>(k) / SAMPLES;
      pts.push_back(pcurve->Value(reversed ? l + (f - l) * s : f + (l - f) * s));
    }
  }
  return pts.size() < 3 ? 0.0 : uv_area2(pts);
}

// Rebuilds `face` with its first wire counter-clockwise in (u, v) and the others clockwise.
TopoDS_Face orient_wires(const TopoDS_Face &face, const TopoDS_Shape &outer_hint) {
  BRep_Builder builder;
  TopoDS_Face out = TopoDS::Face(face.EmptyCopied().Oriented(TopAbs_FORWARD));
  const TopoDS_Face forward = TopoDS::Face(face.Oriented(TopAbs_FORWARD));
  std::vector<TopoDS_Wire> wires;
  for (TopoDS_Iterator it(forward); it.More(); it.Next()) {
    if (it.Value().ShapeType() == TopAbs_WIRE) {
      wires.push_back(TopoDS::Wire(it.Value()));
    }
  }
  // The outer wire is the one made from the first loop, else the largest.
  std::size_t outer = 0;
  double best = -1.0;
  for (std::size_t w = 0; w < wires.size(); ++w) {
    const double a = std::abs(wire_uv_area2(wires[w], forward));
    if (wires[w].IsSame(outer_hint)) {
      outer = w;
      break;
    }
    if (a > best) {
      best = a;
      outer = w;
    }
  }
  for (std::size_t w = 0; w < wires.size(); ++w) {
    const double a = wire_uv_area2(wires[w], forward);
    const bool want_ccw = w == outer;
    TopoDS_Wire wire = wires[w];
    if (a != 0.0 && (a > 0.0) != want_ccw) {
      wire.Reverse();
    }
    builder.Add(out, wire);
  }
  return out;
}
} // namespace

std::unique_ptr<OcBuild> make_planar_faces(rust::Slice<const OcPlanarIn> faces, double tol) {
  auto out = std::make_unique<OcBuild>();
  BRep_Builder builder;
  for (std::size_t f = 0; f < faces.size(); ++f) {
    const OcPlanarIn &in = faces[f];
    const auto &starts = in.loops.loop_starts;
    if (starts.size() < 2) {
      fail("face " + std::to_string(f) + ": no loops");
    }
    occ::handle<Geom_Plane> plane = new Geom_Plane(ax3(in.frame));
    TopoDS_Face face;
    builder.MakeFace(face, plane, std::max(tol, Precision::Confusion()));
    std::vector<TopoDS_Shape> edges;
    NCollection_List<TopoDS_Shape> edge_list;
    for (std::size_t l = 0; l + 1 < starts.size(); ++l) {
      BuiltLoop loop = build_loop(in.loops.edges, starts[l], starts[l + 1], true, loop_name(f, l));
      builder.Add(face, loop.wire);
      for (const TopoDS_Edge &edge : loop.edges) {
        edges.push_back(edge);
        edge_list.Append(edge);
      }
    }
    BRepLib::BuildPCurveForEdgesOnPlane(edge_list, face);
    out->edge_maps.push_back(edge_map(face, edges));
    out->shapes.push_back(face);
  }
  return out;
}

std::unique_ptr<OcBuild> make_face_on_surface(const OcSurface &surface, std::uint8_t bounds,
                                              const OcUvBounds &uv, const OcLoopsIn &loops,
                                              rust::Slice<const V3> points,
                                              rust::Slice<const std::uint32_t> point_starts,
                                              double tol) {
  const occ::handle<Geom_Surface> s = surface_in(surface);
  const double degen = Precision::Confusion();
  TopoDS_Face face;
  std::vector<TopoDS_Shape> edges;
  switch (bounds) {
  case 0: {
    BRepBuilderAPI_MakeFace make(s, degen);
    if (!make.IsDone()) {
      fail("natural bounds: BRepBuilderAPI_MakeFace error " +
           std::to_string(static_cast<int>(make.Error())));
    }
    face = make.Face();
    break;
  }
  case 1: {
    BRepBuilderAPI_MakeFace make(s, uv.umin, uv.umax, uv.vmin, uv.vmax, degen);
    if (!make.IsDone()) {
      fail("uv bounds: BRepBuilderAPI_MakeFace error " +
           std::to_string(static_cast<int>(make.Error())));
    }
    face = make.Face();
    break;
  }
  case 2: {
    const auto &starts = loops.loop_starts;
    if (starts.size() < 2) {
      fail("no loops");
    }
    std::vector<BuiltLoop> built;
    for (std::size_t l = 0; l + 1 < starts.size(); ++l) {
      built.push_back(
          build_loop(loops.edges, starts[l], starts[l + 1], true, "loop " + std::to_string(l)));
    }
    BRepBuilderAPI_MakeFace make(s, built[0].wire, true);
    for (std::size_t l = 1; l < built.size(); ++l) {
      make.Add(built[l].wire);
    }
    if (!make.IsDone()) {
      fail("loop bounds: BRepBuilderAPI_MakeFace error " +
           std::to_string(static_cast<int>(make.Error())));
    }
    // Pcurves (projection), wire order and orientation, missing seams.
    occ::handle<ShapeBuild_ReShape> context = new ShapeBuild_ReShape();
    occ::handle<ShapeFix_Face> fix = new ShapeFix_Face(make.Face());
    fix->SetContext(context);
    fix->SetPrecision(tol);
    fix->SetMaxTolerance(std::max(10.0 * tol, 1e-4));
    fix->Perform();
    face = orient_wires(fix->Face(), context->Value(built[0].wire));
    for (const BuiltLoop &loop : built) {
      for (const TopoDS_Edge &edge : loop.edges) {
        edges.push_back(context->Value(edge));
      }
    }
    break;
  }
  case 3: {
    if (point_starts.size() < 2) {
      fail("no polylines");
    }
    ShapeAnalysis_Surface sas(s);
    BRep_Builder builder;
    builder.MakeFace(face, s, Precision::Confusion());
    for (std::size_t l = 0; l + 1 < point_starts.size(); ++l) {
      std::vector<gp_Pnt> pts;
      for (std::uint32_t i = point_starts[l]; i < point_starts[l + 1]; ++i) {
        pts.push_back(pnt(points[i]));
      }
      builder.Add(face, polyline_wire(s, sas, pts, l == 0, tol, "polyline " + std::to_string(l)));
    }
    if (!BRepLib::BuildCurves3d(face, tol)) {
      fail("polyline bounds: could not build the 3D curves");
    }
    BRepLib::SameParameter(face, tol, true);
    break;
  }
  default:
    fail("unknown bounds kind " + std::to_string(bounds));
  }
  auto out = std::make_unique<OcBuild>();
  out->edge_maps.push_back(edge_map(face, edges));
  out->shapes.push_back(face);
  return out;
}

std::unique_ptr<OcBuild> make_wire(const OcLoopsIn &edges, bool closed, double tol) {
  (void)tol;
  if (edges.loop_starts.size() != 2) {
    fail("a wire is one loop of edges");
  }
  BuiltLoop loop = build_loop(edges.edges, edges.loop_starts[0], edges.loop_starts[1], closed,
                              "wire");
  auto out = std::make_unique<OcBuild>();
  std::vector<TopoDS_Shape> built(loop.edges.begin(), loop.edges.end());
  out->edge_maps.push_back(edge_map(loop.wire, built));
  out->shapes.push_back(loop.wire);
  return out;
}

std::uint32_t build_count(const OcBuild &build) {
  return static_cast<std::uint32_t>(build.shapes.size());
}

std::unique_ptr<OcShape> build_shape(const OcBuild &build, std::uint32_t i) {
  if (i >= build.shapes.size()) {
    throw std::out_of_range("build shape index out of range");
  }
  return std::make_unique<OcShape>(build.shapes[i]);
}

rust::Vec<std::uint32_t> build_edge_map(const OcBuild &build, std::uint32_t i) {
  if (i >= build.edge_maps.size()) {
    throw std::out_of_range("build edge map index out of range");
  }
  rust::Vec<std::uint32_t> out;
  out.reserve(build.edge_maps[i].size());
  for (std::uint32_t e : build.edge_maps[i]) {
    out.push_back(e);
  }
  return out;
}

} // namespace amg::occt
