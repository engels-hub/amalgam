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
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeSegment2d.hxx>
#include <Geom2dAPI_Interpolate.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAbs_JoinType.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomConvert_ApproxCurve.hxx>
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

// The accuracy of the B-splines that replace the offset curves of thin walls, mm.
constexpr double THIN_APPROX = 1e-9;

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

// ---- thin faces (S04 R-S04-3) ------------------------------------------------------------

// One boundary of a thin face: edges, each with the index of the chain edge it came from.
struct ThinSideOut {
  std::vector<TopoDS_Edge> edges;
  std::vector<std::size_t> source;
};

// The signed side of `q` against the chain edge `e` (run along the chain): positive when `q` is
// left of the chain seen against the plane normal `n`.
double chain_side(const TopoDS_Edge &e, const gp_Pnt &q, const gp_Dir &n) {
  double first = 0.0;
  double last = 0.0;
  const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(e, first, last);
  GeomAPI_ProjectPointOnCurve project(q, curve, first, last);
  double u = 0.0;
  if (project.NbPoints() > 0) {
    u = project.LowerDistanceParameter();
  } else {
    u = q.Distance(curve->Value(first)) < q.Distance(curve->Value(last)) ? first : last;
  }
  gp_Pnt p;
  gp_Vec t;
  curve->D1(u, p, t);
  if (e.Orientation() == TopAbs_REVERSED) {
    t.Reverse();
  }
  return t.Crossed(gp_Vec(p, q)).Dot(gp_Vec(n));
}

// One edge for the offset `pieces` of the chain edge `source`, ending at the pieces' outer
// vertices; null when they are not one offset of its curve.
TopoDS_Edge joined_offset(const std::vector<TopoDS_Edge> &pieces, const TopoDS_Edge &source) {
  double first = 0.0;
  double last = 0.0;
  const occ::handle<Geom_Curve> spine = BRep_Tool::Curve(source, first, last);
  occ::handle<Geom_OffsetCurve> whole;
  double lo = RealLast();
  double hi = RealFirst();
  double span = 0.0;
  std::vector<TopoDS_Vertex> vertices;
  for (const TopoDS_Edge &piece : pieces) {
    double f = 0.0;
    double l = 0.0;
    const auto curve = occ::down_cast<Geom_OffsetCurve>(BRep_Tool::Curve(piece, f, l));
    if (curve.IsNull()) {
      return TopoDS_Edge();
    }
    if (whole.IsNull()) {
      whole = new Geom_OffsetCurve(spine, curve->Offset(), curve->Direction());
    }
    for (const double u : {f, 0.5 * (f + l), l}) {
      if (whole->Value(u).Distance(curve->Value(u)) > 1e-9) {
        return TopoDS_Edge();
      }
    }
    lo = std::min(lo, f);
    hi = std::max(hi, l);
    span += l - f;
    vertices.push_back(TopExp::FirstVertex(piece));
    vertices.push_back(TopExp::LastVertex(piece));
  }
  // The pieces cover the range once, without a hole.
  if (std::abs(span - (hi - lo)) > 1e-9 * std::max(1.0, hi - lo)) {
    return TopoDS_Edge();
  }
  const auto vertex_at = [&](double u) {
    const gp_Pnt at = whole->Value(u);
    std::size_t best = 0;
    for (std::size_t i = 1; i < vertices.size(); ++i) {
      if (BRep_Tool::Pnt(vertices[i]).Distance(at) < BRep_Tool::Pnt(vertices[best]).Distance(at)) {
        best = i;
      }
    }
    return vertices[best];
  };
  const TopoDS_Vertex v_lo = vertex_at(lo);
  const bool closed = whole->Value(lo).Distance(whole->Value(hi)) <= 1e-9;
  const TopoDS_Vertex v_hi = closed ? v_lo : vertex_at(hi);
  BRepBuilderAPI_MakeEdge make(whole, v_lo, v_hi, lo, hi);
  if (!make.IsDone()) {
    return TopoDS_Edge();
  }
  TopoDS_Edge edge = make.Edge();
  edge.Orientation(pieces.front().Orientation());
  return edge;
}

// MakeOffset cuts the parallel of a spline or ellipse where the curve's curvature peaks. The
// pieces of one chain edge are one offset curve over its parameter range: they are joined back
// into one edge, so every chain edge gives one face (and one name). Pieces that are not offset
// curves over that parameterisation stay as they are.
void join_offset_pieces(ThinSideOut &side, const BuiltLoop &chain) {
  ThinSideOut out;
  for (std::size_t k = 0; k < chain.edges.size(); ++k) {
    std::vector<TopoDS_Edge> pieces;
    for (std::size_t i = 0; i < side.edges.size(); ++i) {
      if (side.source[i] == k) {
        pieces.push_back(side.edges[i]);
      }
    }
    const TopoDS_Edge joined = pieces.size() > 1 ? joined_offset(pieces, chain.edges[k])
                                                 : TopoDS_Edge();
    if (!joined.IsNull()) {
      pieces.assign(1, joined);
    }
    for (const TopoDS_Edge &piece : pieces) {
      out.edges.push_back(piece);
      out.source.push_back(k);
    }
  }
  side = out;
}

// `edge` on a B-spline within THIN_APPROX of its offset curve, between the same vertices. An
// offset curve gets no faithful pcurve on the wall's plane (its tolerance would grow with the
// wall, and volumes would be wrong); a B-spline's pcurve is exact.
TopoDS_Edge bspline_edge(const TopoDS_Edge &edge) {
  double first = 0.0;
  double last = 0.0;
  const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(edge, first, last);
  if (occ::down_cast<Geom_OffsetCurve>(curve).IsNull()) {
    return edge;
  }
  const occ::handle<Geom_TrimmedCurve> trimmed = new Geom_TrimmedCurve(curve, first, last);
  GeomConvert_ApproxCurve approx(trimmed, THIN_APPROX, GeomAbs_C2, 1000, 8);
  if (!approx.HasResult() || approx.MaxError() > 10.0 * THIN_APPROX) {
    fail("could not approximate an offset curve of the wall");
  }
  const occ::handle<Geom_BSplineCurve> spline = approx.Curve();
  const TopoDS_Edge forward = TopoDS::Edge(edge.Oriented(TopAbs_FORWARD));
  BRepBuilderAPI_MakeEdge make(spline, TopExp::FirstVertex(forward), TopExp::LastVertex(forward),
                               spline->FirstParameter(), spline->LastParameter());
  if (!make.IsDone()) {
    fail("could not build an offset edge of the wall: BRepBuilderAPI_MakeEdge error " +
         std::to_string(static_cast<int>(make.Error())));
  }
  return TopoDS::Edge(make.Edge().Oriented(edge.Orientation()));
}

// The boundary `distance` mm to the left (`left`) or right of the chain: the chain itself for
// distance 0, else its BRepOffsetAPI_MakeOffset parallel with sharp (intersection) corners.
// The side MakeOffset picks follows the spine face's normal (the sign of the distance is not
// honoured for every wire), so the side of the result is checked and the offset repeated on the
// face turned over when it came out on the wrong side.
ThinSideOut thin_side(const gp_Pln &plane, const BuiltLoop &chain, bool closed, double distance,
                      bool left) {
  ThinSideOut out;
  if (distance == 0.0) {
    for (std::size_t k = 0; k < chain.edges.size(); ++k) {
      out.edges.push_back(chain.edges[k]);
      out.source.push_back(k);
    }
    return out;
  }
  const std::string where = std::string(left ? "left" : "right") + " offset by " +
                            std::to_string(distance) + " mm";
  // A lone circle: MakeOffset only ever shrinks it, and its parallel is a circle anyway.
  if (closed && chain.edges.size() == 1) {
    double first = 0.0;
    double last = 0.0;
    const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(chain.edges[0], first, last);
    if (const occ::handle<Geom_Circle> circle = occ::down_cast<Geom_Circle>(curve);
        !circle.IsNull()) {
      const bool centre_left =
          chain_side(chain.edges[0], circle->Location(), plane.Axis().Direction()) > 0.0;
      const double radius = circle->Radius() + (centre_left == left ? -distance : distance);
      if (radius <= Precision::Confusion()) {
        fail(where + ": the circle's radius is smaller than the wall's width");
      }
      BRepBuilderAPI_MakeEdge make(new Geom_Circle(circle->Position(), radius), first, last);
      if (!make.IsDone()) {
        fail(where + ": could not build the parallel circle");
      }
      out.edges.push_back(make.Edge());
      out.source.push_back(0);
      return out;
    }
  }
  const gp_Pln turned(gp_Ax3(plane.Location(), plane.Axis().Direction().Reversed(),
                             plane.XAxis().Direction()));
  for (const auto &[spine, signed_distance] :
       {std::pair<gp_Pln, double>{plane, distance}, std::pair<gp_Pln, double>{turned, distance},
        std::pair<gp_Pln, double>{plane, -distance},
        std::pair<gp_Pln, double>{turned, -distance}}) {
    BRep_Builder builder;
    TopoDS_Face spine_face;
    builder.MakeFace(spine_face, new Geom_Plane(spine), Precision::Confusion());
    BRepOffsetAPI_MakeOffset offset;
    offset.Init(spine_face, GeomAbs_Intersection, !closed);
    offset.AddWire(chain.wire);
    offset.Perform(signed_distance);
    if (!offset.IsDone() || offset.Shape().IsNull()) {
      continue;
    }
    ShapeMap result;
    TopExp::MapShapes(offset.Shape(), TopAbs_EDGE, result);
    if (result.Extent() == 0) {
      continue;
    }
    ThinSideOut got;
    std::vector<bool> used(static_cast<std::size_t>(result.Extent()), false);
    for (std::size_t k = 0; k < chain.edges.size(); ++k) {
      const NCollection_List<TopoDS_Shape> &made = offset.Generated(chain.edges[k]);
      for (NCollection_List<TopoDS_Shape>::Iterator it(made); it.More(); it.Next()) {
        const int found = result.FindIndex(it.Value());
        if (found != 0 && !used[static_cast<std::size_t>(found - 1)]) {
          used[static_cast<std::size_t>(found - 1)] = true;
          got.edges.push_back(TopoDS::Edge(result.FindKey(found)));
          got.source.push_back(k);
        }
      }
    }
    if (got.edges.empty()) {
      continue;
    }
    double first = 0.0;
    double last = 0.0;
    const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(got.edges[0], first, last);
    const gp_Pnt mid = curve->Value(0.5 * (first + last));
    const double side = chain_side(chain.edges[got.source[0]], mid, plane.Axis().Direction());
    if ((side > 0.0) != left) {
      continue;
    }
    if (got.edges.size() != used.size()) {
      fail(where + ": " + std::to_string(used.size() - got.edges.size()) + " of " +
           std::to_string(used.size()) + " offset edges have no source edge");
    }
    join_offset_pieces(got, chain);
    for (TopoDS_Edge &e : got.edges) {
      e = bspline_edge(e);
    }
    return got;
  }
  fail(where + ": OCCT built no parallel on that side (the wall may be wider than the profile "
               "allows, or a corner too sharp for it)");
}

// The two end vertices of an open boundary (the vertices only one edge uses), ordered so the
// first belongs to the chain's start.
std::pair<TopoDS_Vertex, TopoDS_Vertex> boundary_ends(const ThinSideOut &side,
                                                      const gp_Pnt &start, const gp_Pnt &end,
                                                      double distance) {
  std::vector<TopoDS_Vertex> seen;
  std::vector<int> uses;
  for (const TopoDS_Edge &e : side.edges) {
    TopoDS_Vertex v[2];
    TopExp::Vertices(e, v[0], v[1]);
    for (const TopoDS_Vertex &x : v) {
      std::size_t i = 0;
      while (i < seen.size() && !seen[i].IsSame(x)) {
        ++i;
      }
      if (i == seen.size()) {
        seen.push_back(x);
        uses.push_back(0);
      }
      ++uses[i];
    }
  }
  std::vector<TopoDS_Vertex> open;
  for (std::size_t i = 0; i < seen.size(); ++i) {
    if (uses[i] == 1) {
      open.push_back(seen[i]);
    }
  }
  if (open.size() != 2) {
    fail("an offset of the chain is not one open wire (" + std::to_string(open.size()) +
         " free ends); reduce the thickness");
  }
  const auto misfit = [&](const TopoDS_Vertex &a, const TopoDS_Vertex &b) {
    return std::abs(BRep_Tool::Pnt(a).Distance(start) - distance) +
           std::abs(BRep_Tool::Pnt(b).Distance(end) - distance);
  };
  if (misfit(open[0], open[1]) <= misfit(open[1], open[0])) {
    return {open[0], open[1]};
  }
  return {open[1], open[0]};
}

// Twice the signed (u, v) area of `wire` on `face`, from its pcurves in connected order (the
// order of the wire's edge list is not: wire_uv_area2 assumes it is).
double connected_uv_area2(const TopoDS_Wire &wire, const TopoDS_Face &face) {
  std::vector<gp_Pnt2d> pts;
  for (BRepTools_WireExplorer it(wire, face); it.More(); it.Next()) {
    const TopoDS_Edge &edge = it.Current();
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

// `face` with the wire enclosing the largest area counter-clockwise and the others clockwise.
TopoDS_Face orient_thin(const TopoDS_Face &face) {
  std::vector<TopoDS_Wire> wires;
  std::vector<double> areas;
  std::size_t outer = 0;
  for (TopoDS_Iterator it(face); it.More(); it.Next()) {
    wires.push_back(TopoDS::Wire(it.Value()));
    areas.push_back(connected_uv_area2(wires.back(), face));
    if (std::abs(areas.back()) > std::abs(areas[outer])) {
      outer = areas.size() - 1;
    }
  }
  BRep_Builder builder;
  TopoDS_Face out = TopoDS::Face(face.EmptyCopied());
  for (std::size_t w = 0; w < wires.size(); ++w) {
    TopoDS_Wire wire = wires[w];
    if ((areas[w] > 0.0) != (w == outer)) {
      wire.Reverse();
    }
    builder.Add(out, wire);
  }
  return out;
}

// `edges` joined into one closed wire; every edge keeps its identity.
TopoDS_Wire thin_wire(const std::vector<TopoDS_Edge> &edges) {
  NCollection_List<TopoDS_Shape> list;
  for (const TopoDS_Edge &e : edges) {
    list.Append(e);
  }
  BRepBuilderAPI_MakeWire make;
  make.Add(list);
  if (!make.IsDone()) {
    fail("the wall boundary does not join into one wire; reduce the thickness");
  }
  TopoDS_Wire wire = make.Wire();
  if (!BRep_Tool::IsClosed(wire)) {
    fail("the wall boundary is not closed; reduce the thickness");
  }
  ShapeMap map;
  TopExp::MapShapes(wire, TopAbs_EDGE, map);
  if (static_cast<std::size_t>(map.Extent()) != edges.size()) {
    fail("the wall boundary splits into several wires; reduce the thickness");
  }
  return wire;
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

std::unique_ptr<OcBuild> make_thin_face(const OcPlanarIn &chain, bool closed, double left,
                                        double right, double tol) {
  const auto &starts = chain.loops.loop_starts;
  if (starts.size() != 2) {
    fail("a thin face is built from one chain");
  }
  const gp_Pln plane(ax3(chain.frame));
  const BuiltLoop spine = build_loop(chain.loops.edges, starts[0], starts[1], closed, "chain");
  const ThinSideOut l = thin_side(plane, spine, closed, left, true);
  const ThinSideOut r = thin_side(plane, spine, closed, right, false);
  const std::size_t n = spine.edges.size();
  std::vector<TopoDS_Edge> all;
  std::vector<std::uint32_t> codes;
  const auto add = [&](const TopoDS_Edge &e, std::size_t code) {
    all.push_back(e);
    codes.push_back(static_cast<std::uint32_t>(code));
  };
  for (std::size_t i = 0; i < l.edges.size(); ++i) {
    add(l.edges[i], 2 * l.source[i]);
  }
  for (std::size_t i = 0; i < r.edges.size(); ++i) {
    add(r.edges[i], 2 * r.source[i] + 1);
  }
  std::vector<TopoDS_Wire> wires;
  if (closed) {
    wires.push_back(thin_wire(l.edges));
    wires.push_back(thin_wire(r.edges));
  } else {
    const gp_Pnt start = BRep_Tool::Pnt(TopExp::FirstVertex(spine.edges.front(), true));
    const gp_Pnt end = BRep_Tool::Pnt(TopExp::LastVertex(spine.edges.back(), true));
    const auto le = boundary_ends(l, start, end, left);
    const auto re = boundary_ends(r, start, end, right);
    std::vector<TopoDS_Edge> ring = l.edges;
    ring.insert(ring.end(), r.edges.begin(), r.edges.end());
    const std::pair<TopoDS_Vertex, TopoDS_Vertex> caps[2] = {{le.first, re.first},
                                                             {le.second, re.second}};
    for (std::size_t c = 0; c < 2; ++c) {
      BRepBuilderAPI_MakeEdge make(caps[c].first, caps[c].second);
      if (!make.IsDone()) {
        fail("could not close the wall at a chain end");
      }
      ring.push_back(make.Edge());
      add(make.Edge(), 2 * n + c);
    }
    wires.push_back(thin_wire(ring));
  }
  BRep_Builder builder;
  TopoDS_Face face;
  builder.MakeFace(face, new Geom_Plane(plane), std::max(tol, Precision::Confusion()));
  for (const TopoDS_Wire &wire : wires) {
    builder.Add(face, wire);
  }
  NCollection_List<TopoDS_Shape> edge_list;
  for (const TopoDS_Edge &e : all) {
    edge_list.Append(e);
  }
  BRepLib::BuildPCurveForEdgesOnPlane(edge_list, face);
  face = orient_thin(face);
  if (!BRepCheck_Analyzer(face).IsValid()) {
    fail("the wall overlaps itself; reduce the thickness");
  }
  std::vector<TopoDS_Shape> edges(all.begin(), all.end());
  const std::vector<std::uint32_t> map = edge_map(face, edges);
  // Boundaries that touch pass BRepCheck but not as the wires they were built from.
  if (std::find(map.begin(), map.end(), LOST_EDGE) != map.end()) {
    fail("the wall touches itself; reduce the thickness");
  }
  std::vector<std::uint32_t> pairs;
  for (std::size_t i = 0; i < map.size(); ++i) {
    pairs.push_back(map[i]);
    pairs.push_back(codes[i]);
  }
  auto out = std::make_unique<OcBuild>();
  out->edge_maps.push_back(pairs);
  out->shapes.push_back(face);
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
