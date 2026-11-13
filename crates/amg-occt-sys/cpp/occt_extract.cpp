// Geometry extraction for sketches and drawings (BREP-17): plane sections and B-spline forms of
// edges that have no analytic curve. Orthogonal projection itself is exact affine geometry and
// runs in Rust (amg-brep `extract`).

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomConvert.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <GeomConvert_CurveToAnaCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

namespace amg::occt {

namespace {

constexpr std::uint32_t NO_FACE = 0xFFFFFFFFu;

// Approximation settings for curves without an analytic or B-spline form.
constexpr int APPROX_MAX_SEGMENTS = 200;
constexpr int APPROX_MAX_DEGREE = 9;

V3 xyz(const gp_XYZ &p) { return V3{p.X(), p.Y(), p.Z()}; }

gp_XYZ from_v3(const V3 &v) { return gp_XYZ(v.x, v.y, v.z); }

// Same convention as occt_shim.cpp: keep (x, y), normal = x × y.
OcFrame frame_of(const gp_Ax3 &ax) {
  const gp_Dir x = ax.XDirection();
  const gp_Dir y = ax.YDirection();
  const gp_Dir normal = x.Crossed(y);
  return OcFrame{xyz(ax.Location().XYZ()), xyz(x.XYZ()), xyz(y.XYZ()), xyz(normal.XYZ())};
}

OcCurve absent() {
  OcCurve out{};
  out.kind = OcCurveKind::Absent;
  return out;
}

void copy_bspline(const occ::handle<Geom_BSplineCurve> &curve, double first, double last,
                  OcCurve &out) {
  out.kind = OcCurveKind::BSpline;
  out.first = first;
  out.last = last;
  out.degree = static_cast<std::uint32_t>(curve->Degree());
  out.periodic = curve->IsPeriodic();
  for (int i = 1; i <= curve->NbKnots(); ++i) {
    out.knots.push_back(curve->Knot(i));
    out.mults.push_back(static_cast<std::uint32_t>(curve->Multiplicity(i)));
  }
  const bool rational = curve->IsRational();
  for (int i = 1; i <= curve->NbPoles(); ++i) {
    out.poles.push_back(xyz(curve->Pole(i).XYZ()));
    if (rational) {
      out.weights.push_back(curve->Weight(i));
    }
  }
}

// Writes a line, circle or ellipse into `out`; false for any other type.
bool copy_analytic(const occ::handle<Geom_Curve> &curve, double first, double last,
                   OcCurve &out) {
  const GeomAdaptor_Curve adaptor(curve);
  switch (adaptor.GetType()) {
  case GeomAbs_Line: {
    const gp_Lin line = adaptor.Line();
    out.kind = OcCurveKind::Line;
    out.frame.origin = xyz(line.Location().XYZ());
    out.frame.normal = xyz(line.Direction().XYZ());
    break;
  }
  case GeomAbs_Circle: {
    const gp_Circ circle = adaptor.Circle();
    out.kind = OcCurveKind::Circle;
    out.frame = frame_of(gp_Ax3(circle.Position()));
    out.a = circle.Radius();
    break;
  }
  case GeomAbs_Ellipse: {
    const gp_Elips ellipse = adaptor.Ellipse();
    out.kind = OcCurveKind::Ellipse;
    out.frame = frame_of(gp_Ax3(ellipse.Position()));
    out.a = ellipse.MajorRadius();
    out.b = ellipse.MinorRadius();
    break;
  }
  default:
    return false;
  }
  out.first = first;
  out.last = last;
  return true;
}

// The B-spline approximation of an adaptor curve over its whole range, within `tol`.
OcCurve approximate(const occ::handle<Adaptor3d_Curve> &curve, double tol) {
  GeomConvert_ApproxCurve approx(curve, tol, GeomAbs_C2, APPROX_MAX_SEGMENTS, APPROX_MAX_DEGREE);
  if (!approx.HasResult()) {
    throw std::runtime_error("B-spline approximation of a curve failed");
  }
  const occ::handle<Geom_BSplineCurve> bspline = approx.Curve();
  OcCurve out{};
  copy_bspline(bspline, bspline->FirstParameter(), bspline->LastParameter(), out);
  return out;
}

// A trimmed curve as a line, circle or ellipse when it is one (within `tol`), else as a B-spline.
// Analytic curves are kept; B-splines and Beziers are tried for an analytic form; other types
// (parabola, hyperbola, offset, ...) become B-splines, exactly where OCCT can convert them and by
// approximation within `tol` otherwise.
OcCurve recognised(occ::handle<Geom_Curve> curve, double first, double last, double tol) {
  for (occ::handle<Geom_TrimmedCurve> trimmed = Handle(Geom_TrimmedCurve)::DownCast(curve);
       !trimmed.IsNull(); trimmed = Handle(Geom_TrimmedCurve)::DownCast(curve)) {
    curve = trimmed->BasisCurve();
  }
  OcCurve out{};
  if (copy_analytic(curve, first, last, out)) {
    return out;
  }
  occ::handle<Geom_BSplineCurve> bspline = Handle(Geom_BSplineCurve)::DownCast(curve);
  if (!Handle(Geom_BezierCurve)::DownCast(curve).IsNull()) {
    bspline = GeomConvert::CurveToBSplineCurve(curve);
  }
  if (!bspline.IsNull()) {
    if (bspline->IsPeriodic()) {
      // A non-periodic copy of the edge's range, so the data needs no periodic evaluation.
      const occ::handle<Geom_TrimmedCurve> range = new Geom_TrimmedCurve(bspline, first, last);
      bspline = GeomConvert::CurveToBSplineCurve(range);
      first = bspline->FirstParameter();
      last = bspline->LastParameter();
    }
    GeomConvert_CurveToAnaCurve converter(bspline);
    occ::handle<Geom_Curve> analytic;
    double new_first = first;
    double new_last = last;
    if (converter.ConvertToAnalytical(tol, analytic, first, last, new_first, new_last) &&
        !analytic.IsNull() && copy_analytic(analytic, new_first, new_last, out)) {
      return out;
    }
    copy_bspline(bspline, first, last, out);
    return out;
  }
  const occ::handle<Geom_TrimmedCurve> piece = new Geom_TrimmedCurve(curve, first, last);
  if (Handle(Geom_OffsetCurve)::DownCast(curve).IsNull()) {
    try {
      const occ::handle<Geom_BSplineCurve> exact = GeomConvert::CurveToBSplineCurve(piece);
      copy_bspline(exact, exact->FirstParameter(), exact->LastParameter(), out);
      return out;
    } catch (const Standard_Failure &) {
      // Falls through to the approximation.
    }
  }
  return approximate(new GeomAdaptor_Curve(piece), tol);
}

// A finite face on `plane` that covers `box` (enlarged), so the section never meets an
// infinite face.
TopoDS_Face covering_face(const gp_Pln &plane, const Bnd_Box &box) {
  double x0 = 0.0, y0 = 0.0, z0 = 0.0, x1 = 0.0, y1 = 0.0, z1 = 0.0;
  box.Get(x0, y0, z0, x1, y1, z1);
  const gp_Ax3 &pos = plane.Position();
  const gp_XYZ o = pos.Location().XYZ();
  const gp_XYZ ux = pos.XDirection().XYZ();
  const gp_XYZ uy = pos.YDirection().XYZ();
  double umin = std::numeric_limits<double>::max();
  double umax = -umin;
  double vmin = umin;
  double vmax = -umin;
  for (int i = 0; i < 8; ++i) {
    const gp_XYZ corner((i & 1) ? x1 : x0, (i & 2) ? y1 : y0, (i & 4) ? z1 : z0);
    const gp_XYZ d = corner - o;
    umin = std::min(umin, d.Dot(ux));
    umax = std::max(umax, d.Dot(ux));
    vmin = std::min(vmin, d.Dot(uy));
    vmax = std::max(vmax, d.Dot(uy));
  }
  const double margin = 1.0 + 0.1 * std::max(umax - umin, vmax - vmin);
  BRepBuilderAPI_MakeFace maker(plane, umin - margin, umax + margin, vmin - margin,
                                vmax + margin);
  if (!maker.IsDone()) {
    throw std::runtime_error("plane_section: could not build the cutting face");
  }
  return maker.Face();
}

} // namespace

OcSection plane_section(const OcShape &shape, const OcIndex &index, const OcFrame &plane,
                        double tol) {
  if (!(tol > 0.0) || !std::isfinite(tol)) {
    throw std::invalid_argument("plane_section: tolerance must be positive and finite");
  }
  OcSection out{};
  Bnd_Box box;
  BRepBndLib::Add(shape.get(), box, false);
  if (box.IsVoid()) {
    return out;
  }
  const gp_Pln pln(gp_Ax3(gp_Pnt(from_v3(plane.origin)), gp_Dir(from_v3(plane.normal)),
                          gp_Dir(from_v3(plane.x))));
  const TopoDS_Face tool = covering_face(pln, box);

  BRepAlgoAPI_Section section(shape.get(), tool, false);
  section.Approximation(true);
  section.ComputePCurveOn1(false);
  section.ComputePCurveOn2(false);
  section.SetNonDestructive(true);
  section.SetRunParallel(false);
  section.Build();
  if (!section.IsDone() || section.HasErrors()) {
    throw std::runtime_error("plane_section: BRepAlgoAPI_Section failed");
  }

  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges;
  TopExp::MapShapes(section.Shape(), TopAbs_EDGE, edges);
  for (int i = 1; i <= edges.Extent(); ++i) {
    const TopoDS_Edge &edge = TopoDS::Edge(edges(i));
    if (BRep_Tool::Degenerated(edge)) {
      continue;
    }
    double first = 0.0;
    double last = 0.0;
    const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(edge, first, last);
    OcCurve seg = curve.IsNull() ? approximate(new BRepAdaptor_Curve(edge), tol)
                                 : recognised(curve, first, last, tol);
    TopoDS_Shape face;
    std::uint32_t face_index = NO_FACE;
    if (section.HasAncestorFaceOn1(edge, face)) {
      const int found = index.faces.FindIndex(face);
      if (found > 0) {
        face_index = static_cast<std::uint32_t>(found - 1);
      }
    }
    out.curves.push_back(std::move(seg));
    out.faces.push_back(face_index);
  }
  return out;
}

OcCurve edge_curve_recognised(const OcIndex &index, std::uint32_t e, double tol) {
  if (!(tol > 0.0) || !std::isfinite(tol)) {
    throw std::invalid_argument("edge_curve_recognised: tolerance must be positive and finite");
  }
  const TopoDS_Edge &edge = TopoDS::Edge(index.edges.FindKey(static_cast<int>(e) + 1));
  if (BRep_Tool::Degenerated(edge)) {
    return absent();
  }
  double first = 0.0;
  double last = 0.0;
  const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(edge, first, last);
  if (curve.IsNull()) {
    // No 3D curve (pcurves only): approximate the curve on the surface.
    return approximate(new BRepAdaptor_Curve(edge), tol);
  }
  return recognised(curve, first, last, tol);
}

} // namespace amg::occt
