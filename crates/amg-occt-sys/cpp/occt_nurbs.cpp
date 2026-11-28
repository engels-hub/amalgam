// NURBS conversion and oracles (S20 KH-16). Declarations and categories are in occt_shim.hxx.
//
// amg-brep converts amg-nurbs curves and surfaces to and from the bridge's plain B-spline data;
// these functions build `Geom_BSpline*` objects from that data and copy them back out, evaluate
// them (the differential-test oracle), order the edges of a wire, and run `GeomFill_Gordon`
// (the KH-15 oracle). Every function takes plain data or only reads a shape, so all are
// category R.

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <cstdint>
#include <string>

#include <BRepTools_WireExplorer.hxx>
#include <GeomFill_Gordon.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace amg::occt {

namespace {

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

gp_Pnt pnt(const V3 &v) { return gp_Pnt(v.x, v.y, v.z); }
V3 v3(const gp_XYZ &p) { return V3{p.X(), p.Y(), p.Z()}; }

NCollection_Array1<double> reals(const rust::Vec<double> &v) {
  NCollection_Array1<double> out(1, static_cast<int>(v.size()));
  for (std::size_t i = 0; i < v.size(); ++i) {
    out.SetValue(static_cast<int>(i) + 1, v[i]);
  }
  return out;
}

NCollection_Array1<int> ints(const rust::Vec<std::uint32_t> &v) {
  NCollection_Array1<int> out(1, static_cast<int>(v.size()));
  for (std::size_t i = 0; i < v.size(); ++i) {
    out.SetValue(static_cast<int>(i) + 1, static_cast<int>(v[i]));
  }
  return out;
}

occ::handle<Geom_BSplineCurve> curve_in(const OcCurve &c) {
  if (c.kind != OcCurveKind::BSpline) {
    fail("the curve is not a B-spline");
  }
  if (c.knots.empty() || c.knots.size() != c.mults.size() || c.poles.empty()) {
    fail("malformed B-spline curve data");
  }
  const int np = static_cast<int>(c.poles.size());
  NCollection_Array1<gp_Pnt> poles(1, np);
  for (int i = 0; i < np; ++i) {
    poles.SetValue(i + 1, pnt(c.poles[i]));
  }
  const int degree = static_cast<int>(c.degree);
  if (c.weights.empty()) {
    return new Geom_BSplineCurve(poles, reals(c.knots), ints(c.mults), degree, c.periodic);
  }
  if (c.weights.size() != c.poles.size()) {
    fail("B-spline curve weights and poles differ in length");
  }
  return new Geom_BSplineCurve(poles, reals(c.weights), reals(c.knots), ints(c.mults), degree,
                               c.periodic);
}

occ::handle<Geom_BSplineSurface> surface_in(const OcSurface &s) {
  if (s.kind != OcSurfaceKind::BSpline) {
    fail("the surface is not a B-spline");
  }
  const std::size_t n = static_cast<std::size_t>(s.nu) * s.nv;
  if (s.nu == 0 || s.nv == 0 || s.poles.size() != n ||
      (!s.weights.empty() && s.weights.size() != n) || s.u_knots.size() != s.u_mults.size() ||
      s.v_knots.size() != s.v_mults.size()) {
    fail("malformed B-spline surface data");
  }
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
  const int du = static_cast<int>(s.u_degree);
  const int dv = static_cast<int>(s.v_degree);
  if (s.weights.empty()) {
    return new Geom_BSplineSurface(poles, reals(s.u_knots), reals(s.v_knots), ints(s.u_mults),
                                   ints(s.v_mults), du, dv, s.u_periodic, s.v_periodic);
  }
  return new Geom_BSplineSurface(poles, weights, reals(s.u_knots), reals(s.v_knots),
                                 ints(s.u_mults), ints(s.v_mults), du, dv, s.u_periodic,
                                 s.v_periodic);
}

OcCurve curve_out(const occ::handle<Geom_BSplineCurve> &curve) {
  OcCurve out{};
  out.kind = OcCurveKind::BSpline;
  out.first = curve->FirstParameter();
  out.last = curve->LastParameter();
  out.degree = static_cast<std::uint32_t>(curve->Degree());
  out.periodic = curve->IsPeriodic();
  for (int i = 1; i <= curve->NbKnots(); ++i) {
    out.knots.push_back(curve->Knot(i));
    out.mults.push_back(static_cast<std::uint32_t>(curve->Multiplicity(i)));
  }
  const bool rational = curve->IsRational();
  for (int i = 1; i <= curve->NbPoles(); ++i) {
    out.poles.push_back(v3(curve->Pole(i).XYZ()));
    if (rational) {
      out.weights.push_back(curve->Weight(i));
    }
  }
  return out;
}

OcSurface surface_out(const occ::handle<Geom_BSplineSurface> &surface) {
  OcSurface out{};
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
      out.poles.push_back(v3(surface->Pole(i, j).XYZ()));
      if (rational) {
        out.weights.push_back(surface->Weight(i, j));
      }
    }
  }
  return out;
}

} // namespace

OcCurve bspline_curve_round_trip(const OcCurve &curve) { return curve_out(curve_in(curve)); }

OcSurface bspline_surface_round_trip(const OcSurface &surface) {
  return surface_out(surface_in(surface));
}

OcCurve bspline_curve_not_periodic(const OcCurve &curve) {
  const occ::handle<Geom_BSplineCurve> c = curve_in(curve);
  c->SetNotPeriodic();
  return curve_out(c);
}

OcSurface bspline_surface_not_periodic(const OcSurface &surface) {
  const occ::handle<Geom_BSplineSurface> s = surface_in(surface);
  s->SetUNotPeriodic();
  s->SetVNotPeriodic();
  return surface_out(s);
}

rust::Vec<V3> bspline_curve_d2(const OcCurve &curve, rust::Slice<const double> ts) {
  const occ::handle<Geom_BSplineCurve> c = curve_in(curve);
  rust::Vec<V3> out;
  out.reserve(3 * ts.size());
  gp_Pnt p;
  gp_Vec d1;
  gp_Vec d2;
  for (const double t : ts) {
    c->D2(t, p, d1, d2);
    out.push_back(v3(p.XYZ()));
    out.push_back(v3(d1.XYZ()));
    out.push_back(v3(d2.XYZ()));
  }
  return out;
}

rust::Vec<V3> bspline_surface_d2(const OcSurface &surface, rust::Slice<const double> uvs) {
  if (uvs.size() % 2 != 0) {
    fail("parameters come in (u, v) pairs");
  }
  const occ::handle<Geom_BSplineSurface> s = surface_in(surface);
  rust::Vec<V3> out;
  out.reserve(3 * uvs.size());
  gp_Pnt p;
  gp_Vec du;
  gp_Vec dv;
  gp_Vec duu;
  gp_Vec dvv;
  gp_Vec duv;
  for (std::size_t i = 0; i + 1 < uvs.size(); i += 2) {
    s->D2(uvs[i], uvs[i + 1], p, du, dv, duu, dvv, duv);
    out.push_back(v3(p.XYZ()));
    out.push_back(v3(du.XYZ()));
    out.push_back(v3(dv.XYZ()));
    out.push_back(v3(duu.XYZ()));
    out.push_back(v3(dvv.XYZ()));
    out.push_back(v3(duv.XYZ()));
  }
  return out;
}

rust::Vec<std::uint32_t> wire_edge_order(const OcShape &shape, const OcIndex &index) {
  const TopoDS_Shape &s = shape.get();
  rust::Vec<std::uint32_t> out;
  const auto push = [&](const TopoDS_Edge &edge) {
    const int found = index.edges.FindIndex(edge);
    if (found == 0) {
      fail("a wire edge is not in the shape's index");
    }
    out.push_back(static_cast<std::uint32_t>(found - 1));
    out.push_back(edge.Orientation() == TopAbs_REVERSED ? 1u : 0u);
  };
  if (s.ShapeType() == TopAbs_EDGE) {
    push(TopoDS::Edge(s));
    return out;
  }
  if (s.ShapeType() != TopAbs_WIRE) {
    fail("the shape is not a wire or an edge");
  }
  std::size_t stored = 0;
  for (TopoDS_Iterator it(s); it.More(); it.Next()) {
    if (it.Value().ShapeType() == TopAbs_EDGE) {
      ++stored;
    }
  }
  for (BRepTools_WireExplorer exp(TopoDS::Wire(s)); exp.More(); exp.Next()) {
    push(exp.Current());
  }
  if (out.size() != 2 * stored) {
    fail("the wire is not a simple chain (the wire explorer missed edges)");
  }
  return out;
}

OcGordon gordon_surface(rust::Slice<const OcCurve> profiles, rust::Slice<const OcCurve> guides,
                        double tol) {
  NCollection_Array1<occ::handle<Geom_Curve>> p(1, static_cast<int>(profiles.size()));
  NCollection_Array1<occ::handle<Geom_Curve>> g(1, static_cast<int>(guides.size()));
  for (std::size_t i = 0; i < profiles.size(); ++i) {
    p.SetValue(static_cast<int>(i) + 1, curve_in(profiles[i]));
  }
  for (std::size_t i = 0; i < guides.size(); ++i) {
    g.SetValue(static_cast<int>(i) + 1, curve_in(guides[i]));
  }
  GeomFill_Gordon gordon;
  gordon.Init(p, g, tol);
  gordon.SetParallelMode(false);
  gordon.Perform();
  const GeomFill_Gordon::BuildReport &report = gordon.Report();
  OcGordon out{};
  out.status = static_cast<std::uint8_t>(report.Status);
  out.approximate = report.IsApproximate;
  out.max_contact_gap = report.MaxContactGap;
  out.max_profile_deviation = report.MaxProfileDeviation;
  out.max_guide_deviation = report.MaxGuideDeviation;
  out.surface.kind = OcSurfaceKind::Other;
  if (gordon.IsDone() && !gordon.Surface().IsNull()) {
    out.surface = surface_out(gordon.Surface());
  }
  return out;
}

} // namespace amg::occt
