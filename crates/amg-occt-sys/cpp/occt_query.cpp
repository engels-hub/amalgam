// Measurement queries (BREP-09): mass properties, point classification, distances, ray casts,
// surface evaluation with curvatures, point projection onto a face and edge end tangents.
// Declarations and categories are in occt_shim.hxx.
//
// Every function here is category R: the algorithms only read the shapes (BRepGProp,
// BRepClass3d, BRepExtrema, IntCurvesFace and BRepLProp build their own adaptors and bounding
// boxes; none of them writes to a TShape). Nothing is added to a compound either, since
// TopoDS_Builder::Add clears the added TShape's Free flag (the BREP-04 finding).
//
// Orientation and sign conventions are applied in Rust (amg-brep `query`): this file returns
// OCCT's natural surface normal and curvatures, and the face orientation comes from the index.

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepExtrema_SupportType.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopAbs_State.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <Geom_Surface.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

namespace amg::occt {

namespace {

constexpr std::uint8_t NO_SUPPORT = 255;
constexpr std::uint32_t NO_FACE = 0xFFFFFFFFu;

V3 v3(const gp_XYZ &p) { return V3{p.X(), p.Y(), p.Z()}; }

gp_Pnt pnt(const V3 &p) { return gp_Pnt(p.x, p.y, p.z); }

const TopoDS_Face &face_at(const OcIndex &index, std::uint32_t f) {
  if (f >= static_cast<std::uint32_t>(index.faces.Extent())) {
    throw std::out_of_range("face index out of range");
  }
  return TopoDS::Face(index.faces.FindKey(static_cast<int>(f) + 1));
}

const TopoDS_Edge &edge_at(const OcIndex &index, std::uint32_t e) {
  if (e >= static_cast<std::uint32_t>(index.edges.Extent())) {
    throw std::out_of_range("edge index out of range");
  }
  return TopoDS::Edge(index.edges.FindKey(static_cast<int>(e) + 1));
}

// The index (kind, i) of a BRepExtrema support in `index`, or NO_SUPPORT.
void support_in(const OcIndex &index, BRepExtrema_SupportType type, const TopoDS_Shape &support,
                std::uint8_t &kind, std::uint32_t &i) {
  kind = NO_SUPPORT;
  i = 0;
  int found = 0;
  switch (type) {
  case BRepExtrema_IsInFace:
    found = index.faces.FindIndex(support);
    kind = 0;
    break;
  case BRepExtrema_IsOnEdge:
    found = index.edges.FindIndex(support);
    kind = 1;
    break;
  case BRepExtrema_IsVertex:
    found = index.vertices.FindIndex(support);
    kind = 2;
    break;
  }
  if (found == 0) {
    kind = NO_SUPPORT;
    return;
  }
  i = static_cast<std::uint32_t>(found - 1);
}

// The minimum distance between `s1` and `s2`, first solution (BRepExtrema lists solutions in a
// fixed order for the same input, so the choice is deterministic).
OcDistance run_distance(const TopoDS_Shape &s1, const OcIndex *i1, const TopoDS_Shape &s2,
                        const OcIndex *i2) {
  BRepExtrema_DistShapeShape dist;
  dist.SetFlag(Extrema_ExtFlag_MIN);
  dist.SetMultiThread(false);
  dist.LoadS1(s1);
  dist.LoadS2(s2);
  if (!dist.Perform() || !dist.IsDone() || dist.NbSolution() < 1) {
    throw std::runtime_error("BRepExtrema_DistShapeShape found no solution");
  }
  OcDistance out{};
  out.distance = dist.Value();
  out.on_a = v3(dist.PointOnShape1(1).XYZ());
  out.on_b = v3(dist.PointOnShape2(1).XYZ());
  out.inner = dist.InnerSolution();
  out.kind_a = NO_SUPPORT;
  out.kind_b = NO_SUPPORT;
  if (!out.inner) {
    if (i1 != nullptr) {
      support_in(*i1, dist.SupportTypeShape1(1), dist.SupportOnShape1(1), out.kind_a,
                 out.index_a);
    }
    if (i2 != nullptr) {
      support_in(*i2, dist.SupportTypeShape2(1), dist.SupportOnShape2(1), out.kind_b,
                 out.index_b);
    }
    if (out.kind_a == 0) {
      dist.ParOnFaceS1(1, out.u_a, out.v_a);
    }
    if (out.kind_b == 0) {
      dist.ParOnFaceS2(1, out.u_b, out.v_b);
    }
  }
  return out;
}

// One face's OCCT-natural differential properties at (u, v).
OcSurfacePoint surface_point(const TopoDS_Face &face, double u, double v, double tol) {
  const BRepAdaptor_Surface surface(face, false);
  BRepLProp_SLProps props(surface, u, v, 2, tol);
  OcSurfacePoint out{};
  out.point = v3(props.Value().XYZ());
  out.d1u = v3(props.D1U().XYZ());
  out.d1v = v3(props.D1V().XYZ());
  out.normal_defined = props.IsNormalDefined();
  if (!out.normal_defined) {
    return out;
  }
  out.normal = v3(props.Normal().XYZ());
  out.curvature_defined = props.IsCurvatureDefined();
  if (!out.curvature_defined) {
    return out;
  }
  out.k_max = props.MaxCurvature();
  out.k_min = props.MinCurvature();
  out.umbilic = props.IsUmbilic();
  if (!out.umbilic) {
    gp_Dir dmax;
    gp_Dir dmin;
    props.CurvatureDirections(dmax, dmin);
    out.dir_max = v3(dmax.XYZ());
    out.dir_min = v3(dmin.XYZ());
  }
  return out;
}

} // namespace

OcMassProps mass_props(const OcShape &shape, const OcIndex &index, double eps) {
  OcMassProps out{};
  const TopoDS_Shape &s = shape.get();
  GProp_GProps surface;
  if (index.faces.Extent() > 0) {
    BRepGProp::SurfaceProperties(s, surface, eps);
    out.area = surface.Mass();
  }
  GProp_GProps props;
  if (index.solids.Extent() > 0) {
    // OnlyClosed: loose faces next to the solids of a compound enclose nothing.
    BRepGProp::VolumeProperties(s, props, eps, true);
    out.volume = props.Mass();
    out.basis = 0;
  } else if (index.faces.Extent() > 0) {
    props = surface;
    out.basis = 1;
  } else if (index.edges.Extent() > 0) {
    BRepGProp::LinearProperties(s, props);
    out.basis = 2;
  } else {
    out.basis = 3;
    return out;
  }
  out.measure = props.Mass();
  out.centroid = v3(props.CentreOfMass().XYZ());
  // About the centre of mass (GProp applies Huygens' theorem to its reference point).
  const gp_Mat m = props.MatrixOfInertia();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.inertia[static_cast<std::size_t>(3 * r + c)] = m.Value(r + 1, c + 1);
    }
  }
  return out;
}

std::uint8_t classify_point(const OcIndex &index, const V3 &p, double tol) {
  const int n = index.solids.Extent();
  if (n == 0) {
    throw std::invalid_argument("classify needs a shape with a solid");
  }
  bool on = false;
  const gp_Pnt point = pnt(p);
  for (int i = 1; i <= n; ++i) {
    BRepClass3d_SolidClassifier classifier(index.solids.FindKey(i), point, tol);
    const TopAbs_State state = classifier.State();
    if (state == TopAbs_IN) {
      return 0;
    }
    if (state == TopAbs_ON) {
      on = true;
    }
  }
  return on ? 2 : 1;
}

OcDistance distance_shapes(const OcShape &a, const OcIndex &ia, const OcShape &b,
                           const OcIndex &ib) {
  return run_distance(a.get(), &ia, b.get(), &ib);
}

OcDistance closest_point_on(const OcShape &shape, const OcIndex &index, const V3 &p) {
  if (index.faces.Extent() == 0 && index.edges.Extent() == 0 && index.vertices.Extent() == 0) {
    throw std::invalid_argument("closest_point needs a shape with a face, edge or vertex");
  }
  const TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(pnt(p)).Vertex();
  OcDistance whole = run_distance(shape.get(), &index, vertex, nullptr);
  if (!whole.inner) {
    return whole;
  }
  // Inside a solid BRepExtrema reports distance 0 with no support. The nearest boundary point
  // is the best of the faces taken one by one (they cannot be put into one compound without
  // writing to their TShapes, see the top of this file).
  OcDistance best{};
  bool any = false;
  for (int i = 1; i <= index.faces.Extent(); ++i) {
    OcDistance d = run_distance(index.faces.FindKey(i), &index, vertex, nullptr);
    if (!any || d.distance < best.distance) {
      best = d;
      any = true;
    }
  }
  if (!any) {
    return whole;
  }
  return best;
}

OcRayHit ray_cast_faces(const OcIndex &index, const V3 &origin, const V3 &dir, double max_t,
                        double tol) {
  OcRayHit out{};
  out.face = NO_FACE;
  const gp_Lin line(pnt(origin), gp_Dir(dir.x, dir.y, dir.z));
  double best = std::numeric_limits<double>::infinity();
  for (int f = 1; f <= index.faces.Extent(); ++f) {
    const TopoDS_Face &face = TopoDS::Face(index.faces.FindKey(f));
    IntCurvesFace_Intersector inter(face, tol);
    inter.Perform(line, 0.0, max_t);
    if (!inter.IsDone()) {
      continue;
    }
    for (int i = 1; i <= inter.NbPnt(); ++i) {
      const TopAbs_State state = inter.State(i);
      if (state != TopAbs_IN && state != TopAbs_ON) {
        continue;
      }
      const double w = inter.WParameter(i);
      // Strictly smaller: on a tie (an edge shared by two faces) the lower face index wins.
      if (w < 0.0 || w > max_t || !(w < best)) {
        continue;
      }
      best = w;
      out.hit = true;
      out.t = w;
      out.point = v3(inter.Pnt(i).XYZ());
      out.face = static_cast<std::uint32_t>(f - 1);
      out.u = inter.UParameter(i);
      out.v = inter.VParameter(i);
    }
  }
  return out;
}

OcSurfacePoint eval_face(const OcIndex &index, std::uint32_t f, double u, double v, double tol) {
  return surface_point(face_at(index, f), u, v, tol);
}

OcDistance project_on_face(const OcIndex &index, std::uint32_t f, const V3 &p, double tol) {
  const TopoDS_Face &face = face_at(index, f);
  const TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(pnt(p)).Vertex();
  OcDistance out = run_distance(face, &index, vertex, nullptr);
  if (out.kind_a != 0) {
    // The nearest point is on the face's boundary: find its (u, v) on the surface, inside the
    // face's parameter range for periodic surfaces.
    // BRep_Tool::Surface(face) has the face's (rigid) location applied, which keeps (u, v).
    const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(face);
    ShapeAnalysis_Surface analysis(surface);
    const gp_Pnt2d uv = analysis.ValueOfUV(pnt(out.on_a), tol);
    double umin = 0.0;
    double umax = 0.0;
    double vmin = 0.0;
    double vmax = 0.0;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    double u = uv.X();
    double v = uv.Y();
    if (surface->IsUPeriodic()) {
      const double period = surface->UPeriod();
      while (u < umin - tol) {
        u += period;
      }
      while (u > umax + tol) {
        u -= period;
      }
    }
    if (surface->IsVPeriodic()) {
      const double period = surface->VPeriod();
      while (v < vmin - tol) {
        v += period;
      }
      while (v > vmax + tol) {
        v -= period;
      }
    }
    out.u_a = u;
    out.v_a = v;
  }
  return out;
}

OcEdgeEnds edge_ends(const OcIndex &index, std::uint32_t e) {
  const TopoDS_Edge &edge = edge_at(index, e);
  OcEdgeEnds out{};
  if (BRep_Tool::Degenerated(edge)) {
    out.degenerated = true;
    return out;
  }
  // Parameter direction of the 3D curve (as edge_curve and the index's edge_vertices order).
  const BRepAdaptor_Curve curve(TopoDS::Edge(edge.Oriented(TopAbs_FORWARD)));
  gp_Pnt p;
  gp_Vec d;
  curve.D1(curve.FirstParameter(), p, d);
  out.p0 = v3(p.XYZ());
  out.t0 = v3(d.XYZ());
  curve.D1(curve.LastParameter(), p, d);
  out.p1 = v3(p.XYZ());
  out.t1 = v3(d.XYZ());
  return out;
}

} // namespace amg::occt
