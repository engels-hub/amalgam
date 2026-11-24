// Naming inputs (S02 §3, §6, BREP-05): sub-shape fingerprints for S01 `GeomFingerprint` and the
// split order, and index maps between a shape and a part of it (per-solid histories).
// Declarations and categories are in occt_shim.hxx. All category R.

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <gp_Vec2d.hxx>

namespace amg::occt {

namespace {

// amg-brep `geom::GeomClass`, in declaration order (amg-brep maps the codes explicitly).
enum Class : std::uint8_t {
  PLANE = 0,
  CYLINDER = 1,
  CONE = 2,
  SPHERE = 3,
  TORUS = 4,
  BSPLINE = 5,
  OTHER = 6,
  LINE = 7,
  CIRCLE = 8,
  ELLIPSE = 9,
  CURVE = 10,
  POINT = 11,
};

V3 v3(const gp_XYZ &p) { return V3{p.X(), p.Y(), p.Z()}; }

void set_axis(OcSubGeom &g, const gp_Dir &d) {
  g.has_axis = true;
  g.axis = v3(d.XYZ());
}

// `BRepBndLib::Add` from the geometry (no triangulation), without the tolerance gap it adds:
// exact for lines, circles and planar faces (bounded by their edges) and for analytic surface
// patches, the pole hull for B-splines. `AddOptimal` is tighter on B-splines but optimises over
// every face: 110 µs per hole wall and 39 ms for a cap with 1024 holes, five times the whole
// `sub_geom_all` budget on a 1000-face part (BREP-05).
void set_bbox(OcSubGeom &g, const TopoDS_Shape &s) {
  Bnd_Box box;
  BRepBndLib::Add(s, box, false);
  box.SetGap(0.0);
  if (box.IsVoid()) {
    g.bmin = g.centroid;
    g.bmax = g.centroid;
    return;
  }
  g.bmin = v3(box.CornerMin().XYZ());
  g.bmax = v3(box.CornerMax().XYZ());
}

constexpr double QUARTER_TURN = 1.5707963267948966;

// Gauss-Legendre, 8 points on [-1, 1].
constexpr double GAUSS_X[8] = {-0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
                               -0.1834346424956498, 0.1834346424956498,  0.5255324099163290,
                               0.7966664774136267,  0.9602898564975363};
constexpr double GAUSS_W[8] = {0.1012285362903763, 0.2223810344533745, 0.3137066458778873,
                               0.3626837833783620, 0.3626837833783620, 0.3137066458778873,
                               0.2223810344533745, 0.1012285362903763};

// Area and first moment `∬ (P − c0) dA` of a face.
struct Moments {
  double area = 0.0;
  gp_XYZ moment{0.0, 0.0, 0.0};
};

// Area and centroid of a face on an analytic surface by Green's theorem in (u, v):
// `∬ f J du dv = −∮ F du` with `F(u, v) = ∫_{v0}^{v} f(u, s) J(u, s) ds`, the loop running
// with the material on its left (a FORWARD face). Gauss-Legendre along each pcurve (one
// 8-point rule per line, per quarter of a circle or ellipse, per span of anything else) and,
// off planes, along v. Exact for planes up to the pcurve rule, and for the other analytic
// surfaces to round-off on the usual pcurves; `BRepGProp::SurfaceProperties` evaluates the same
// integral, at about ten times the cost per face (it rebuilds an adaptor with UV bounds per
// face and integrates every edge, seams included). Returns false where it does not apply.
bool analytic_moments(const TopoDS_Face &face, const BRepAdaptor_Surface &surface, gp_Pnt &c0,
                      Moments &out) {
  const GeomAbs_SurfaceType type = surface.GetType();
  if (type != GeomAbs_Plane && type != GeomAbs_Cylinder && type != GeomAbs_Cone &&
      type != GeomAbs_Sphere && type != GeomAbs_Torus) {
    return false;
  }
  const TopoDS_Face forward = TopoDS::Face(face.Oriented(TopAbs_FORWARD));
  TopExp_Explorer edges(forward, TopAbs_EDGE);
  if (!edges.More()) {
    return false; // natural bounds: no loop to integrate along
  }
  bool have_origin = false;
  double v0 = 0.0;
  const bool plane = type == GeomAbs_Plane;
  gp_XYZ origin;
  gp_XYZ xdir;
  gp_XYZ ydir;
  if (plane) {
    const gp_Ax3 pos = surface.Plane().Position();
    origin = pos.Location().XYZ();
    xdir = pos.XDirection().XYZ();
    ydir = pos.YDirection().XYZ();
  }
  // F_A and F_P at (u, v): the inner integrals from v0 to v.
  auto inner = [&](double u, double v, double &fa, gp_XYZ &fp) {
    if (plane) {
      // P = O + u X + s Y, J = 1.
      const double dv = v - v0;
      fa = dv;
      fp = (origin + u * xdir - c0.XYZ()) * dv + ydir * (0.5 * (v * v - v0 * v0));
      return;
    }
    fa = 0.0;
    fp.SetCoord(0.0, 0.0, 0.0);
    const double half = 0.5 * (v - v0);
    const double mid = 0.5 * (v + v0);
    for (int k = 0; k < 8; ++k) {
      const double s = mid + half * GAUSS_X[k];
      gp_Pnt p;
      gp_Vec du;
      gp_Vec dv;
      surface.D1(u, s, p, du, dv);
      const double j = du.Crossed(dv).Magnitude() * GAUSS_W[k] * half;
      fa += j;
      fp += (p.XYZ() - c0.XYZ()) * j;
    }
  };
  for (; edges.More(); edges.Next()) {
    const TopoDS_Edge &edge = TopoDS::Edge(edges.Current());
    double first = 0.0;
    double last = 0.0;
    const Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, forward, first, last);
    if (pcurve.IsNull()) {
      return false;
    }
    const Geom2dAdaptor_Curve curve(pcurve, first, last);
    const double sign = edge.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
    if (!have_origin) {
      const gp_Pnt2d start = curve.Value(first);
      v0 = start.Y();
      c0 = surface.Value(start.X(), start.Y());
      have_origin = true;
    }
    int pieces = 1;
    switch (curve.GetType()) {
    case GeomAbs_Line: {
      // An iso-u line (a seam, a meridian) has du = 0: nothing to add.
      if (std::abs(curve.Line().Direction().X()) < 1e-15) {
        continue;
      }
      break;
    }
    case GeomAbs_Circle:
    case GeomAbs_Ellipse:
      pieces = std::max(1, static_cast<int>(std::ceil((last - first) / QUARTER_TURN - 1e-9)));
      break;
    default:
      pieces = std::max(8, 8 * curve.NbIntervals(GeomAbs_C2));
      break;
    }
    const double step = (last - first) / pieces;
    for (int piece = 0; piece < pieces; ++piece) {
      const double a = first + piece * step;
      const double half = 0.5 * step;
      for (int k = 0; k < 8; ++k) {
        const double t = a + half * (1.0 + GAUSS_X[k]);
        gp_Pnt2d uv;
        gp_Vec2d d;
        curve.D1(t, uv, d);
        const double w = -sign * GAUSS_W[k] * half * d.X();
        if (w == 0.0) {
          continue;
        }
        double fa = 0.0;
        gp_XYZ fp;
        inner(uv.X(), uv.Y(), fa, fp);
        out.area += w * fa;
        out.moment += fp * w;
      }
    }
  }
  return have_origin;
}

OcSubGeom face_geom(const TopoDS_Face &face) {
  OcSubGeom g{};
  g.geom_class = OTHER;
  const BRepAdaptor_Surface surface(face, false);
  switch (surface.GetType()) {
  case GeomAbs_Plane: {
    g.geom_class = PLANE;
    // The surface normal is D1u x D1v = XDirection x YDirection (Direction() is its opposite on
    // a left-handed frame); a reversed face points the other way, out of the material.
    const gp_Ax3 frame = surface.Plane().Position();
    gp_Dir normal = frame.XDirection().Crossed(frame.YDirection());
    if (face.Orientation() == TopAbs_REVERSED) {
      normal.Reverse();
    }
    set_axis(g, normal);
    break;
  }
  case GeomAbs_Cylinder:
    g.geom_class = CYLINDER;
    set_axis(g, surface.Cylinder().Axis().Direction());
    break;
  case GeomAbs_Cone:
    g.geom_class = CONE;
    set_axis(g, surface.Cone().Axis().Direction());
    break;
  case GeomAbs_Sphere:
    g.geom_class = SPHERE;
    break;
  case GeomAbs_Torus:
    g.geom_class = TORUS;
    set_axis(g, surface.Torus().Axis().Direction());
    break;
  case GeomAbs_BSplineSurface:
  case GeomAbs_BezierSurface:
    g.geom_class = BSPLINE;
    break;
  default:
    break;
  }
  Moments m;
  gp_Pnt c0;
  if (analytic_moments(face, surface, c0, m) && m.area != 0.0) {
    g.measure = std::abs(m.area);
    g.centroid = v3(c0.XYZ() + m.moment / m.area);
  } else {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    g.measure = props.Mass();
    g.centroid = v3(props.CentreOfMass().XYZ());
  }
  set_bbox(g, face);
  return g;
}

OcSubGeom edge_geom(const TopoDS_Edge &edge) {
  OcSubGeom g{};
  g.geom_class = CURVE;
  if (BRep_Tool::Degenerated(edge)) {
    // A pole: no length; its vertex is the centroid.
    TopoDS_Vertex v1;
    TopoDS_Vertex v2;
    TopExp::Vertices(edge, v1, v2);
    if (!v1.IsNull()) {
      g.centroid = v3(BRep_Tool::Pnt(v1).XYZ());
    }
    g.bmin = g.centroid;
    g.bmax = g.centroid;
    return g;
  }
  const BRepAdaptor_Curve curve(edge);
  switch (curve.GetType()) {
  case GeomAbs_Line:
    g.geom_class = LINE;
    set_axis(g, curve.Line().Direction());
    break;
  case GeomAbs_Circle:
    g.geom_class = CIRCLE;
    set_axis(g, curve.Circle().Axis().Direction());
    break;
  case GeomAbs_Ellipse:
    g.geom_class = ELLIPSE;
    set_axis(g, curve.Ellipse().Axis().Direction());
    break;
  default:
    break;
  }
  GProp_GProps props;
  BRepGProp::LinearProperties(edge, props);
  g.measure = props.Mass();
  g.centroid = v3(props.CentreOfMass().XYZ());
  set_bbox(g, edge);
  return g;
}

OcSubGeom vertex_geom(const TopoDS_Vertex &vertex) {
  OcSubGeom g{};
  g.geom_class = POINT;
  g.centroid = v3(BRep_Tool::Pnt(vertex).XYZ());
  g.bmin = g.centroid;
  g.bmax = g.centroid;
  return g;
}

const ShapeMap &map_of(const OcIndex &index, std::uint8_t kind) {
  switch (kind) {
  case 0:
    return index.faces;
  case 1:
    return index.edges;
  case 2:
    return index.vertices;
  default:
    throw std::invalid_argument("sub_geoms: unknown sub-shape kind");
  }
}

rust::Vec<std::uint32_t> positions(const ShapeMap &whole, const ShapeMap &part) {
  rust::Vec<std::uint32_t> out;
  out.reserve(static_cast<std::size_t>(part.Extent()));
  for (int i = 1; i <= part.Extent(); ++i) {
    const int j = whole.FindIndex(part(i));
    out.push_back(j == 0 ? std::numeric_limits<std::uint32_t>::max()
                         : static_cast<std::uint32_t>(j - 1));
  }
  return out;
}

} // namespace

rust::Vec<OcSubGeom> sub_geoms(const OcIndex &index, std::uint8_t kind,
                               rust::Slice<const std::uint32_t> indices) {
  const ShapeMap &map = map_of(index, kind);
  rust::Vec<OcSubGeom> out;
  out.reserve(indices.size());
  for (const std::uint32_t i : indices) {
    if (i >= static_cast<std::uint32_t>(map.Extent())) {
      throw std::out_of_range("sub_geoms: index out of range");
    }
    const TopoDS_Shape &s = map(static_cast<int>(i) + 1);
    switch (kind) {
    case 0:
      out.push_back(face_geom(TopoDS::Face(s)));
      break;
    case 1:
      out.push_back(edge_geom(TopoDS::Edge(s)));
      break;
    default:
      out.push_back(vertex_geom(TopoDS::Vertex(s)));
      break;
    }
  }
  return out;
}

OcSubMap sub_index_map(const OcIndex &whole, const OcIndex &part) {
  OcSubMap out;
  out.faces = positions(whole.faces, part.faces);
  out.edges = positions(whole.edges, part.edges);
  out.vertices = positions(whole.vertices, part.vertices);
  return out;
}

} // namespace amg::occt
