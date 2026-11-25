#include "amg-occt-sys/cpp/occt_fixtures.hxx"

#include <stdexcept>
#include <string>
#include <utility>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Plane.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Vec2d.hxx>

namespace amg::occt {

namespace {

constexpr double TWO_PI = 6.283185307179586476925286766559;

// The plate fixture's hole grid: 32 × 32 holes give 1030 faces (S02's 1000-face budget part).
constexpr int PLATE_HOLES_PER_SIDE = 32;

TopoDS_Shape box() { return BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape(); }

TopoDS_Shape cylinder() {
  return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(3.0, 1.0, 0.0), gp::DZ()), 5.0, 20.0).Shape();
}

TopoDS_Shape cone() {
  return BRepPrimAPI_MakeCone(gp_Ax2(gp::Origin(), gp::DZ()), 5.0, 2.0, 10.0).Shape();
}

TopoDS_Shape sphere() { return BRepPrimAPI_MakeSphere(gp::Origin(), 7.0).Shape(); }

TopoDS_Shape torus() {
  return BRepPrimAPI_MakeTorus(gp_Ax2(gp::Origin(), gp::DZ()), 20.0, 5.0).Shape();
}

TopoDS_Shape filleted_box() {
  const TopoDS_Shape base = box();
  BRepFilletAPI_MakeFillet maker(base);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges;
  TopExp::MapShapes(base, TopAbs_EDGE, edges);
  for (int i = 1; i <= edges.Extent(); ++i) {
    maker.Add(1.0, TopoDS::Edge(edges(i)));
  }
  return maker.Shape();
}

TopoDS_Shape plate_with_holes() {
  const double spacing = 10.0;
  const double side = spacing * (PLATE_HOLES_PER_SIDE + 1);
  const TopoDS_Shape plate = BRepPrimAPI_MakeBox(side, side, 5.0).Shape();
  BRep_Builder builder;
  TopoDS_Compound tools;
  builder.MakeCompound(tools);
  for (int i = 1; i <= PLATE_HOLES_PER_SIDE; ++i) {
    for (int j = 1; j <= PLATE_HOLES_PER_SIDE; ++j) {
      const gp_Ax2 axes(gp_Pnt(i * spacing, j * spacing, -1.0), gp::DZ());
      builder.Add(tools, BRepPrimAPI_MakeCylinder(axes, 3.0, 7.0).Shape());
    }
  }
  BRepAlgoAPI_Cut cut(plate, tools);
  if (!cut.IsDone()) {
    throw std::runtime_error("plate_with_holes: boolean cut failed");
  }
  return cut.Shape();
}

// One box twice: the same TShapes at two locations must index as distinct sub-shapes.
TopoDS_Shape located_compound() {
  const TopoDS_Shape base = box();
  gp_Trsf move;
  move.SetTranslation(gp_Vec(100.0, 0.0, 0.0));
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  builder.Add(compound, base);
  builder.Add(compound, base.Moved(TopLoc_Location(move)));
  return compound;
}

TopoDS_Shape bspline_patch() {
  NCollection_Array2<gp_Pnt> poles(1, 4, 1, 4);
  for (int i = 1; i <= 4; ++i) {
    for (int j = 1; j <= 4; ++j) {
      const double z = (i == 2 || i == 3) && (j == 2 || j == 3) ? 5.0 : 0.0;
      poles(i, j) = gp_Pnt((i - 1) * 10.0, (j - 1) * 10.0, z);
    }
  }
  NCollection_Array1<double> knots(1, 2);
  knots(1) = 0.0;
  knots(2) = 1.0;
  NCollection_Array1<int> mults(1, 2);
  mults(1) = 4;
  mults(2) = 4;
  const Handle(Geom_BSplineSurface) surface =
      new Geom_BSplineSurface(poles, knots, knots, mults, mults, 3, 3);
  return BRepBuilderAPI_MakeFace(surface, 1e-6).Face();
}

// A Bezier profile revolved a full turn: a surface of revolution, which reads back as `Other`.
TopoDS_Shape revolved_profile() {
  NCollection_Array1<gp_Pnt> poles(1, 4);
  poles(1) = gp_Pnt(5.0, 0.0, 0.0);
  poles(2) = gp_Pnt(8.0, 0.0, 3.0);
  poles(3) = gp_Pnt(4.0, 0.0, 7.0);
  poles(4) = gp_Pnt(6.0, 0.0, 10.0);
  const Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
  const TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(curve).Edge();
  return BRepPrimAPI_MakeRevol(edge, gp_Ax1(gp::Origin(), gp::DZ()), TWO_PI).Shape();
}

// Reflection gives the cylinder's surface a left-handed frame.
TopoDS_Shape cylinder_mirrored() {
  gp_Trsf mirror;
  mirror.SetMirror(gp_Ax2(gp::Origin(), gp::DX()));
  return BRepBuilderAPI_Transform(cylinder(), mirror, true).Shape();
}

// -- defective shapes for the BREP-10 healing and check corpus (testdata/brep/defects/) --------
//
// Each starts from a valid shape and breaks one thing, so the defect is known by construction.
// The shapes are built fresh, so their sub-shapes are changed in place.

using Map = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;

Map map_of(const TopoDS_Shape &s, TopAbs_ShapeEnum type) {
  Map m;
  TopExp::MapShapes(s, type, m);
  return m;
}

// Box vertex 1 moved 5e-5 mm off its three edges, its tolerance left at 1e-7.
TopoDS_Shape defect_vertex_off_edges() {
  const TopoDS_Shape s = box();
  const TopoDS_Vertex v = TopoDS::Vertex(map_of(s, TopAbs_VERTEX)(1));
  BRep_Builder().UpdateVertex(v, BRep_Tool::Pnt(v).Translated(gp_Vec(3e-5, 3e-5, 3e-5)), 1e-7);
  return s;
}

// Box edge 1's 3D line shifted 2e-5 mm off its faces (pcurves and vertices unchanged).
TopoDS_Shape defect_edge_off_faces() {
  const TopoDS_Shape s = box();
  const TopoDS_Edge e = TopoDS::Edge(map_of(s, TopAbs_EDGE)(1));
  TopLoc_Location loc;
  double first = 0.0;
  double last = 0.0;
  const Handle(Geom_Curve) curve = BRep_Tool::Curve(e, loc, first, last);
  const Handle(Geom_Curve) moved =
      Handle(Geom_Curve)::DownCast(curve->Translated(gp_Vec(2e-5, -2e-5, 2e-5)));
  BRep_Builder().UpdateEdge(e, moved, loc, 1e-7);
  return s;
}

// The lateral face of a cylinder, and its first circle (non-seam) edge.
std::pair<TopoDS_Face, TopoDS_Edge> cylinder_lateral(const TopoDS_Shape &s) {
  for (TopExp_Explorer f(s, TopAbs_FACE); f.More(); f.Next()) {
    const TopoDS_Face face = TopoDS::Face(f.Current());
    if (BRepAdaptor_Surface(face).GetType() != GeomAbs_Cylinder) {
      continue;
    }
    for (TopExp_Explorer e(face, TopAbs_EDGE); e.More(); e.Next()) {
      const TopoDS_Edge edge = TopoDS::Edge(e.Current());
      if (!BRep_Tool::IsClosed(edge, face)) {
        return {face, edge};
      }
    }
  }
  throw std::runtime_error("cylinder_lateral: no circle edge");
}

// The pcurve of a cylinder's circle edge on its lateral face moved 4e-5 mm along the axis.
TopoDS_Shape defect_pcurve_off_edge() {
  const TopoDS_Shape s = cylinder();
  const auto [face, edge] = cylinder_lateral(s);
  double first = 0.0;
  double last = 0.0;
  const Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);
  const Handle(Geom2d_Curve) moved =
      Handle(Geom2d_Curve)::DownCast(pcurve->Translated(gp_Vec2d(0.0, 4e-5)));
  BRep_Builder().UpdateEdge(edge, moved, face, 1e-7);
  return s;
}

// A cylinder solid whose shell holds the lateral face reversed.
TopoDS_Shape defect_reversed_lateral() {
  const TopoDS_Shape s = cylinder();
  const TopoDS_Face lateral = cylinder_lateral(s).first;
  BRep_Builder b;
  TopoDS_Shell shell;
  b.MakeShell(shell);
  for (TopExp_Explorer f(s, TopAbs_FACE); f.More(); f.Next()) {
    b.Add(shell, f.Current().IsSame(lateral) ? f.Current().Reversed() : f.Current());
  }
  TopoDS_Solid solid;
  b.MakeSolid(solid);
  b.Add(solid, shell);
  return solid;
}

// The first circle edge of a cylinder's lateral face loses its pcurve on that face.
TopoDS_Shape defect_missing_pcurve() {
  const TopoDS_Shape s = cylinder();
  const auto [face, edge] = cylinder_lateral(s);
  BRep_Builder().UpdateEdge(edge, Handle(Geom2d_Curve)(), face, 1e-7);
  return s;
}

// A box solid whose shell holds face 1 reversed.
TopoDS_Shape defect_reversed_face() {
  const TopoDS_Shape s = box();
  BRep_Builder b;
  TopoDS_Shell shell;
  b.MakeShell(shell);
  bool first = true;
  for (TopExp_Explorer f(s, TopAbs_FACE); f.More(); f.Next()) {
    b.Add(shell, first ? f.Current().Reversed() : f.Current());
    first = false;
  }
  TopoDS_Solid solid;
  b.MakeSolid(solid);
  b.Add(solid, shell);
  return solid;
}

// Box edge 1 with its SameParameter flag cleared.
TopoDS_Shape defect_same_parameter_off() {
  const TopoDS_Shape s = box();
  BRep_Builder().SameParameter(TopoDS::Edge(map_of(s, TopAbs_EDGE)(1)), false);
  return s;
}

// Box edge 1 with its SameRange flag cleared.
TopoDS_Shape defect_same_range_off() {
  const TopoDS_Shape s = box();
  BRep_Builder().SameRange(TopoDS::Edge(map_of(s, TopAbs_EDGE)(1)), false);
  return s;
}

// A planar face on the 4 corners `p`, its edges built in `order`, each with its own end vertices
// (`gap` mm apart from the next edge's start, along x) unless `gap` is 0.
TopoDS_Shape square_face(const gp_Pnt (&p)[4], const int (&order)[4], double gap, bool reversed) {
  BRep_Builder b;
  TopoDS_Edge edges[4];
  if (gap == 0.0) {
    TopoDS_Vertex v[4];
    for (int i = 0; i < 4; ++i) {
      b.MakeVertex(v[i], p[i], 1e-7);
    }
    for (int i = 0; i < 4; ++i) {
      edges[i] = BRepBuilderAPI_MakeEdge(v[i], v[(i + 1) % 4]).Edge();
    }
  } else {
    for (int i = 0; i < 4; ++i) {
      const gp_Pnt a = i == 0 ? p[0] : p[i].Translated(gp_Vec(gap, 0.0, 0.0));
      edges[i] = BRepBuilderAPI_MakeEdge(a, p[(i + 1) % 4]).Edge();
    }
  }
  TopoDS_Wire wire;
  b.MakeWire(wire);
  for (int i : order) {
    b.Add(wire, edges[i]);
  }
  TopoDS_Face face;
  b.MakeFace(face, new Geom_Plane(gp_Pln(p[0], gp::DZ())), 1e-7);
  b.Add(face, reversed ? wire.Reversed() : wire);
  return face;
}

const gp_Pnt SQUARE[4] = {gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0),
                              gp_Pnt(0, 10, 0)};
constexpr int IN_ORDER[4] = {0, 1, 2, 3};

// A square face whose wire has 3e-5 mm gaps at three corners.
TopoDS_Shape defect_wire_gaps() { return square_face(SQUARE, IN_ORDER, 3e-5, false); }

// A square face whose closed wire lists its edges out of order.
TopoDS_Shape defect_wire_order() {
  constexpr int order[4] = {0, 2, 1, 3};
  return square_face(SQUARE, order, 0.0, false);
}

// A square face whose only wire runs clockwise (a hole without an outer boundary).
TopoDS_Shape defect_reversed_wire() { return square_face(SQUARE, IN_ORDER, 0.0, true); }

// A box solid whose shell is reversed: it bounds the infinite outside.
TopoDS_Shape defect_inside_out() {
  const TopoDS_Shape s = box();
  BRep_Builder b;
  TopoDS_Solid solid;
  b.MakeSolid(solid);
  b.Add(solid, map_of(s, TopAbs_SHELL)(1).Reversed());
  return solid;
}

// Box edge 1 with tolerance 1e-3 mm while its vertices keep 1e-7.
TopoDS_Shape defect_vertex_tolerance() {
  const TopoDS_Shape s = box();
  const TopoDS_Edge e = TopoDS::Edge(map_of(s, TopAbs_EDGE)(1));
  BRep_Builder().UpdateEdge(e, 1e-3);
  return s;
}

// A square face with a 3e-7 mm edge at its first corner (vertex tolerances 1e-6).
TopoDS_Shape defect_small_edge() {
  BRep_Builder b;
  const gp_Pnt p[5] = {gp_Pnt(0, 0, 0), gp_Pnt(3e-7, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0),
                       gp_Pnt(0, 10, 0)};
  TopoDS_Vertex v[5];
  for (int i = 0; i < 5; ++i) {
    b.MakeVertex(v[i], p[i], 1e-6);
  }
  TopoDS_Wire wire;
  b.MakeWire(wire);
  for (int i = 0; i < 5; ++i) {
    b.Add(wire, BRepBuilderAPI_MakeEdge(v[i], v[(i + 1) % 5]).Edge());
  }
  TopoDS_Face face;
  b.MakeFace(face, new Geom_Plane(gp_Pln(p[0], gp::DZ())), 1e-7);
  b.Add(face, wire);
  return face;
}

// A bow-tie: a face whose wire crosses itself.
TopoDS_Shape defect_bow_tie() {
  const gp_Pnt p[4] = {gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 0), gp_Pnt(10, 0, 0),
                           gp_Pnt(0, 10, 0)};
  return square_face(p, IN_ORDER, 0.0, false);
}

// A box solid with its top face missing.
TopoDS_Shape defect_open_solid() {
  const TopoDS_Shape s = box();
  const Map faces = map_of(s, TopAbs_FACE);
  BRep_Builder b;
  TopoDS_Shell shell;
  b.MakeShell(shell);
  for (int i = 1; i < faces.Extent(); ++i) {
    b.Add(shell, faces(i));
  }
  TopoDS_Solid solid;
  b.MakeSolid(solid);
  b.Add(solid, shell);
  return solid;
}

// Two boxes overlapping by half, in one compound (valid for BRepCheck; they interfere).
TopoDS_Shape defect_overlapping_boxes() {
  BRep_Builder b;
  TopoDS_Compound c;
  b.MakeCompound(c);
  b.Add(c, BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape());
  b.Add(c, BRepPrimAPI_MakeBox(gp_Pnt(5.0, 5.0, 5.0), 10.0, 10.0, 10.0).Shape());
  return c;
}

TopoDS_Shape defect(const std::string &n) {
  if (n == "vertex_off_edges") {
    return defect_vertex_off_edges();
  }
  if (n == "edge_off_faces") {
    return defect_edge_off_faces();
  }
  if (n == "missing_pcurve") {
    return defect_missing_pcurve();
  }
  if (n == "pcurve_off_edge") {
    return defect_pcurve_off_edge();
  }
  if (n == "reversed_lateral") {
    return defect_reversed_lateral();
  }
  if (n == "reversed_face") {
    return defect_reversed_face();
  }
  if (n == "same_parameter_off") {
    return defect_same_parameter_off();
  }
  if (n == "same_range_off") {
    return defect_same_range_off();
  }
  if (n == "wire_gaps") {
    return defect_wire_gaps();
  }
  if (n == "wire_order") {
    return defect_wire_order();
  }
  if (n == "reversed_wire") {
    return defect_reversed_wire();
  }
  if (n == "inside_out") {
    return defect_inside_out();
  }
  if (n == "vertex_tolerance") {
    return defect_vertex_tolerance();
  }
  if (n == "small_edge") {
    return defect_small_edge();
  }
  if (n == "bow_tie") {
    return defect_bow_tie();
  }
  if (n == "open_solid") {
    return defect_open_solid();
  }
  if (n == "overlapping_boxes") {
    return defect_overlapping_boxes();
  }
  throw std::invalid_argument("unknown defect fixture: " + n);
}

} // namespace

std::unique_ptr<OcShape> fixture_shape(rust::Str name) {
  const std::string n(name);
  TopoDS_Shape shape;
  if (n == "box") {
    shape = box();
  } else if (n == "cylinder") {
    shape = cylinder();
  } else if (n == "cone") {
    shape = cone();
  } else if (n == "sphere") {
    shape = sphere();
  } else if (n == "torus") {
    shape = torus();
  } else if (n == "filleted_box") {
    shape = filleted_box();
  } else if (n == "plate_with_holes") {
    shape = plate_with_holes();
  } else if (n == "located_compound") {
    shape = located_compound();
  } else if (n == "bspline_patch") {
    shape = bspline_patch();
  } else if (n == "revolved_profile") {
    shape = revolved_profile();
  } else if (n == "cylinder_mirrored") {
    shape = cylinder_mirrored();
  } else if (n.rfind("defect_", 0) == 0) {
    shape = defect(n.substr(7));
  } else {
    throw std::invalid_argument("unknown fixture: " + n);
  }
  return std::make_unique<OcShape>(std::move(shape));
}

} // namespace amg::occt
