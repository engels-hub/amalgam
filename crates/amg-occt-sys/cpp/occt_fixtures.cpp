#include "amg-occt-sys/cpp/occt_fixtures.hxx"

#include <stdexcept>
#include <string>
#include <utility>

#include <BRepAlgoAPI_Cut.hxx>
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
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

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
  } else {
    throw std::invalid_argument("unknown fixture: " + n);
  }
  return std::make_unique<OcShape>(std::move(shape));
}

} // namespace amg::occt
