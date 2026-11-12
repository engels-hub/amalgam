// Tessellation shim (S02 §8, BREP-08). Declarations and categories are in occt_shim.hxx.
//
// tessellate_shape is category C: BRepMesh_IncrementalMesh stores Poly_Triangulation on faces
// and Poly_PolygonOnTriangulation on edges, so it meshes a private BRepBuilderAPI_Copy and never
// touches the caller's TShapes. The copy keeps the input's structure, so TopExp::MapShapes over
// it yields the same face/edge/vertex order as the input's OcIndex.

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <NCollection_List.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace amg::occt {

namespace {

constexpr std::uint32_t UNSET = 0xFFFFFFFFu;

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// Same bridge as SinkProgress in occt_shim.cpp (kept file-local there).
class TessProgress : public Message_ProgressIndicator {
public:
  explicit TessProgress(const ProgressSink &sink) : sink_(sink) {}

  bool UserBreak() override { return progress_user_break(sink_); }

  void Show(const Message_ProgressScope &, const bool) override {
    progress_show(sink_, GetPosition());
  }

private:
  const ProgressSink &sink_;
};

void push_point(rust::Vec<double> &out, const gp_Pnt &p) {
  out.push_back(p.X());
  out.push_back(p.Y());
  out.push_back(p.Z());
}

std::uint32_t to_u32(std::size_t n) {
  if (n >= static_cast<std::size_t>(UNSET)) {
    throw std::length_error("tessellation exceeds 2^32 - 1 elements");
  }
  return static_cast<std::uint32_t>(n);
}

std::uint32_t index_in(const ShapeMap &map, const TopoDS_Shape &shape, const char *what) {
  const int found = map.FindIndex(shape);
  if (found == 0) {
    throw std::logic_error(std::string(what) + " is not in the copy's index");
  }
  return static_cast<std::uint32_t>(found - 1);
}

// One edge's discretisation, as first seen on a face (or from its Polygon3D for free edges).
struct EdgeSamples {
  bool seen = false;
  std::uint32_t nodes = 0;          // polygon node count, ends included
  std::uint32_t interior_base = 0;  // canonical id of interior node 1
  std::vector<gp_Pnt> points;       // global coordinates, parameter order
};

// Polygon node indices (1-based, into the triangulation) in increasing edge parameter.
std::vector<int> polygon_nodes(const Poly_PolygonOnTriangulation &polygon) {
  const NCollection_Array1<int> &nodes = polygon.Nodes();
  std::vector<int> out(nodes.begin(), nodes.end());
  if (polygon.HasParameters() && out.size() >= 2 &&
      polygon.Parameter(1) > polygon.Parameter(polygon.NbNodes())) {
    std::reverse(out.begin(), out.end());
  }
  return out;
}

} // namespace

OcTessMesh tessellate_shape(const OcShape &shape, const OcTessParams &params,
                            const ProgressSink &sink) {
  if (!(params.linear > 0.0) || !std::isfinite(params.linear) || !(params.angular > 0.0) ||
      !std::isfinite(params.angular)) {
    throw std::invalid_argument("tessellation deflections must be positive and finite");
  }
  OcTessMesh out{};

  // -- private topology copy (category C) --
  const Clock::time_point copy_start = Clock::now();
  // Geometry is shared (copyGeom = false): BRepMesh writes only into topology (faces' and
  // edges' representation lists), never into Geom_* objects.
  BRepBuilderAPI_Copy copier(shape.get(), false, false);
  const TopoDS_Shape &copy = copier.Shape();
  ShapeMap faces;
  ShapeMap edges;
  ShapeMap vertices;
  TopExp::MapShapes(copy, TopAbs_FACE, faces);
  TopExp::MapShapes(copy, TopAbs_EDGE, edges);
  TopExp::MapShapes(copy, TopAbs_VERTEX, vertices);
  out.copy_seconds = seconds_since(copy_start);

  // -- mesh --
  const Clock::time_point mesh_start = Clock::now();
  IMeshTools_Parameters mesh_params;
  mesh_params.Deflection = params.linear;
  mesh_params.Angle = params.angular;
  mesh_params.Relative = false;
  mesh_params.InParallel = params.in_parallel;
  if (params.min_size > 0.0) {
    mesh_params.MinSize = params.min_size;
  }
  Handle(TessProgress) indicator = new TessProgress(sink);
  {
    Message_ProgressScope scope(indicator->Start(), "amg tessellate", 1);
    BRepMesh_IncrementalMesh mesher(copy, mesh_params, scope.Next());
    if (scope.UserBreak() || progress_user_break(sink)) {
      out.cancelled = true;
      return out;
    }
  }
  out.mesh_seconds = seconds_since(mesh_start);

  // -- copy out --
  const Clock::time_point extract_start = Clock::now();
  const auto n_faces = static_cast<std::size_t>(faces.Extent());
  const auto n_edges = static_cast<std::size_t>(edges.Extent());
  std::vector<EdgeSamples> samples(n_edges);
  std::uint32_t next_id = to_u32(static_cast<std::size_t>(vertices.Extent()));

  std::size_t total_nodes = 0;
  std::size_t total_tris = 0;
  for (int f = 1; f <= faces.Extent(); ++f) {
    TopLoc_Location loc;
    const Handle(Poly_Triangulation) &tri = BRep_Tool::Triangulation(TopoDS::Face(faces(f)), loc);
    if (!tri.IsNull()) {
      total_nodes += static_cast<std::size_t>(tri->NbNodes());
      total_tris += static_cast<std::size_t>(tri->NbTriangles());
    }
  }
  out.positions.reserve(total_nodes * 3);
  out.normals.reserve(total_nodes * 3);
  out.weld.reserve(total_nodes);
  out.tris.reserve(total_tris * 3);
  out.face_tri_starts.reserve(n_faces + 1);

  std::vector<std::uint32_t> canon;
  std::vector<gp_Vec> fallback;
  for (int f = 1; f <= faces.Extent(); ++f) {
    out.face_tri_starts.push_back(to_u32(out.tris.size() / 3));
    const TopoDS_Face &face = TopoDS::Face(faces(f));
    TopLoc_Location loc;
    const Handle(Poly_Triangulation) &tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull() || tri->NbNodes() == 0) {
      continue;
    }
    const gp_Trsf &trsf = loc.Transformation();
    const bool moved = !loc.IsIdentity();
    const bool reversed = face.Orientation() == TopAbs_REVERSED;
    const int n_nodes = tri->NbNodes();
    const auto base = to_u32(out.weld.size());
    const auto node_point = [&](int node) {
      gp_Pnt p = tri->Node(node);
      if (moved) {
        p.Transform(trsf);
      }
      return p;
    };

    // Canonical ids: vertices, then each edge's interior nodes, then face interior nodes.
    canon.assign(static_cast<std::size_t>(n_nodes) + 1, UNSET);
    for (TopExp_Explorer it(face, TopAbs_EDGE); it.More(); it.Next()) {
      const TopoDS_Edge &edge = TopoDS::Edge(it.Current());
      const Handle(Poly_PolygonOnTriangulation) &polygon =
          BRep_Tool::PolygonOnTriangulation(edge, tri, loc);
      if (polygon.IsNull() || polygon->NbNodes() == 0) {
        continue;
      }
      const std::vector<int> nodes = polygon_nodes(*polygon);
      TopoDS_Vertex first;
      TopoDS_Vertex last;
      TopExp::Vertices(edge, first, last);
      const std::uint32_t v_first =
          first.IsNull() ? UNSET : index_in(vertices, first, "edge vertex");
      const std::uint32_t v_last =
          last.IsNull() ? UNSET : index_in(vertices, last, "edge vertex");
      if (BRep_Tool::Degenerated(edge)) {
        // Every node of a degenerated edge is the one pole point.
        for (const int node : nodes) {
          if (v_first != UNSET) {
            canon[static_cast<std::size_t>(node)] = v_first;
          }
        }
        continue;
      }
      const std::uint32_t e = index_in(edges, edge, "face edge");
      EdgeSamples &s = samples[e];
      const auto count = to_u32(nodes.size());
      bool fresh = false;
      if (!s.seen) {
        s.seen = true;
        s.nodes = count;
        s.interior_base = next_id;
        next_id += count >= 2 ? count - 2 : 0;
        s.points.reserve(nodes.size());
        for (const int node : nodes) {
          s.points.push_back(node_point(node));
        }
      } else if (s.nodes != count) {
        ++out.weld_mismatches;
        fresh = true;
      }
      for (std::size_t k = 0; k < nodes.size(); ++k) {
        std::uint32_t &slot = canon[static_cast<std::size_t>(nodes[k])];
        if (k == 0 && v_first != UNSET) {
          slot = v_first;
        } else if (k + 1 == nodes.size() && v_last != UNSET) {
          slot = v_last;
        } else if (slot == UNSET || !fresh) {
          slot = fresh ? next_id++ : s.interior_base + static_cast<std::uint32_t>(k - 1);
        }
      }
    }

    // Normals from the surface at each UV node, facing out of the material; triangle normals
    // where the surface normal vanishes (poles, degenerate points).
    fallback.assign(static_cast<std::size_t>(n_nodes) + 1, gp_Vec(0.0, 0.0, 0.0));
    const int n_tris = tri->NbTriangles();
    for (int t = 1; t <= n_tris; ++t) {
      int a = 0;
      int b = 0;
      int c = 0;
      tri->Triangle(t).Get(a, b, c);
      if (reversed) {
        std::swap(b, c);
      }
      const gp_Pnt pa = node_point(a);
      const gp_Pnt pb = node_point(b);
      const gp_Pnt pc = node_point(c);
      const gp_Vec n = gp_Vec(pa, pb).Crossed(gp_Vec(pa, pc));
      fallback[static_cast<std::size_t>(a)] += n;
      fallback[static_cast<std::size_t>(b)] += n;
      fallback[static_cast<std::size_t>(c)] += n;
      out.tris.push_back(base + static_cast<std::uint32_t>(a - 1));
      out.tris.push_back(base + static_cast<std::uint32_t>(b - 1));
      out.tris.push_back(base + static_cast<std::uint32_t>(c - 1));
    }

    const bool has_uv = tri->HasUVNodes();
    BRepAdaptor_Surface surface(face, false);
    for (int node = 1; node <= n_nodes; ++node) {
      const gp_Pnt p = node_point(node);
      push_point(out.positions, p);
      gp_Vec normal(0.0, 0.0, 0.0);
      if (has_uv) {
        const gp_Pnt2d uv = tri->UVNode(node);
        gp_Pnt on;
        gp_Vec du;
        gp_Vec dv;
        surface.D1(uv.X(), uv.Y(), on, du, dv);
        normal = du.Crossed(dv);
        if (reversed) {
          normal.Reverse();
        }
      }
      const double scale = normal.Magnitude();
      if (!(scale > 1e-12) || !std::isfinite(scale)) {
        normal = fallback[static_cast<std::size_t>(node)];
      }
      const double length = normal.Magnitude();
      if (length > 0.0 && std::isfinite(length)) {
        normal.Divide(length);
      } else {
        normal = gp_Vec(0.0, 0.0, 1.0);
      }
      out.normals.push_back(normal.X());
      out.normals.push_back(normal.Y());
      out.normals.push_back(normal.Z());
      std::uint32_t &id = canon[static_cast<std::size_t>(node)];
      if (id == UNSET) {
        id = next_id++;
      }
      out.weld.push_back(id);
    }
  }
  out.face_tri_starts.push_back(to_u32(out.tris.size() / 3));

  // Edge polylines in edge index order. Free edges have only a Polygon3D.
  out.edge_point_starts.reserve(n_edges + 1);
  for (std::size_t e = 0; e < n_edges; ++e) {
    out.edge_point_starts.push_back(to_u32(out.edge_points.size() / 3));
    const TopoDS_Edge &edge = TopoDS::Edge(edges(static_cast<int>(e) + 1));
    if (BRep_Tool::Degenerated(edge)) {
      continue;
    }
    const EdgeSamples &s = samples[e];
    if (s.seen) {
      for (const gp_Pnt &p : s.points) {
        push_point(out.edge_points, p);
      }
      continue;
    }
    TopLoc_Location loc;
    const Handle(Poly_Polygon3D) &polygon = BRep_Tool::Polygon3D(edge, loc);
    if (polygon.IsNull()) {
      continue;
    }
    const bool moved = !loc.IsIdentity();
    for (const gp_Pnt &node : polygon->Nodes()) {
      gp_Pnt p = node;
      if (moved) {
        p.Transform(loc.Transformation());
      }
      push_point(out.edge_points, p);
    }
  }
  out.edge_point_starts.push_back(to_u32(out.edge_points.size() / 3));
  out.extract_seconds = seconds_since(extract_start);
  return out;
}

rust::Vec<V3> tessellate_edge(const OcIndex &index, std::uint32_t e, double linear,
                              double angular) {
  if (!(linear > 0.0) || !std::isfinite(linear) || !(angular > 0.0) || !std::isfinite(angular)) {
    throw std::invalid_argument("tessellation deflections must be positive and finite");
  }
  const TopoDS_Edge &edge = TopoDS::Edge(index.edges.FindKey(static_cast<int>(e) + 1));
  if (BRep_Tool::Degenerated(edge)) {
    throw std::invalid_argument("edge is degenerated");
  }
  const BRepAdaptor_Curve curve(edge);
  const GCPnts_TangentialDeflection points(curve, angular, linear, 2);
  rust::Vec<V3> out;
  out.reserve(static_cast<std::size_t>(points.NbPoints()));
  for (int i = 1; i <= points.NbPoints(); ++i) {
    const gp_Pnt p = points.Value(i);
    out.push_back(V3{p.X(), p.Y(), p.Z()});
  }
  return out;
}

std::unique_ptr<OcShape> make_sphere(double radius) {
  return std::make_unique<OcShape>(BRepPrimAPI_MakeSphere(gp::Origin(), radius).Shape());
}

std::unique_ptr<OcShape> boolean_cut_probe(const OcShape &object, const OcShape &tool) {
  NCollection_List<TopoDS_Shape> objects;
  objects.Append(object.get());
  NCollection_List<TopoDS_Shape> tools;
  tools.Append(tool.get());
  BRepAlgoAPI_Cut cut;
  cut.SetArguments(objects);
  cut.SetTools(tools);
  cut.SetNonDestructive(true);
  cut.SetRunParallel(false);
  cut.Build();
  if (!cut.IsDone()) {
    throw std::runtime_error("boolean_cut_probe: BRepAlgoAPI_Cut failed");
  }
  return std::make_unique<OcShape>(cut.Shape());
}

} // namespace amg::occt
