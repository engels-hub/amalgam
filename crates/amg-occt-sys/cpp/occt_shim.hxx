// C++ side of the amg-occt-sys bridge (S02 §1, occt-integration.md §3).
//
// One function per kernel operation, each tagged with its S02 §2 thread-safety category, which
// BREP-12's lint enforces.
#pragma once

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <NCollection_IndexedMap.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Shape.hxx>

#include "rust/cxx.h"

namespace amg::occt {

// Shared structs and enums, defined by the generated bridge header.
struct V3;
struct OcMat34;
struct OcBox;
struct OcUvBounds;
struct OcTopology;
struct OcSurface;
struct OcCurve;
struct OcCheckReport;
struct OcTessParams;
struct OcTessMesh;
struct OcEdgeIn;
struct OcLoopsIn;
struct OcPlanarIn;
struct OcSection;
struct OcFrame;

// Defined on the Rust side (`extern "Rust"` in bridge.rs): the progress/cancel target of one call.
struct ProgressSink;

class OcShape {
public:
  explicit OcShape(TopoDS_Shape shape) : shape_(std::move(shape)) {}
  const TopoDS_Shape &get() const noexcept { return shape_; }

private:
  TopoDS_Shape shape_;
};

using ShapeMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;

// The sub-shape maps of one shape, in TopExp::MapShapes order (S02 §3). Only read after
// build_index returns.
class OcIndex {
public:
  ShapeMap faces;
  ShapeMap edges;
  ShapeMap vertices;
  ShapeMap solids;
};

// What one builder call made (BREP-03): shapes, and per shape the edge index of each input edge
// (UINT32_MAX where it did not survive). Only read after the builder returns.
class OcBuild {
public:
  std::vector<TopoDS_Shape> shapes;
  std::vector<std::vector<std::uint32_t>> edge_maps;
};

// CATEGORY: R (formats an exception; not bridged, used by trycatch below)
std::string format_failure(const Standard_Failure &failure);

// CATEGORY: R (reads only compile-time constants)
rust::String kernel_version();

// CATEGORY: R (takes no shape; builds new topology)
std::unique_ptr<OcShape> make_box(double dx, double dy, double dz);

// CATEGORY: R
double volume(const OcShape &shape);

// CATEGORY: R (takes no shape; always throws)
void gp_dir_from_zero_vector();

// CATEGORY: R
bool shape_is_null(const OcShape &shape);

// CATEGORY: R
std::uint8_t shape_kind(const OcShape &shape);

// CATEGORY: R
bool shape_is_same(const OcShape &a, const OcShape &b);

// CATEGORY: R (a new located handle; the shared TShape is not written)
std::unique_ptr<OcShape> shape_moved(const OcShape &shape, const OcMat34 &m);

// CATEGORY: R (BRepBndLib::AddOptimal without triangulation)
OcBox shape_bbox(const OcShape &shape);

// CATEGORY: R
std::unique_ptr<OcIndex> build_index(const OcShape &shape);

// CATEGORY: R
OcTopology index_topology(const OcIndex &index);

// CATEGORY: R
std::uint32_t index_solid_count(const OcIndex &index);

// CATEGORY: R (a new handle sharing the TShape)
std::unique_ptr<OcShape> index_solid(const OcIndex &index, std::uint32_t i);

// CATEGORY: R (a new handle sharing the TShape)
std::unique_ptr<OcShape> index_sub_shape(const OcIndex &index, std::uint8_t kind, std::uint32_t i);

// CATEGORY: R
double index_tolerance(const OcIndex &index, std::uint8_t kind, std::uint32_t i);

// CATEGORY: R
std::uint64_t index_estimated_bytes(const OcIndex &index);

// CATEGORY: R (BRep_Tool::Surface copies the geometry when the face has a location)
OcSurface face_surface(const OcIndex &index, std::uint32_t f);

// CATEGORY: R
bool face_is_reversed(const OcIndex &index, std::uint32_t f);

// CATEGORY: R
OcUvBounds face_uv_bounds(const OcIndex &index, std::uint32_t f);

// CATEGORY: R (BRepGProp without triangulation)
double face_area(const OcIndex &index, std::uint32_t f);

// CATEGORY: R (BRep_Tool::Curve copies the geometry when the edge has a location)
OcCurve edge_curve(const OcIndex &index, std::uint32_t e);

// CATEGORY: R
bool edge_is_seam(const OcIndex &index, std::uint32_t e, std::uint32_t f);

// CATEGORY: R
V3 vertex_point(const OcIndex &index, std::uint32_t v);

// CATEGORY: R (BinTools write)
std::unique_ptr<std::string> shape_write_bin(const OcShape &shape);

// CATEGORY: R (takes no shape; binary read does not use Standard_Strtod)
std::unique_ptr<OcShape> shape_read_bin(rust::Slice<const std::uint8_t> data);

// CATEGORY: R (BRepTools write)
std::unique_ptr<std::string> shape_write_text(const OcShape &shape);

// CATEGORY: R (takes no shape; OCCT 8.0.1 made Standard_Strtod thread-safe, so no G lock)
std::unique_ptr<OcShape> shape_read_text(rust::Slice<const std::uint8_t> data);

// -- kernel infrastructure (BREP-12) --

// CATEGORY: G (replaces the process-wide default messenger's printers; called once under
// amg_brep::init's OnceLock, before any kernel work)
void install_message_printer();

// CATEGORY: R (takes no shape; sends through the default messenger, whose printer list is only
// read)
void emit_message(std::uint8_t gravity, rust::Str text);

// CATEGORY: G (creates or resizes the process-wide OSD_ThreadPool::DefaultPool; called once
// under amg_brep::init's OnceLock)
std::int32_t set_default_thread_pool(std::int32_t threads);

// CATEGORY: R (reads the default pool's thread count)
std::int32_t default_thread_pool_size();

// CATEGORY: R (takes no shape; a progress scope of `steps` busy-waits of `step_micros` each)
bool progress_test_op(const ProgressSink &sink, std::uint32_t steps, std::uint32_t step_micros);

// CATEGORY: R (ShapeAnalysis_ShapeTolerance keeps its running maximum in itself)
double shape_max_tolerance(const OcShape &shape);

// CATEGORY: R (BRepCheck_Analyzer keeps its results in its own maps)
OcCheckReport check_shape(const OcShape &shape, const OcIndex &index);

// CATEGORY: R (takes no shape; builds a face on an open wire, which BRepCheck rejects)
std::unique_ptr<OcShape> make_open_wire_face();

// -- tessellation (BREP-08), implemented in occt_tess.cpp --

// CATEGORY: C (BRepMesh_IncrementalMesh writes triangulations into the faces and edges it meshes,
// so it runs on a private BRepBuilderAPI_Copy of `shape` made here; `shape` is only read)
OcTessMesh tessellate_shape(const OcShape &shape, const OcTessParams &params,
                            const ProgressSink &sink);

// CATEGORY: R (GCPnts_TangentialDeflection over a BRepAdaptor_Curve; the edge is only read)
rust::Vec<V3> tessellate_edge(const OcIndex &index, std::uint32_t e, double linear,
                              double angular);

// CATEGORY: R (takes no shape; builds new topology)
std::unique_ptr<OcShape> make_sphere(double radius);

// CATEGORY: N (BRepAlgoAPI_Cut with SetNonDestructive(true); a test probe for the BREP-08
// concurrency test until BREP-04 lands the real booleans)
std::unique_ptr<OcShape> boolean_cut_probe(const OcShape &object, const OcShape &tool);

// -- face and wire builders (BREP-03), implemented in occt_build.cpp --

// CATEGORY: R (takes no shape; builds new topology from plain curve data)
std::unique_ptr<OcBuild> make_planar_faces(rust::Slice<const OcPlanarIn> faces, double tol);

// CATEGORY: R (takes no shape; builds new topology; ShapeFix_Face runs on that new face only)
std::unique_ptr<OcBuild> make_face_on_surface(const OcSurface &surface, std::uint8_t bounds,
                                              const OcUvBounds &uv, const OcLoopsIn &loops,
                                              rust::Slice<const V3> points,
                                              rust::Slice<const std::uint32_t> point_starts,
                                              double tol);

// CATEGORY: R (takes no shape; builds new topology from plain curve data)
std::unique_ptr<OcBuild> make_wire(const OcLoopsIn &edges, bool closed, double tol);

// CATEGORY: R
std::uint32_t build_count(const OcBuild &build);

// CATEGORY: R (a new handle sharing the built TShape)
std::unique_ptr<OcShape> build_shape(const OcBuild &build, std::uint32_t i);

// CATEGORY: R
rust::Vec<std::uint32_t> build_edge_map(const OcBuild &build, std::uint32_t i);

// -- geometry extraction (BREP-17), implemented in occt_extract.cpp --

// CATEGORY: N (BRepAlgoAPI_Section with SetNonDestructive(true) against a new finite face on
// `plane`; `shape` is only read)
OcSection plane_section(const OcShape &shape, const OcIndex &index, const OcFrame &plane,
                        double tol);

// CATEGORY: R (GeomConvert / GeomConvert_ApproxCurve on a copy of the edge's curve; the edge is
// only read)
OcCurve edge_curve_recognised(const OcIndex &index, std::uint32_t e, double tol);

} // namespace amg::occt

// The cxx customisation point: every bridge call runs inside this handler. `Standard_Failure` is
// caught by name so the shim also compiles against the 7.9.3 fallback, where it does not derive
// from std::exception (occt-integration.md §1.3, §3.3).
namespace rust {
namespace behavior {

template <typename Try, typename Fail>
static void trycatch(Try &&func, Fail &&fail) noexcept try {
  func();
} catch (const Standard_Failure &failure) {
  const std::string message = amg::occt::format_failure(failure);
  fail(message.c_str());
} catch (const std::exception &error) {
  fail(error.what());
} catch (...) {
  fail("unknown C++ exception");
}

} // namespace behavior
} // namespace rust
