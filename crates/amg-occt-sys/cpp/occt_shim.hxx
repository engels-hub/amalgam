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
struct OcOpHistory;
struct OcBoolOpts;
struct OcBoolStatus;
struct OcMassProps;
struct OcDistance;
struct OcRayHit;
struct OcSurfacePoint;
struct OcEdgeEnds;
struct OcSubGeom;
struct OcSubMap;
struct OcBlendEdge;
struct OcFilletParams;
struct OcOffsetParams;
struct OcHealParams;
struct OcUnifyParams;
struct OcSewParams;
struct OcCheckProblem;
struct OcPipeParams;
struct OcLoftParams;
struct OcLoftSection;
struct OcDraftParams;
struct OcXdeNode;
struct OcXdeProto;
struct OcXdeFaceColor;
struct OcXdeStatus;
struct OcXdeWriteParams;
struct OcGordon;

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

// The result of one shape op (BREP-14): the shape, and its history from operand 0 in index
// terms. Relation `i` maps input `(in_kinds[i], in_indices[i])` to the outputs
// `out_starts[i]..out_starts[i + 1]`. Only read after the op returns.
//
// Booleans (BREP-04) also fill `in_operands` (one per relation; empty means operand 0 for every
// relation), use relation 3 (kept) and report `cancelled`, BOPAlgo `errors`/`warnings` and the
// operand sub-shapes the errors name.
class OcOp {
public:
  TopoDS_Shape shape;
  std::vector<std::uint16_t> in_operands;
  std::vector<std::uint8_t> in_kinds;
  std::vector<std::uint32_t> in_indices;
  std::vector<std::uint8_t> relations;
  std::vector<std::uint32_t> out_starts{0};
  std::vector<std::uint8_t> out_kinds;
  std::vector<std::uint32_t> out_indices;
  std::vector<std::uint32_t> role_faces;
  std::vector<std::uint8_t> roles;
  // One per role: the contour of a fillet/chamfer face (BREP-06), 0 for every other role.
  std::vector<std::uint32_t> role_args;
  bool cancelled = false;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  std::vector<std::uint16_t> offending_operands;
  std::vector<std::uint8_t> offending_kinds;
  std::vector<std::uint32_t> offending_indices;
  // Heal (BREP-10): the sub-shapes the fixes replaced, removed or re-toleranced.
  std::uint32_t fixes = 0;
};

// What one STEP or IGES read gave (BREP-13): plain nodes, prototype records, face colours and one
// shape per prototype. Defined in occt_xde.cpp; only read after the reader returns.
struct XdeData;
class OcXde {
public:
  OcXde();
  ~OcXde();
  OcXde(const OcXde &) = delete;
  OcXde &operator=(const OcXde &) = delete;
  std::unique_ptr<XdeData> data;
};

// The argument or tool list of one boolean (BREP-04): handles sharing the callers' TShapes.
class OcShapeList {
public:
  std::vector<TopoDS_Shape> shapes;
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

// CATEGORY: R (takes no shape; builds new topology from plain curve data;
// BRepOffsetAPI_MakeOffset runs on the new wire only). The edge map holds (edge, code) pairs:
// code 2*k is the left boundary of chain edge k, 2*k+1 the right one, 2*n and 2*n+1 the end caps.
std::unique_ptr<OcBuild> make_thin_face(const OcPlanarIn &chain, bool closed, double left,
                                        double right, double tol);

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

// -- primitives, sweeps and transforms (BREP-14), implemented in occt_ops.cpp --

// CATEGORY: R (takes no shape; BRepPrimAPI builds new topology)
std::unique_ptr<OcShape> make_primitive(std::uint8_t kind, const OcFrame &frame,
                                        rust::Slice<const double> params);

// CATEGORY: C (BRepSweep adds pcurves to the edges of the base it sweeps, so it sweeps a private
// BRepBuilderAPI_Copy of `profile` made here; draft and UnifySameDomain then run on that new
// result only)
std::unique_ptr<OcOp> prism(const OcShape &profile, const V3 &vec, std::uint8_t mode,
                            double taper, bool merge_coplanar);

// CATEGORY: C (as prism: the sweep runs on a private copy of `profile`)
std::unique_ptr<OcOp> revol(const OcShape &profile, const V3 &origin, const V3 &axis,
                            double angle);

// CATEGORY: R (BRepBuilderAPI_Transform / BRepBuilderAPI_GTransform with copied geometry: the
// input is only read)
std::unique_ptr<OcOp> shape_gtransformed(const OcShape &shape, const OcMat34 &m);

// CATEGORY: R (a new handle sharing the op's result TShape)
std::unique_ptr<OcShape> op_shape(const OcOp &op);

// CATEGORY: R
OcOpHistory op_history(const OcOp &op);

// CATEGORY: C (ShapeUpgrade_UnifySameDomain, faces and edges, on a private BRepBuilderAPI_Copy;
// the caller's TShapes are only read)
std::unique_ptr<OcOp> unify_op(const OcShape &shape);

// -- history and naming inputs (BREP-05), implemented in occt_naming.cpp --

// CATEGORY: R (BRepGProp fixed-order quadrature, BRepAdaptor and BRepBndLib::AddOptimal without
// triangulation; the sub-shapes are only read)
rust::Vec<OcSubGeom> sub_geoms(const OcIndex &index, std::uint8_t kind,
                               rust::Slice<const std::uint32_t> indices);

// CATEGORY: R (index map lookups only)
OcSubMap sub_index_map(const OcIndex &whole, const OcIndex &part);

// -- booleans, split and section (BREP-04), implemented in occt_bool.cpp --

// CATEGORY: R (takes no shape; an empty list)
std::unique_ptr<OcShapeList> new_shape_list();

// CATEGORY: R (stores a handle sharing the TShape; nothing is written)
void shape_list_push(OcShapeList &list, const OcShape &shape);

// CATEGORY: N (BRepAlgoAPI_BooleanOperation / BRepAlgoAPI_Splitter with SetNonDestructive(true),
// on arguments given private top-level TShapes, so the callers' TShapes are only read)
std::unique_ptr<OcOp> boolean_op(std::uint8_t op, const OcShapeList &objects,
                                 const OcShapeList &tools, const OcBoolOpts &opts,
                                 const ProgressSink &sink);

// CATEGORY: R (BRepClass3d_SolidClassifier keeps its state in itself; the solids are only read)
// KH-20: whether a solid of `shape` is inverted (BOPTools_AlgoTools::IsInvertedSolid, the test
// behind BOPAlgo's check_inverted option).
bool shape_has_inverted_solid(const OcShape &shape);

// CATEGORY: N (BRepAlgoAPI_Section with SetNonDestructive(true) on private top-level TShapes:
// both shapes are only read)
std::unique_ptr<OcOp> section_op(const OcShape &a, const OcShape &b, bool parallel,
                                 const ProgressSink &sink);

// CATEGORY: R
OcBoolStatus op_bool_status(const OcOp &op);

// CATEGORY: R (BRepGProp with an adaptive relative error bound `eps`, without triangulation)
double volume_adaptive(const OcShape &shape, double eps);

// -- fillets and chamfers (BREP-06), implemented in occt_blend.cpp --

// CATEGORY: C (ChFi3d writes tolerances and pcurves into the faces and edges it blends, so
// BRepFilletAPI_MakeFillet / _MakeChamfer run on a private BRepBuilderAPI_Copy of `shape` made
// here; `shape` and `index` are only read)
std::unique_ptr<OcOp> blend_op(const OcShape &shape, const OcIndex &index, bool chamfer,
                               rust::Slice<const OcBlendEdge> edges, rust::Slice<const double> laws,
                               const OcFilletParams &params, bool with_history);

// CATEGORY: R (BRep_Tool::CurveOnSurface and BRepAdaptor_Surface::D1; the faces are only read)
double edge_face_angle(const OcIndex &index, std::uint32_t e, std::uint32_t f1, std::uint32_t f2);

// -- shell, offset and thicken (BREP-07), implemented in occt_offset.cpp --

// CATEGORY: C (BRepOffset_MakeOffset trims and re-tolerances the faces it offsets, so it runs on
// a private BRepBuilderAPI_Copy of `shape` made here; `shape` and `index` are only read)
std::unique_ptr<OcOp> offset_op(const OcShape &shape, const OcIndex &index,
                                const OcOffsetParams &params,
                                rust::Slice<const std::uint32_t> remove,
                                rust::Slice<const std::uint32_t> faces,
                                rust::Slice<const double> offsets, const ProgressSink &sink);

// -- healing and assembly (BREP-10), implemented in occt_heal.cpp --

// CATEGORY: C (ShapeFix_Shape and ShapeFix_Wireframe write tolerances into and rebuild what they
// fix, so they run on a private BRepBuilderAPI_Copy (geometry copied too) made here; `shape` is
// only read)
std::unique_ptr<OcOp> heal_op(const OcShape &shape, const OcHealParams &params,
                              const ProgressSink &sink);

// CATEGORY: C (ShapeUpgrade_UnifySameDomain on a private BRepBuilderAPI_Copy made here; `shape`
// is only read)
std::unique_ptr<OcOp> unify_same_domain_op(const OcShape &shape, const OcUnifyParams &params);

// CATEGORY: R (maps and surface types of `shape` only; KH-20: whether unify_same_domain_op could
// merge anything, i.e. does not skip the algorithm under `params.fast`)
bool unify_candidates(const OcShape &shape, const OcUnifyParams &params);

// CATEGORY: C (BRepBuilderAPI_Sewing re-tolerances the edges and vertices it merges, so it runs on
// one private BRepBuilderAPI_Copy of all the shapes made here; they are only read)
std::unique_ptr<OcOp> sew_op(const OcShapeList &shapes, const OcSewParams &params,
                             const ProgressSink &sink);

// CATEGORY: C (BRepBuilderAPI_MakeSolid sets the shell's Closed flag on its TShape, so it runs on a
// private BRepBuilderAPI_Copy made here; `shape` is only read)
std::unique_ptr<OcOp> make_solid_op(const OcShape &shape);

// CATEGORY: N (a new compound that shares the shapes' TShapes; they are only read)
std::unique_ptr<OcShape> make_compound_of(const OcShapeList &shapes);

// CATEGORY: C (BOPAlgo_ArgumentAnalyzer with self-interference, small-edge and curve-on-surface
// tests runs on a private BRepBuilderAPI_Copy made here; `shape` and `index` are only read)
rust::Vec<OcCheckProblem> analyze_shape(const OcShape &shape, const OcIndex &index);

// CATEGORY: R
std::uint32_t op_fix_count(const OcOp &op);

// -- pipe-shell sweeps and the raw loft (BREP-15), implemented in occt_sweep.cpp --

// CATEGORY: C (BRepOffsetAPI_MakePipeShell builds its faces on the section edges and adds pcurves
// to them, so it sweeps private BRepBuilderAPI_Copy copies of every operand made here; the
// operands are only read)
std::unique_ptr<OcOp> pipe_op(const OcShapeList &operands, const OcPipeParams &params,
                              rust::Slice<const double> twist, rust::Slice<const double> scale,
                              const ProgressSink &sink);

// CATEGORY: C (as pipe_op: BRepOffsetAPI_ThruSections lofts private copies of the wires made here)
std::unique_ptr<OcOp> thru_sections_op(const OcShapeList &wires,
                                       rust::Slice<const OcLoftSection> sections,
                                       const OcLoftParams &params, const ProgressSink &sink);

// -- draft, defeature, replace face, split faces (BREP-16), implemented in occt_local.cpp --

// CATEGORY: C (BRepOffsetAPI_DraftAngle rebuilds the faces it tilts and re-tolerances their edges,
// so it runs on a private BRepBuilderAPI_Copy of `shape` made here; `shape` and `index` are only
// read)
std::unique_ptr<OcOp> draft_op(const OcShape &shape, const OcIndex &index,
                               rust::Slice<const std::uint32_t> faces,
                               const OcDraftParams &params, const ProgressSink &sink);

// CATEGORY: C (BRepAlgoAPI_Defeaturing on a private BRepBuilderAPI_Copy made here; `shape` and
// `index` are only read)
std::unique_ptr<OcOp> defeature_op(const OcShape &shape, const OcIndex &index,
                                   rust::Slice<const std::uint32_t> faces, bool parallel,
                                   const ProgressSink &sink);

// CATEGORY: R (samples the face and intersects lines with the tool's surface; both only read)
double face_reach(const OcIndex &index, std::uint32_t face, const OcShape &tool);

// CATEGORY: C (BRepAlgoAPI_Splitter of a private BRepBuilderAPI_Copy made here by the tool face,
// non-destructive; `shape`, `index` and `tool` are only read)
std::unique_ptr<OcOp> trim_face_op(const OcShape &shape, const OcIndex &index, std::uint32_t face,
                                   const OcShape &tool, const ProgressSink &sink);

// CATEGORY: C (BRepAlgoAPI_Section / BRepAlgoAPI_Splitter on private BRepBuilderAPI_Copy copies of
// `shape` and `tool` made here; the callers' shapes are only read)
std::unique_ptr<OcOp> split_faces_op(const OcShape &shape, const OcIndex &index,
                                     rust::Slice<const std::uint32_t> faces, const OcShape &tool,
                                     const ProgressSink &sink);

// -- measurement queries (BREP-09), implemented in occt_query.cpp --

// CATEGORY: R (BRepGProp with an adaptive relative error bound; the shape is only read)
OcMassProps mass_props(const OcShape &shape, const OcIndex &index, double eps);

// CATEGORY: R (BRepClass3d_SolidClassifier per solid of the index; only reads the solids)
std::uint8_t classify_point(const OcIndex &index, const V3 &p, double tol);

// CATEGORY: R (BRepExtrema_DistShapeShape, single-threaded; both shapes are only read)
OcDistance distance_shapes(const OcShape &a, const OcIndex &ia, const OcShape &b,
                           const OcIndex &ib);

// CATEGORY: R (BRepExtrema_DistShapeShape against a new vertex; the shape is only read)
OcDistance closest_point_on(const OcShape &shape, const OcIndex &index, const V3 &p);

// CATEGORY: R (IntCurvesFace_Intersector per face; the faces are only read)
OcRayHit ray_cast_faces(const OcIndex &index, const V3 &origin, const V3 &dir, double max_t,
                        double tol);

// CATEGORY: R (BRepLProp_SLProps on a BRepAdaptor_Surface of the face)
OcSurfacePoint eval_face(const OcIndex &index, std::uint32_t f, double u, double v, double tol);

// CATEGORY: R (BRepExtrema_DistShapeShape and ShapeAnalysis_Surface; the face is only read)
OcDistance project_on_face(const OcIndex &index, std::uint32_t f, const V3 &p, double tol);

// CATEGORY: R (BRepAdaptor_Curve D1 at both ends)
OcEdgeEnds edge_ends(const OcIndex &index, std::uint32_t e);

// -- STEP and IGES through XCAF (BREP-13), implemented in occt_xde.cpp --

// CATEGORY: R (takes no shape; one STEPCAFControl_Reader and one private XCAF document per call,
// thread-safe per instance on OCCT 8.0.1; the translator registration runs once under
// std::call_once; shapes it made get missing 3D curves built in place before they are returned)
std::unique_ptr<OcXde> xde_read_step(rust::Str path, bool names, bool colors,
                                     const ProgressSink &sink);

// CATEGORY: G (IGES translation is not thread-safe in OCCT 8.0: amg-brep calls it under the
// global lock)
std::unique_ptr<OcXde> xde_read_iges(rust::Str path, bool names, bool colors,
                                     const ProgressSink &sink);

// CATEGORY: R
OcXdeStatus xde_status(const OcXde &xde);

// CATEGORY: R
rust::Vec<OcXdeNode> xde_nodes(const OcXde &xde);

// CATEGORY: R
rust::Vec<OcXdeProto> xde_protos(const OcXde &xde);

// CATEGORY: R
rust::Vec<OcXdeFaceColor> xde_face_colors(const OcXde &xde);

// CATEGORY: R (a new handle sharing the read prototype's TShape)
std::unique_ptr<OcShape> xde_proto_shape(const OcXde &xde, std::uint32_t i);

// CATEGORY: C (the XCAF document holds private BRepBuilderAPI_Copy copies of the prototypes made
// here, so the callers' TShapes never enter an XCAF compound; one STEPCAFControl_Writer per call)
OcXdeStatus xde_write_step(rust::Str path, const OcXdeWriteParams &params,
                           const OcShapeList &protos, rust::Slice<const OcXdeProto> meta,
                           rust::Slice<const OcXdeFaceColor> face_colors,
                           rust::Slice<const OcXdeNode> nodes, const ProgressSink &sink);

// CATEGORY: G (IGES translation and the `write.iges.brep.mode` static are not thread-safe:
// amg-brep calls it under the global lock; the shapes are private copies as for xde_write_step)
OcXdeStatus xde_write_iges(rust::Str path, const OcXdeWriteParams &params,
                           const OcShapeList &protos,
                           rust::Slice<const OcXdeProto> meta,
                           rust::Slice<const OcXdeFaceColor> face_colors,
                           rust::Slice<const OcXdeNode> nodes, const ProgressSink &sink);

// CATEGORY: C (strips the 3D curve of edge `edge` on a private BRepBuilderAPI_Copy, then runs the
// readers' missing-curve repair on it; exists to test that repair)
std::unique_ptr<OcShape> xde_guard_probe(const OcShape &shape, std::uint32_t edge);

// -- NURBS conversion and oracles (S20 KH-16), implemented in occt_nurbs.cpp --

// CATEGORY: R (takes no shape; builds a new Geom_BSplineCurve from plain data)
OcCurve bspline_curve_round_trip(const OcCurve &curve);

// CATEGORY: R (takes no shape; builds a new Geom_BSplineSurface from plain data)
OcSurface bspline_surface_round_trip(const OcSurface &surface);

// CATEGORY: R (takes no shape; SetNotPeriodic on a new curve built here)
OcCurve bspline_curve_not_periodic(const OcCurve &curve);

// CATEGORY: R (takes no shape; SetUNotPeriodic/SetVNotPeriodic on a new surface built here)
OcSurface bspline_surface_not_periodic(const OcSurface &surface);

// CATEGORY: R (takes no shape; evaluates a new curve built here, whose cache is its own)
rust::Vec<V3> bspline_curve_d2(const OcCurve &curve, rust::Slice<const double> ts);

// CATEGORY: R (takes no shape; evaluates a new surface built here, whose cache is its own)
rust::Vec<V3> bspline_surface_d2(const OcSurface &surface, rust::Slice<const double> uvs);

// CATEGORY: R (BRepTools_WireExplorer only reads the wire)
rust::Vec<std::uint32_t> wire_edge_order(const OcShape &shape, const OcIndex &index);

// CATEGORY: R (takes no shape; GeomFill_Gordon on new curves built here, serial mode)
OcGordon gordon_surface(rust::Slice<const OcCurve> profiles, rust::Slice<const OcCurve> guides,
                        double tol);

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
