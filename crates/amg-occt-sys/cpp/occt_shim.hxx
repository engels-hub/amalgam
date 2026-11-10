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

// "<ExceptionType>: <message>".
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
