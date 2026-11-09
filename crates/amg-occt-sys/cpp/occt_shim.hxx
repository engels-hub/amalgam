// C++ side of the amg-occt-sys bridge (S02 §1, occt-integration.md §3).
//
// One function per kernel operation, each tagged with its S02 §2 thread-safety category, which
// BREP-12's lint enforces.
#pragma once

#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopoDS_Shape.hxx>

#include "rust/cxx.h"

namespace amg::occt {

class OcShape {
public:
  explicit OcShape(TopoDS_Shape shape) : shape_(std::move(shape)) {}
  const TopoDS_Shape &get() const noexcept { return shape_; }

private:
  TopoDS_Shape shape_;
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
