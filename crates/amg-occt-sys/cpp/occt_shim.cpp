#include "amg-occt-sys/cpp/occt_shim.hxx"

#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <GProp_GProps.hxx>
#include <gp_Dir.hxx>

namespace amg::occt {

namespace {

// Pinned by FND-13 (vcpkg baseline 2026.10.16). Changing OCCT must be a deliberate edit here.
constexpr char PINNED_VERSION[] = "8.0.1";
constexpr char SHIM_VERSION[] = "1";

constexpr bool str_eq(const char *a, const char *b) {
  while (*a != '\0' && *a == *b) {
    ++a;
    ++b;
  }
  return *a == *b;
}

static_assert(str_eq(OCC_VERSION_COMPLETE, PINNED_VERSION),
              "OCCT version differs from the pin in occt_shim.cpp (S02 section 1)");

} // namespace

std::string format_failure(const Standard_Failure &failure) {
#if OCC_VERSION_HEX >= 0x080000
  const char *type = failure.ExceptionType();
  const char *message = failure.what();
#else
  const char *type = failure.DynamicType()->Name();
  const char *message = failure.GetMessageString();
#endif
  return std::string(type) + ": " + (message != nullptr ? message : "");
}

rust::String kernel_version() {
  return rust::String(std::string("occt-") + OCC_VERSION_COMPLETE + ";shim-" + SHIM_VERSION);
}

std::unique_ptr<OcShape> make_box(double dx, double dy, double dz) {
  BRepPrimAPI_MakeBox maker(dx, dy, dz);
  return std::make_unique<OcShape>(maker.Shape());
}

double volume(const OcShape &shape) {
  GProp_GProps props;
  BRepGProp::VolumeProperties(shape.get(), props);
  return props.Mass();
}

void gp_dir_from_zero_vector() {
  // volatile: the zero must reach the constructor at run time, not in a constant expression.
  volatile double zero = 0.0;
  const gp_Dir direction(zero, zero, zero);
  (void)direction;
}

} // namespace amg::occt
