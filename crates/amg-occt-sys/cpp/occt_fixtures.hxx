// Fixture parts for testdata/brep/ (BREP-02). Compiled only with the `fixture-gen` feature.
#pragma once

#include <memory>

#include "amg-occt-sys/cpp/occt_shim.hxx"
#include "rust/cxx.h"

namespace amg::occt {

// CATEGORY: R (takes no shape; builds new topology)
std::unique_ptr<OcShape> fixture_shape(rust::Str name);

} // namespace amg::occt
