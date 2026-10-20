# Amalgam overlay triplet (S00 §1).
# rustc always links the release CRT (/MD), even in debug builds, so C++ deps are built
# release-only against the dynamic CRT. Static libraries avoid copying DLLs next to every
# test binary.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_BUILD_TYPE release)
