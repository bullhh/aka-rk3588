# Build-time guard for the AArch64 cross build (see CMakeLists.txt, tennis).
#
# The cross build links the real target runtime libraries from
# TARGET_RUNTIME_LIB_DIR. The historical "empty stub .so" + unresolved-symbol
# scheme is forbidden: it produced a binary whose uvc_* calls had no dynamic
# relocation and jumped through an unresolved PLT entry (SIGILL) in
# UvcCapture::open.
#
# This script runs as a dependency of the `tennis` target, so a missing or
# incomplete directory fails the build with an actionable message. It is not a
# configure-time check: configuring and building the host test targets alone
# must keep working without any cross runtime directory.
#
# libudev.so.1 is required because the real libusb-1.0.so depends on it. It is
# resolved as a transitive dependency through the -rpath-link directory in
# CMakeLists.txt, so it does not need its own explicit DT_NEEDED entry.

set(_required_libs libuvc.so libusb-1.0.so libturbojpeg.so libudev.so.1)

if(NOT DEFINED TARGET_RUNTIME_LIB_DIR OR TARGET_RUNTIME_LIB_DIR STREQUAL "")
  message(FATAL_ERROR
      "TARGET_RUNTIME_LIB_DIR is not set.\n"
      "The AArch64 cross build must link the real target libraries, not empty stubs.\n"
      "Provide the directory holding the real AArch64 ${_required_libs}.\n"
      "build_rk3588.sh: pass -L <dir> or set AKA_RK3588_CROSS_LIB_DIR=<dir>.\n"
      "Direct CMake: set -DTARGET_RUNTIME_LIB_DIR=<dir>.\n"
      "Configuring and building only the host test targets does not need this variable.")
endif()

if(NOT IS_DIRECTORY "${TARGET_RUNTIME_LIB_DIR}")
  message(FATAL_ERROR
      "TARGET_RUNTIME_LIB_DIR='${TARGET_RUNTIME_LIB_DIR}' is not a directory.")
endif()

set(_missing "")
foreach(_lib IN LISTS _required_libs)
  if(NOT EXISTS "${TARGET_RUNTIME_LIB_DIR}/${_lib}")
    list(APPEND _missing "${TARGET_RUNTIME_LIB_DIR}/${_lib}")
  endif()
endforeach()

if(_missing)
  message(FATAL_ERROR
      "TARGET_RUNTIME_LIB_DIR='${TARGET_RUNTIME_LIB_DIR}' is missing real AArch64 libraries:\n"
      "  ${_missing}\n"
      "Use the Jammy AArch64 libuvc/libusb/libturbojpeg/libudev .so files; empty stubs are not allowed.")
endif()

message(STATUS "Cross-build AArch64 runtime libraries OK: ${TARGET_RUNTIME_LIB_DIR}")
