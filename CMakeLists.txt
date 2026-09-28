cmake_minimum_required(VERSION 3.10)
project(aka_rk3588_tennis)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTING "Build aka-rk3588 unit tests" OFF)
include(CTest)

if(BUILD_TESTING)
  find_program(PYTHON3_EXECUTABLE python3)
  if(NOT PYTHON3_EXECUTABLE)
    message(FATAL_ERROR "python3 is required for launcher tests")
  endif()
  add_test(NAME vision_usb_ci_launcher
      COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/vision_usb_ci_launcher_test.py)

  # Pure FT232R framing/status/selection/verdict logic: no libusb, no hardware.
  add_executable(ftdi_protocol_test
      tests/ftdi_protocol_test.cpp
      usb/ftdi_protocol.cpp
  )
  target_include_directories(ftdi_protocol_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(NAME ftdi_protocol COMMAND ftdi_protocol_test)

  # Linux-only: exercises the FT232R TTY transport selection without hardware.
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(ft232_tty_selection_test
        tests/ft232_tty_selection_test.cpp
        usb/ft232_tty.cpp
        usb/ftdi_protocol.cpp
    )
    target_include_directories(ft232_tty_selection_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_test(NAME ft232_tty_selection COMMAND ft232_tty_selection_test)
  endif()

  # Static/build-gate regression for the AArch64 cross link contract: the empty
  # stub scheme and unresolved-symbol linker flags must never come back.
  add_test(NAME cross_link_policy
      COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/cross_link_policy_test.py)
endif()

# ── Log level ─────────────────────────────────────────────────────────────────
if(NOT DEFINED LOG_LEVEL)
  set(LOG_LEVEL "INFO")
endif()
if(LOG_LEVEL STREQUAL "DEBUG")
  add_definitions(-DCURRENT_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "INFO")
  add_definitions(-DCURRENT_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "WARN")
  add_definitions(-DCURRENT_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "ERROR")
  add_definitions(-DCURRENT_LOG_LEVEL=3)
else()
  add_definitions(-DCURRENT_LOG_LEVEL=1)
endif()

# ── 3rd: self-contained dependency tree (no ../yolov8 needed) ─────────────────
set(3RD ${CMAKE_CURRENT_SOURCE_DIR}/3rd)

if(NOT DEFINED TARGET_SOC)
  set(TARGET_SOC rk3588)
endif()

# ── NATIVE_BUILD: compile directly on the board (no cross-toolchain needed) ──
option(NATIVE_BUILD "Build natively on the board using system libraries" OFF)

# ── RKNN YOLOv8 sources (rknpu2 for rk3588) ──────────────────────────────────
set(RKNPU2_SRC
    ${3RD}/yolov8_src/postprocess.cc
    ${3RD}/yolov8_src/rknpu2/yolov8.cc
)

# ── Common 3rd paths ──────────────────────────────────────────────────────────
set(RKNNRT_DIR         ${3RD}/rknpu2)
set(LIBRKNNRT          ${RKNNRT_DIR}/Linux/aarch64/librknnrt.so)
set(LIBRKNNRT_INCLUDES ${RKNNRT_DIR}/include)

set(STB_INCLUDES        ${3RD}/stb_image)
set(LIBRGA_INCLUDES     ${3RD}/librga/include)

# tennis.rknn is a single-class model (tennis ball only): output=[1,5,N]
add_definitions(-DOBJ_CLASS_NUM=1)
add_definitions(-DDISABLE_RGA)

if(NATIVE_BUILD)
  # ── Board native build ─────────────────────────────────────────────────────
  # librknnrt is already installed on the board; link dynamically, no .so in repo.
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUVC    REQUIRED libuvc)
  pkg_check_modules(LIBUSB    REQUIRED libusb-1.0)
  pkg_check_modules(LIBJPEG   REQUIRED libturbojpeg)

  set(LIBJPEG_INCLUDES    ${LIBJPEG_INCLUDE_DIRS})
  set(LIBJPEG             ${LIBJPEG_LIBRARIES})
  set(LIBRGA              "")
  # The tennis binary does not link audioutils, so sndfile is intentionally
  # optional for board-native builds.
  set(LIBSNDFILE          "")
  set(LIBSNDFILE_INCLUDES "")

  add_subdirectory(${3RD}/utils utils.out)

  include_directories(
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${LIBRKNNRT_INCLUDES}
      ${3RD}/yolov8_src
      ${3RD}/utils
      ${LIBUVC_INCLUDE_DIRS}
      ${LIBUSB_INCLUDE_DIRS}
      ${LIBJPEG_INCLUDE_DIRS}
  )

  add_executable(tennis
      tennis.cpp
      vision_usb_ci.cpp
      usb/ftdi_protocol.cpp usb/ft232_loopback.cpp usb/ft232_tty.cpp
      capture/uvc_capture.cpp
      detect/detect.cpp
      ${RKNPU2_SRC}
  )

  target_link_directories(tennis PRIVATE
      ${LIBUVC_LIBRARY_DIRS}
      ${LIBUSB_LIBRARY_DIRS}
      ${LIBJPEG_LIBRARY_DIRS}
  )

  # Use the bundled librknnrt.so (newer than what's installed on the board)
  target_link_libraries(tennis
      imageutils fileutils imagedrawing
      ${LIBRKNNRT}
      ${LIBUVC_LIBRARIES}
      ${LIBUSB_LIBRARIES}
      ${LIBJPEG_LIBRARIES}
      -lpthread -ldl -lm
  )

  # Set RPATH so the binary finds the bundled librknnrt.so at runtime.
  # Deploy the .so alongside the binary (e.g. same directory or ../lib/).
  set_target_properties(tennis PROPERTIES
      BUILD_RPATH   "${RKNNRT_DIR}/Linux/aarch64"
      INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib"
  )

else()
  # ── Cross-compile on PC ────────────────────────────────────────────────────
  # The published AArch64 binary must link the real target runtime libraries.
  # The former "empty stub .so" + --unresolved-symbols=ignore-all scheme is
  # forbidden: it produced a binary whose uvc_* calls had no dynamic relocation
  # and jumped through an unresolved PLT entry (SIGILL) in UvcCapture::open.
  #
  # TARGET_RUNTIME_LIB_DIR points at a directory holding the real AArch64
  # libuvc.so, libusb-1.0.so and libturbojpeg.so taken from the target rootfs
  # (Ubuntu 22.04 / Jammy). Configuring without it still works so the three host
  # test targets can be built, but building `tennis` then fails through the
  # guard target below instead of emitting a broken binary.
  set(TARGET_RUNTIME_LIB_DIR "" CACHE PATH
      "Directory with real AArch64 libuvc.so, libusb-1.0.so and libturbojpeg.so for the cross build")

  set(TARGET_UVC_LIB     "${TARGET_RUNTIME_LIB_DIR}/libuvc.so")
  set(TARGET_USB_LIB     "${TARGET_RUNTIME_LIB_DIR}/libusb-1.0.so")
  set(TARGET_JPEG_LIB    "${TARGET_RUNTIME_LIB_DIR}/libturbojpeg.so")

  set(LIBJPEG_DIR       ${3RD}/jpeg_turbo)
  set(LIBJPEG_INCLUDES  ${LIBJPEG_DIR}/include)
  set(LIBJPEG           "")
  set(LIBRGA            "")
  set(LIBSNDFILE        "")
  set(LIBSNDFILE_INCLUDES "")

  add_subdirectory(${3RD}/utils utils.out)

  include_directories(
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${LIBRKNNRT_INCLUDES}
      ${3RD}/yolov8_src
      ${3RD}/utils
      ${LIBJPEG_DIR}/include
      /usr/include   # fallback for libuvc/libuvc.h
  )

  add_executable(tennis
      tennis.cpp
      vision_usb_ci.cpp
      usb/ftdi_protocol.cpp usb/ft232_loopback.cpp usb/ft232_tty.cpp
      capture/uvc_capture.cpp
      detect/detect.cpp
      ${RKNPU2_SRC}
  )

  # Build-time guard: a missing/incomplete TARGET_RUNTIME_LIB_DIR fails loudly
  # here, before the linker could ever produce an unresolved binary. Only
  # `tennis` depends on it, so the host test targets stay buildable without the
  # cross runtime directory.
  add_custom_target(tennis_requires_target_runtime_libs
      COMMAND ${CMAKE_COMMAND}
              "-DTARGET_RUNTIME_LIB_DIR=${TARGET_RUNTIME_LIB_DIR}"
              -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/require_target_runtime_libs.cmake"
      COMMENT "Checking the real AArch64 runtime libraries for the cross build"
      VERBATIM
  )
  add_dependencies(tennis tennis_requires_target_runtime_libs)

  # Link the real AArch64 libraries by absolute path. Recording the library
  # SONAME keeps the final DT_NEEDED entries (libuvc.so.0, libusb-1.0.so.0,
  # libturbojpeg.so.0) satisfiable by the Jammy runtime libraries shipped with
  # the target rootfs. No host x86 library can satisfy these absolute paths.
  # libusb-1.0.so depends on libudev.so.1; -rpath-link resolves that transitive
  # dependency from the same directory without an explicit DT_NEEDED entry.
  target_link_options(tennis PRIVATE
      -Wl,-rpath-link,${TARGET_RUNTIME_LIB_DIR}
  )
  target_link_libraries(tennis
      imageutils fileutils imagedrawing
      ${LIBRKNNRT}
      ${TARGET_UVC_LIB}
      ${TARGET_USB_LIB}
      ${TARGET_JPEG_LIB}
      -lpthread -ldl -lm
  )
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS tennis DESTINATION bin)
