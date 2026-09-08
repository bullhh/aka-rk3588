cmake_minimum_required(VERSION 3.10)
project(aka_rk3588_tennis)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTING "Build aka-rk3588 unit tests" OFF)
include(CTest)

if(BUILD_TESTING)
  add_executable(feetech_motion_policy_test
      tests/feetech_motion_policy_test.cpp
      robot/feetech_motion_policy.cpp
  )
  target_include_directories(feetech_motion_policy_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(NAME feetech_motion_policy COMMAND feetech_motion_policy_test)

  add_executable(bucket_detector_test
      tests/bucket_detector_test.cpp
      perception/bucket_detector.cpp
  )
  target_include_directories(bucket_detector_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(NAME bucket_detector COMMAND bucket_detector_test)

  add_executable(robot_runtime_config_test
      tests/robot_runtime_config_test.cpp
      robot/lekiwi_runtime_config.cpp
      robot/lekiwi_calibration.cpp
      zephyr/orangepi_robot_control/src/robot_config_receiver.c
  )
  target_include_directories(robot_runtime_config_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/protocol
      ${CMAKE_CURRENT_SOURCE_DIR}/zephyr/orangepi_robot_control/src
  )
  target_compile_definitions(robot_runtime_config_test PRIVATE
      AKA_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(NAME robot_runtime_config COMMAND robot_runtime_config_test)

  add_executable(ivc_publish_retry_test
      tests/ivc_publish_retry_test.cpp
  )
  target_include_directories(ivc_publish_retry_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(NAME ivc_publish_retry COMMAND ivc_publish_retry_test)
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

set(PERCEPTION_SOURCES
    perception/perception_main.cpp
    perception/bucket_detector.cpp
    robot/lekiwi_runtime_config.cpp
    robot/lekiwi_calibration.cpp
    capture/uvc_capture.cpp
    detect/detect.cpp
    ${RKNPU2_SRC}
)

# AXIVC is shared by the StarryOS perception publisher and the Zephyr control
# guest.  Keep the SDK in its own repository and compile its Linux/StarryOS
# user library into tennis-perception instead of copying the transport code.
set(IVC_SDK_DIR "" CACHE PATH "Path to the ivc-sdk checkout")
if(NOT IVC_SDK_DIR AND DEFINED ENV{IVC_SDK_DIR})
  set(IVC_SDK_DIR "$ENV{IVC_SDK_DIR}")
endif()
if(NOT IVC_SDK_DIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../ivc-sdk")
  set(IVC_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ivc-sdk")
endif()
if(NOT EXISTS "${IVC_SDK_DIR}/src/axivc.c" OR
   NOT EXISTS "${IVC_SDK_DIR}/include/axivc/axivc.h")
  message(FATAL_ERROR
    "ivc-sdk not found; set -DIVC_SDK_DIR=/path/to/ivc-sdk or place it next to aka-rk3588")
endif()

add_library(axivc_sdk STATIC "${IVC_SDK_DIR}/src/axivc.c")
target_include_directories(axivc_sdk PUBLIC "${IVC_SDK_DIR}/include")
set_target_properties(axivc_sdk PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)

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
      test_cmds.cpp
      arm/arm.cpp
      feetech/feetech_bus.cpp
      motor/motor.cpp motor/pwm_motor_driver.cpp motor/uart_motor_driver.cpp
      robot/omni_base.cpp robot/feetech_arm.cpp robot/lekiwi_calibration.cpp robot/lekiwi_arm_poses.cpp
      robot/feetech_motion_policy.cpp
      robot/lekiwi_task_controller.cpp
      capture/uvc_capture.cpp
      detect/detect.cpp
      ${RKNPU2_SRC}
  )

  add_executable(tennis-perception ${PERCEPTION_SOURCES})

  target_link_directories(tennis PRIVATE
      ${LIBUVC_LIBRARY_DIRS}
      ${LIBUSB_LIBRARY_DIRS}
      ${LIBJPEG_LIBRARY_DIRS}
  )
  target_link_directories(tennis-perception PRIVATE
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
  target_link_libraries(tennis-perception
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
  set_target_properties(tennis-perception PROPERTIES
      BUILD_RPATH   "${RKNNRT_DIR}/Linux/aarch64"
      INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib"
  )

else()
  # ── Cross-compile on PC ────────────────────────────────────────────────────
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
      test_cmds.cpp
      arm/arm.cpp
      feetech/feetech_bus.cpp
      motor/motor.cpp motor/pwm_motor_driver.cpp motor/uart_motor_driver.cpp
      robot/omni_base.cpp robot/feetech_arm.cpp robot/lekiwi_calibration.cpp robot/lekiwi_arm_poses.cpp
      robot/feetech_motion_policy.cpp
      robot/lekiwi_task_controller.cpp
      capture/uvc_capture.cpp
      detect/detect.cpp
      ${RKNPU2_SRC}
  )

  add_executable(tennis-perception ${PERCEPTION_SOURCES})

  # Generate empty stub .so so the linker can resolve -luvc etc.
  foreach(STUBLIB uvc usb-1.0 turbojpeg)
      set(STUB_SRC ${CMAKE_CURRENT_BINARY_DIR}/stub_${STUBLIB}.c)
      set(STUB_SO  ${CMAKE_CURRENT_BINARY_DIR}/lib${STUBLIB}.so)
      if(STUBLIB STREQUAL "uvc")
          set(STUB_SONAME libuvc.so.0)
      elseif(STUBLIB STREQUAL "usb-1.0")
          set(STUB_SONAME libusb-1.0.so.0)
      else()
          set(STUB_SONAME libturbojpeg.so.0)
      endif()
      file(WRITE ${STUB_SRC} "/* stub */\n")
      add_custom_command(
          OUTPUT  ${STUB_SO}
          COMMAND ${CMAKE_C_COMPILER} -shared -fPIC
                  -Wl,-soname,${STUB_SONAME} -o ${STUB_SO} ${STUB_SRC}
          DEPENDS ${STUB_SRC}
          COMMENT "Generating stub lib${STUBLIB}.so"
      )
      list(APPEND STUB_SO_LIST ${STUB_SO})
  endforeach()
  add_custom_target(stub_libs ALL DEPENDS ${STUB_SO_LIST})
  add_dependencies(tennis stub_libs)
  add_dependencies(tennis-perception stub_libs)

  target_link_options(tennis PRIVATE
      -Wl,--allow-shlib-undefined
      -Wl,--unresolved-symbols=ignore-all
      -L${CMAKE_CURRENT_BINARY_DIR}
  )
  target_link_options(tennis-perception PRIVATE
      -Wl,--allow-shlib-undefined
      -Wl,--unresolved-symbols=ignore-all
      -L${CMAKE_CURRENT_BINARY_DIR}
  )

  target_link_libraries(tennis
      imageutils fileutils imagedrawing
      ${LIBRKNNRT}
      -Wl,--no-as-needed
      -luvc -lusb-1.0 -lturbojpeg
      -Wl,--as-needed
      -lpthread -ldl -lm
  )
  target_link_libraries(tennis-perception
      imageutils fileutils imagedrawing
      ${LIBRKNNRT}
      -Wl,--no-as-needed
      -luvc -lusb-1.0 -lturbojpeg
      -Wl,--as-needed
      -lpthread -ldl -lm
  )
endif()

target_link_libraries(tennis-perception axivc_sdk)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS tennis DESTINATION bin)
set_target_properties(tennis-perception PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
)
install(TARGETS tennis-perception
    RUNTIME DESTINATION bin
    COMPONENT dual-runtime
)
install(FILES 3rd/rknpu2/Linux/aarch64/librknnrt.so
    DESTINATION lib
    COMPONENT dual-runtime
)
install(FILES models/tennis.rknn
    DESTINATION models
    COMPONENT dual-runtime
)
install(FILES
    config/lekiwi_calibration.json
    config/lekiwi_pick_config.txt
    DESTINATION config
    COMPONENT dual-runtime
)
install(PROGRAMS
    run_dual_pick.sh
    run_dual_pick_ci_once.sh
    DESTINATION .
    COMPONENT dual-runtime
)
