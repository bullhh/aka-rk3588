# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(orangepi_robot_control_tests)

set(ROBOT_APP_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

target_include_directories(app PRIVATE
  ${ROBOT_APP_DIR}/src
  ${ROBOT_APP_DIR}/../../protocol
)
target_sources(app PRIVATE
  src/main.c
  ${ROBOT_APP_DIR}/src/robot_controller.c
  ${ROBOT_APP_DIR}/src/resettable_watchdog.c
)
