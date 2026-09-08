#ifndef ROBOT_LEKIWI_RUNTIME_CONFIG_HPP
#define ROBOT_LEKIWI_RUNTIME_CONFIG_HPP

#include <string>

#include "protocol/robot_runtime_config_v1.h"

bool build_lekiwi_runtime_config(
    const std::string& calibration_path,
    const std::string& pick_config_path,
    robot_runtime_config_v1& output,
    std::string& error);

#endif // ROBOT_LEKIWI_RUNTIME_CONFIG_HPP
