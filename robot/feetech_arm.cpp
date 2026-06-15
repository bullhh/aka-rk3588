#include "feetech_arm.hpp"

#include <algorithm>
#include <stdio.h>
#include <unistd.h>
#include <vector>

FeetechArm::FeetechArm(feetech::FeetechBus& bus, const std::string& calibration_path) : bus_(bus) {
    joints_ = {
        {"arm_shoulder_pan",  {1, -100.0f, 100.0f, false}},
        {"arm_shoulder_lift", {2, -100.0f, 100.0f, false}},
        {"arm_elbow_flex",    {3, -100.0f, 100.0f, false}},
        {"arm_wrist_flex",    {4, -100.0f, 100.0f, false}},
        {"arm_wrist_roll",    {5, -100.0f, 100.0f, false}},
        {"arm_gripper",       {6,    0.0f, 100.0f, true}},
    };
    load_calibration(calibration_path);
    poses_.load("config/lekiwi_arm_poses.txt");
}

bool FeetechArm::load_calibration(const std::string& path) {
    calibrated_ = calibration_.load(path);
    calibration_error_ = calibration_.last_error();
    if (!calibrated_) return false;

    for (auto& kv : joints_) {
        const JointCalibration* cal = calibration_.get(kv.first);
        if (!cal) {
            calibrated_ = false;
            calibration_error_ = "missing joint calibration: " + kv.first;
            return false;
        }
        kv.second.id = cal->id;
    }
    return true;
}

bool FeetechArm::configure() {
    if (!calibrated_) {
        last_error_ = "not calibrated: " + calibration_error_;
        return false;
    }
    for (const auto& kv : joints_) {
        int id = kv.second.id;
        if (!bus_.enable_torque(id, false)) {
            last_error_ = "configure " + kv.first + " id=" + std::to_string(id) +
                          " torque-off failed: " + bus_.last_error();
            fprintf(stderr, "[FeetechArm] configure %s id=%d torque-off failed: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
            return false;
        }
        if (!bus_.set_operating_mode(id, feetech::OperatingMode::POSITION)) {
            last_error_ = "configure " + kv.first + " id=" + std::to_string(id) +
                          " position-mode failed: " + bus_.last_error();
            fprintf(stderr, "[FeetechArm] configure %s id=%d position-mode failed: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
            return false;
        }
        if (!bus_.write_u8(id, feetech::reg::P_COEFFICIENT, 16)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d P coefficient skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }
        if (!bus_.write_u8(id, feetech::reg::I_COEFFICIENT, 0)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d I coefficient skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }
        if (!bus_.write_u8(id, feetech::reg::D_COEFFICIENT, 32)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d D coefficient skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }
        if (!bus_.set_acceleration(id, 80)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d acceleration skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }
        if (!bus_.enable_torque(id, true)) {
            last_error_ = "configure " + kv.first + " id=" + std::to_string(id) +
                          " torque-on failed: " + bus_.last_error();
            fprintf(stderr, "[FeetechArm] configure %s id=%d torque-on failed: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
            return false;
        }
    }
    return true;
}

int FeetechArm::deg_to_raw(const Joint& joint, float deg) const {
    deg = std::max(joint.min_deg, std::min(joint.max_deg, deg));
    const JointCalibration* cal = nullptr;
    for (const auto& kv : joints_) {
        if (kv.second.id == joint.id) {
            cal = calibration_.get(kv.first);
            break;
        }
    }
    if (!cal) return 2048;

    // LeRobot LeKiwi uses normalized positions by default:
    // arm joints 1-5 are RANGE_M100_100, gripper is RANGE_0_100.
    // The variable is still named deg for compatibility with older commands.
    float t = (deg - joint.min_deg) / (joint.max_deg - joint.min_deg);
    int raw = (int)(cal->range_min + t * (cal->range_max - cal->range_min) + 0.5f);
    return std::max(cal->range_min, std::min(cal->range_max, raw));
}

float FeetechArm::raw_to_deg(const Joint& joint, int raw) const {
    const JointCalibration* cal = nullptr;
    for (const auto& kv : joints_) {
        if (kv.second.id == joint.id) {
            cal = calibration_.get(kv.first);
            break;
        }
    }
    if (!cal) return 0.0f;

    int span = std::max(1, cal->range_max - cal->range_min);
    float t = (raw - cal->range_min) / (float)span;
    float deg = joint.min_deg + t * (joint.max_deg - joint.min_deg);
    return std::max(joint.min_deg, std::min(joint.max_deg, deg));
}

bool FeetechArm::set_joint_deg(const std::string& name, float deg) {
    auto it = joints_.find(name);
    if (it == joints_.end()) {
        last_error_ = "unknown joint: " + name;
        return false;
    }
    if (!bus_.set_goal_position(it->second.id, deg_to_raw(it->second, deg))) {
        last_error_ = "write joint " + name + " id=" + std::to_string(it->second.id) +
                      " failed: " + bus_.last_error();
        return false;
    }
    return true;
}

bool FeetechArm::set_joint_raw(const std::string& name, int raw) {
    auto it = joints_.find(name);
    if (it == joints_.end()) {
        last_error_ = "unknown joint: " + name;
        return false;
    }
    raw = std::max(0, std::min(4095, raw));
    if (!bus_.set_goal_position(it->second.id, raw)) {
        last_error_ = "write raw joint " + name + " id=" + std::to_string(it->second.id) +
                      " failed: " + bus_.last_error();
        return false;
    }
    return true;
}

bool FeetechArm::get_joint_deg(const std::string& name, float& deg) {
    auto it = joints_.find(name);
    if (it == joints_.end()) {
        last_error_ = "unknown joint: " + name;
        return false;
    }
    int raw = 0;
    static const int kReadAttempts = 4;
    for (int attempt = 1; attempt <= kReadAttempts; attempt++) {
        if (bus_.read_u16(it->second.id, feetech::reg::PRESENT_POSITION, raw, true)) {
            if (attempt > 1) {
                fprintf(stderr, "[FeetechArm] read joint %s id=%d recovered on attempt %d\n",
                        name.c_str(), it->second.id, attempt);
            }
            deg = raw_to_deg(it->second, raw);
            return true;
        }
        if (attempt < kReadAttempts) usleep(20000);
    }
    {
        last_error_ = "read joint " + name + " id=" + std::to_string(it->second.id) +
                      " position failed: " + bus_.last_error();
        fprintf(stderr, "[FeetechArm] %s\n", last_error_.c_str());
        return false;
    }
}

bool FeetechArm::get_joint_degs(std::map<std::string, float>& positions) {
    positions.clear();
    bool ok = true;
    for (const auto& kv : joints_) {
        float deg = 0.0f;
        if (get_joint_deg(kv.first, deg)) positions[kv.first] = deg;
        else ok = false;
    }
    return ok;
}

std::vector<std::string> FeetechArm::joint_names() const {
    std::vector<std::string> names;
    names.reserve(joints_.size());
    for (const auto& kv : joints_) names.push_back(kv.first);
    return names;
}

bool FeetechArm::write_degrees(const std::map<std::string, float>& pose, int settle_ms) {
    return write_pose(pose, settle_ms);
}

bool FeetechArm::write_pose(const std::map<std::string, float>& pose, int settle_ms) {
    std::vector<std::pair<int, int>> goals;
    for (const auto& kv : pose) {
        auto it = joints_.find(kv.first);
        if (it == joints_.end()) continue;
        goals.push_back({it->second.id, deg_to_raw(it->second, kv.second)});
    }
    if (goals.empty()) {
        last_error_ = "write pose failed: no valid joints";
        return false;
    }
    bool ok = bus_.sync_write_u16(feetech::reg::GOAL_POSITION, goals, true);
    if (!ok) last_error_ = "sync write pose failed: " + bus_.last_error();
    if (settle_ms > 0) usleep(settle_ms * 1000);
    return ok;
}

bool FeetechArm::write_raw_pose(const ArmRawPose& pose, int settle_ms) {
    std::vector<std::pair<int, int>> goals;
    for (const auto& kv : pose.joints) {
        auto it = joints_.find(kv.first);
        if (it == joints_.end()) continue;
        const JointCalibration* cal = calibration_.get(kv.first);
        int raw = kv.second;
        if (cal) raw = std::max(cal->range_min, std::min(cal->range_max, raw));
        else raw = std::max(0, std::min(4095, raw));
        goals.push_back({it->second.id, raw});
    }
    if (goals.empty()) return false;
    bool ok = bus_.sync_write_u16(feetech::reg::GOAL_POSITION, goals, true);
    if (settle_ms > 0) usleep(settle_ms * 1000);
    return ok;
}

bool FeetechArm::run_pose(const std::string& name, int settle_ms) {
    const ArmRawPose* pose = poses_.get(name);
    if (!pose) return false;
    return write_raw_pose(*pose, settle_ms);
}

bool FeetechArm::grab_pos() {
    if (run_pose("home", 1200)) return true;
    return write_pose({
        {"arm_shoulder_pan", 0.0f},
        {"arm_shoulder_lift", -31.7f},
        {"arm_elbow_flex", 27.7f},
        {"arm_wrist_flex", 80.0f},
        {"arm_wrist_roll", 0.0f},
        {"arm_gripper", 10.0f},
    }, 1200);
}

bool FeetechArm::release_pos() {
    if (run_pose("release_pos", 1200)) return true;
    return write_pose({
        {"arm_shoulder_pan", -12.0f},
        {"arm_shoulder_lift", -55.0f},
        {"arm_elbow_flex", 45.0f},
        {"arm_wrist_flex", 80.0f},
        {"arm_wrist_roll", 0.0f},
        {"arm_gripper", 60.0f},
    }, 1200);
}

bool FeetechArm::grab() {
    if (run_pose("grab_down", 1000)) {
        bool ok = true;
        if (has_pose("grab_close")) ok = run_pose("grab_close", 700) && ok;
        if (has_pose("lift")) ok = run_pose("lift", 1000) && ok;
        return ok;
    }
    bool ok = true;
    ok = write_pose({
        {"arm_shoulder_pan", -12.0f},
        {"arm_shoulder_lift", -50.0f},
        {"arm_elbow_flex", 40.0f},
        {"arm_wrist_flex", 80.0f},
        {"arm_gripper", 60.0f},
    }, 1000) && ok;
    ok = write_pose({{"arm_gripper", 5.0f}}, 700) && ok;
    ok = show() && ok;
    return ok;
}

bool FeetechArm::release() {
    if (run_pose("release", 700)) return true;
    return write_pose({{"arm_gripper", 60.0f}}, 700);
}

bool FeetechArm::show() {
    if (run_pose("lift", 1000)) return true;
    return write_pose({
        {"arm_shoulder_pan", 0.0f},
        {"arm_shoulder_lift", -31.7f},
        {"arm_elbow_flex", 27.7f},
        {"arm_wrist_flex", 80.0f},
        {"arm_gripper", 5.0f},
    }, 1000);
}

bool FeetechArm::stop_torque() {
    bool ok = true;
    for (const auto& kv : joints_) ok = bus_.enable_torque(kv.second.id, false) && ok;
    return ok;
}
