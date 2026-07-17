#include "feetech_arm.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
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
    auto write_config_u8 = [&](const Joint& joint, uint8_t addr, uint8_t value) {
        if (joint.gripper) {
            return bus_.write_u8_allow_status(
                joint.id, addr, value, feetech::STATUS_ERROR_OVERLOAD);
        }
        return bus_.write_u8(joint.id, addr, value);
    };

    // Disable every arm joint first. Configuring and re-enabling one joint at
    // a time can let an early joint chase a stale power-on goal while the
    // remaining joints are still being prepared.
    for (const auto& kv : joints_) {
        int id = kv.second.id;
        const bool torque_off_ok = kv.second.gripper
            ? write_config_u8(kv.second, feetech::reg::TORQUE_ENABLE, 0)
            : bus_.enable_torque(id, false);
        if (!torque_off_ok) {
            last_error_ = "configure " + kv.first + " id=" + std::to_string(id) +
                          " torque-off failed: " + bus_.last_error();
            fprintf(stderr, "[FeetechArm] configure %s id=%d torque-off failed: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
            return false;
        }
    }

    std::vector<std::pair<int, int>> hold_goals;
    for (const auto& kv : joints_) {
        int id = kv.second.id;
        if (!write_config_u8(kv.second, feetech::reg::OPERATING_MODE,
                             static_cast<uint8_t>(feetech::OperatingMode::POSITION))) {
            last_error_ = "configure " + kv.first + " id=" + std::to_string(id) +
                          " position-mode failed: " + bus_.last_error();
            fprintf(stderr, "[FeetechArm] configure %s id=%d position-mode failed: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
            return false;
        }
        if (!write_config_u8(kv.second, feetech::reg::P_COEFFICIENT, 16)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d P coefficient skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }
        if (!write_config_u8(kv.second, feetech::reg::I_COEFFICIENT, 0)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d I coefficient skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }
        if (!write_config_u8(kv.second, feetech::reg::D_COEFFICIENT, 32)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d D coefficient skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }
        if (!write_config_u8(kv.second, feetech::reg::ACCELERATION, 80)) {
            fprintf(stderr, "[FeetechArm] configure %s id=%d acceleration skipped: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
        }

        int current_raw = 0;
        bool read_ok = false;
        for (int attempt = 0; attempt < 4 && !read_ok; attempt++) {
            if (kv.second.gripper) {
                uint8_t status_error = 0;
                read_ok = bus_.read_u16_allow_status(
                    id, feetech::reg::PRESENT_POSITION, current_raw, true,
                    feetech::STATUS_ERROR_OVERLOAD, status_error);
            } else {
                read_ok = bus_.read_u16(id, feetech::reg::PRESENT_POSITION,
                                        current_raw, true);
            }
            if (!read_ok && attempt < 3) usleep(20000);
        }
        if (!read_ok) {
            last_error_ = "configure " + kv.first + " id=" + std::to_string(id) +
                          " current-position read failed: " + bus_.last_error();
            fprintf(stderr, "[FeetechArm] %s\n", last_error_.c_str());
            return false;
        }
        hold_goals.push_back({id, current_raw});
    }

    // Seed every goal with its measured position while torque is still off.
    // Enabling torque now holds the current pose instead of snapping to an old
    // goal retained by the servo.
    if (!bus_.sync_write_u16(feetech::reg::GOAL_POSITION, hold_goals, true)) {
        last_error_ = "configure current-position hold failed: " + bus_.last_error();
        fprintf(stderr, "[FeetechArm] %s\n", last_error_.c_str());
        return false;
    }

    for (const auto& kv : joints_) {
        int id = kv.second.id;
        const bool torque_on_ok = kv.second.gripper
            ? write_config_u8(kv.second, feetech::reg::TORQUE_ENABLE, 1)
            : bus_.enable_torque(id, true);
        if (!torque_on_ok) {
            last_error_ = "configure " + kv.first + " id=" + std::to_string(id) +
                          " torque-on failed: " + bus_.last_error();
            fprintf(stderr, "[FeetechArm] configure %s id=%d torque-on failed: %s\n",
                    kv.first.c_str(), id, bus_.last_error().c_str());
            for (const auto& rollback : joints_) {
                bus_.enable_torque(rollback.second.id, false);
            }
            return false;
        }
    }
    fprintf(stderr, "[FeetechArm] torque enabled at current positions; no startup jump\n");
    last_error_.clear();
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
    return move_degrees_slow({{name, deg}}, 0);
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
    return get_joint_deg_impl(name, deg, 0, nullptr);
}

bool FeetechArm::get_gripper_deg_allow_overload(float& deg,
                                                 bool& gripper_overloaded) {
    uint8_t status_error = 0;
    const bool ok = get_joint_deg_impl("arm_gripper", deg,
                                       feetech::STATUS_ERROR_OVERLOAD,
                                       &status_error);
    gripper_overloaded = ok &&
        (status_error & feetech::STATUS_ERROR_OVERLOAD) != 0;
    return ok;
}

bool FeetechArm::get_joint_deg_impl(const std::string& name, float& deg,
                                    uint8_t allowed_error_mask,
                                    uint8_t* status_error) {
    auto it = joints_.find(name);
    if (it == joints_.end()) {
        last_error_ = "unknown joint: " + name;
        return false;
    }
    int raw = 0;
    static const int kReadAttempts = 4;
    for (int attempt = 1; attempt <= kReadAttempts; attempt++) {
        uint8_t error = 0;
        if (bus_.read_u16_allow_status(
                it->second.id, feetech::reg::PRESENT_POSITION, raw, true,
                allowed_error_mask, error)) {
            if (attempt > 1) {
                fprintf(stderr, "[FeetechArm] read joint %s id=%d recovered on attempt %d\n",
                        name.c_str(), it->second.id, attempt);
            }
            if (status_error) *status_error = error;
            deg = raw_to_deg(it->second, raw);
            return true;
        }
        if (attempt < kReadAttempts) usleep(20000);
    }
    last_error_ = "read joint " + name + " id=" + std::to_string(it->second.id) +
                  " position failed: " + bus_.last_error();
    fprintf(stderr, "[FeetechArm] %s\n", last_error_.c_str());
    return false;
}

bool FeetechArm::get_joint_degs(std::map<std::string, float>& positions) {
    return get_joint_degs_impl(positions, false, nullptr);
}

bool FeetechArm::get_joint_degs_allow_gripper_overload(
    std::map<std::string, float>& positions, bool& gripper_overloaded) {
    gripper_overloaded = false;
    return get_joint_degs_impl(positions, true, &gripper_overloaded);
}

bool FeetechArm::get_joint_degs_impl(std::map<std::string, float>& positions,
                                     bool allow_gripper_overload,
                                     bool* gripper_overloaded) {
    positions.clear();
    bool ok = true;
    for (const auto& kv : joints_) {
        float deg = 0.0f;
        if (allow_gripper_overload && kv.second.gripper) {
            uint8_t status_error = 0;
            if (get_joint_deg_impl(kv.first, deg,
                                   feetech::STATUS_ERROR_OVERLOAD,
                                   &status_error)) {
                positions[kv.first] = deg;
                if (gripper_overloaded &&
                    (status_error & feetech::STATUS_ERROR_OVERLOAD)) {
                    *gripper_overloaded = true;
                }
            } else {
                ok = false;
            }
        } else if (get_joint_deg(kv.first, deg)) {
            positions[kv.first] = deg;
        } else {
            ok = false;
        }
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

bool FeetechArm::move_degrees_slow(const std::map<std::string, float>& pose, int settle_ms) {
    if (pose.empty()) {
        last_error_ = "slow move failed: empty pose";
        return false;
    }

    constexpr float kArmStepDeg = 0.75f;       // 15 deg/s at 20 Hz
    constexpr float kGripperStepDeg = 2.0f;   // gripper may move faster
    constexpr int kPeriodUs = 50000;
    constexpr int kMaxTicks = 500;             // 25 seconds maximum

    fprintf(stderr,
            "[FeetechArm] slow move: arm<=%.1f deg/s gripper<=%.1f deg/s\n",
            kArmStepDeg * 20.0f, kGripperStepDeg * 20.0f);

    std::map<std::string, float> positions;
    if (!get_joint_degs(positions)) return false;
    std::map<std::string, float> commanded;
    for (const auto& target : pose) {
        auto joint = joints_.find(target.first);
        auto actual = positions.find(target.first);
        if (joint == joints_.end() || actual == positions.end()) {
            last_error_ = "slow move failed: unknown joint " + target.first;
            return false;
        }
        if (target.second < joint->second.min_deg || target.second > joint->second.max_deg) {
            last_error_ = "slow move failed: target outside joint range for " + target.first;
            return false;
        }
        commanded[target.first] = actual->second;
    }

    for (int tick = 0; tick < kMaxTicks; tick++) {
        std::map<std::string, float> next;
        bool command_reached = true;
        for (const auto& target : pose) {
            auto joint = joints_.find(target.first);
            float error = target.second - commanded[target.first];
            float max_step = joint->second.gripper ? kGripperStepDeg : kArmStepDeg;
            if (std::abs(error) > max_step) {
                command_reached = false;
                error = std::max(-max_step, std::min(max_step, error));
            }
            commanded[target.first] += error;
            next[target.first] = commanded[target.first];
        }
        if (!write_pose(next, 0)) return false;

        if (!get_joint_degs(positions)) return false;
        bool feedback_reached = command_reached;
        for (const auto& target : pose) {
            auto joint = joints_.find(target.first);
            // Match the real arm's settled feedback accuracy. Tighter bounds
            // make the controller push continuously against normal static
            // load without improving the visible pose.
            const float tolerance = joint->second.gripper ? 12.0f : 3.0f;
            if (std::abs(target.second - positions[target.first]) > tolerance) {
                feedback_reached = false;
            }
        }
        if (feedback_reached) {
            if (settle_ms > 0) usleep(settle_ms * 1000);
            fprintf(stderr, "[FeetechArm] slow move complete in %.2f s\n",
                    (tick + 1) * 0.05f);
            last_error_.clear();
            return true;
        }
        usleep(kPeriodUs);
    }

    std::ostringstream error;
    error << "slow move timed out, residuals=";
    bool first = true;
    for (const auto& target : pose) {
        if (!first) error << ',';
        error << target.first << ':' << (target.second - positions[target.first]);
        first = false;
    }
    last_error_ = error.str();
    fprintf(stderr, "[FeetechArm] %s\n", last_error_.c_str());
    return false;
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

bool FeetechArm::move_raw_pose_slow(const ArmRawPose& pose, int settle_ms) {
    std::map<std::string, float> degrees;
    for (const auto& kv : pose.joints) {
        auto joint = joints_.find(kv.first);
        if (joint == joints_.end()) continue;
        degrees[kv.first] = raw_to_deg(joint->second, kv.second);
    }
    return move_degrees_slow(degrees, settle_ms);
}

bool FeetechArm::run_pose(const std::string& name, int settle_ms) {
    const ArmRawPose* pose = poses_.get(name);
    if (!pose) {
        last_error_ = "unknown arm pose: " + name;
        return false;
    }
    return move_raw_pose_slow(*pose, settle_ms);
}

bool FeetechArm::grab_pos() {
    if (has_pose("home")) return run_pose("home", 1200);
    return move_degrees_slow({
        {"arm_shoulder_pan", 0.0f},
        {"arm_shoulder_lift", -31.7f},
        {"arm_elbow_flex", 27.7f},
        {"arm_wrist_flex", 80.0f},
        {"arm_wrist_roll", 0.0f},
        {"arm_gripper", 10.0f},
    }, 1200);
}

bool FeetechArm::release_pos() {
    if (has_pose("release_pos")) return run_pose("release_pos", 1200);
    return move_degrees_slow({
        {"arm_shoulder_pan", -12.0f},
        {"arm_shoulder_lift", -55.0f},
        {"arm_elbow_flex", 45.0f},
        {"arm_wrist_flex", 80.0f},
        {"arm_wrist_roll", 0.0f},
        {"arm_gripper", 60.0f},
    }, 1200);
}

bool FeetechArm::grab() {
    if (has_pose("grab_down")) {
        if (!run_pose("grab_down", 1000)) return false;
        bool ok = true;
        if (has_pose("grab_close")) ok = run_pose("grab_close", 700) && ok;
        if (has_pose("lift")) ok = run_pose("lift", 1000) && ok;
        return ok;
    }
    bool ok = true;
    ok = move_degrees_slow({
        {"arm_shoulder_pan", -12.0f},
        {"arm_shoulder_lift", -50.0f},
        {"arm_elbow_flex", 40.0f},
        {"arm_wrist_flex", 80.0f},
        {"arm_gripper", 60.0f},
    }, 1000) && ok;
    ok = move_degrees_slow({{"arm_gripper", 5.0f}}, 700) && ok;
    ok = show() && ok;
    return ok;
}

bool FeetechArm::release() {
    if (has_pose("release")) return run_pose("release", 700);
    return move_degrees_slow({{"arm_gripper", 60.0f}}, 700);
}

bool FeetechArm::show() {
    if (has_pose("lift")) return run_pose("lift", 1000);
    return move_degrees_slow({
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
