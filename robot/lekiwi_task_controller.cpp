#include "lekiwi_task_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace {
static int clamp_speed(int v) {
    return std::max(-100, std::min(100, v));
}

static LeKiwiMoveController::Command diff_drive_cmd(const char* label, int left, int right) {
    LeKiwiMoveController::Command cmd;
    cmd.left_speed = clamp_speed(left);
    cmd.right_speed = clamp_speed(right);
    cmd.idle = (cmd.left_speed == 0 && cmd.right_speed == 0);
    cmd.label = label;
    return cmd;
}

static void solve_inverse_kinematics(float x,
                                     float y,
                                     float& shoulder_lift,
                                     float& elbow_flex) {
    const float l1 = 0.1159f;
    const float l2 = 0.1350f;
    const float theta1_offset = std::atan2(0.028f, 0.11257f);
    const float theta2_offset = std::atan2(0.0052f, 0.1349f) + theta1_offset;

    float r = std::sqrt(x * x + y * y);
    const float r_max = l1 + l2;
    if (r > r_max && r > 0.0f) {
        float scale = r_max / r;
        x *= scale;
        y *= scale;
        r = r_max;
    }
    const float r_min = std::abs(l1 - l2);
    if (r < r_min && r > 0.0f) {
        float scale = r_min / r;
        x *= scale;
        y *= scale;
        r = r_min;
    }

    float cos_theta2 = -(r * r - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    cos_theta2 = std::max(-1.0f, std::min(1.0f, cos_theta2));
    float theta2 = (float)M_PI - std::acos(cos_theta2);

    float beta = std::atan2(y, x);
    float gamma = std::atan2(l2 * std::sin(theta2), l1 + l2 * std::cos(theta2));
    float theta1 = beta + gamma;

    float joint2 = theta1 + theta1_offset;
    float joint3 = theta2 + theta2_offset;
    joint2 = std::max(-0.1f, std::min(3.45f, joint2));
    joint3 = std::max(-0.2f, std::min((float)M_PI, joint3));

    shoulder_lift = 90.0f - joint2 * 180.0f / (float)M_PI;
    elbow_flex = joint3 * 180.0f / (float)M_PI - 90.0f;
}

static void solve_forward_kinematics(float shoulder_lift,
                                     float elbow_flex,
                                     float& x,
                                     float& y) {
    const float l1 = 0.1159f;
    const float l2 = 0.1350f;
    const float theta1_offset = std::atan2(0.028f, 0.11257f);
    const float theta2_offset = std::atan2(0.0052f, 0.1349f) + theta1_offset;
    const float theta1 = (90.0f - shoulder_lift) * (float)M_PI / 180.0f - theta1_offset;
    const float theta2 = (elbow_flex + 90.0f) * (float)M_PI / 180.0f - theta2_offset;
    x = l1 * std::cos(theta1) + l2 * std::cos(theta1 - theta2);
    y = l1 * std::sin(theta1) + l2 * std::sin(theta1 - theta2);
}

struct ResolvedGrabTarget {
    float x = 0.0f;
    float y = 0.0f;
    float pre_y = 0.0f;
    float pan = 0.0f;
    float pre_wrist = 0.0f;
    float grab_wrist = 0.0f;
};

struct ResolvedPlaceTarget {
    float release_x = 0.0f;
    float release_y = 0.0f;
    float approach_x = 0.0f;
    float approach_y = 0.0f;
    float release_pan = 0.0f;
    float release_shoulder = 0.0f;
    float release_elbow = 0.0f;
    float release_wrist = 0.0f;
    float release_roll = 0.0f;
    float approach_pan = 0.0f;
    float approach_shoulder = 0.0f;
    float approach_elbow = 0.0f;
    float approach_wrist = 0.0f;
    float approach_roll = 0.0f;
};

// Fixed safe pose measured from the previously validated high/retracted
// approach. It must not change when the user tunes the final release pose.
constexpr float kPlaceApproachPanDeg = 0.0f;
constexpr float kPlaceApproachShoulderDeg = -4.1f;
constexpr float kPlaceApproachElbowDeg = -52.8f;
constexpr float kPlaceApproachWristDeg = 80.0f;
constexpr float kPlaceApproachRollDeg = 0.0f;
constexpr int kPlaceSettleMs = 500;

static ResolvedGrabTarget resolve_grab_target(const LeKiwiPickConfig& config) {
    float reference_x = 0.0f;
    float reference_y = 0.0f;
    solve_forward_kinematics(config.grab_id2_deg, config.grab_id3_deg,
                             reference_x, reference_y);
    const float forward = reference_x + config.grab_forward_offset_cm / 100.0f;
    const float lateral = config.grab_lateral_offset_cm / 100.0f;

    ResolvedGrabTarget target;
    target.x = std::sqrt(forward * forward + lateral * lateral);
    target.y = reference_y + config.grab_height_offset_cm / 100.0f;
    target.pre_y = target.y + config.pre_grab_clearance_m;
    target.pan = config.grab_id1_deg +
                 std::atan2(lateral, std::max(0.01f, forward)) *
                 180.0f / (float)M_PI;
    target.grab_wrist = config.grab_id4_deg + config.grab_pitch_offset_deg;

    // Keep PRE_GRAB comfortable, but do not impose a fixed total pitch on the
    // whole path. The recorded ID4 value remains the exact final grab angle.
    float pre_shoulder = 0.0f;
    float pre_elbow = 0.0f;
    solve_inverse_kinematics(target.x, target.pre_y, pre_shoulder, pre_elbow);
    const float final_pitch = config.grab_id2_deg + config.grab_id3_deg +
                              target.grab_wrist;
    const float ideal_pre_wrist = final_pitch - pre_shoulder - pre_elbow;
    target.pre_wrist = std::max(-80.0f, std::min(80.0f, ideal_pre_wrist));
    return target;
}

static ResolvedPlaceTarget resolve_place_target(const LeKiwiPickConfig& config) {
    ResolvedPlaceTarget target;
    target.release_pan = config.place_id1_deg;
    target.release_shoulder = config.place_id2_deg;
    target.release_elbow = config.place_id3_deg;
    target.release_wrist = config.place_id4_deg;
    target.release_roll = config.place_id5_deg;
    target.approach_pan = kPlaceApproachPanDeg;
    target.approach_shoulder = kPlaceApproachShoulderDeg;
    target.approach_elbow = kPlaceApproachElbowDeg;
    target.approach_wrist = kPlaceApproachWristDeg;
    target.approach_roll = kPlaceApproachRollDeg;
    solve_forward_kinematics(target.release_shoulder, target.release_elbow,
                             target.release_x, target.release_y);
    solve_forward_kinematics(target.approach_shoulder, target.approach_elbow,
                             target.approach_x, target.approach_y);
    return target;
}

static bool validate_pick_target(const LeKiwiPickConfig& config,
                                 const ResolvedGrabTarget& target,
                                 std::string& error) {
    constexpr float kArmLimit = 85.0f;
    constexpr float kWristLimit = 80.0f;
    constexpr float kL1 = 0.1159f;
    constexpr float kL2 = 0.1350f;

    auto check_joint = [&](const char* name, float value, float limit) {
        if (!std::isfinite(value) || value < -limit || value > limit) {
            std::ostringstream os;
            os << name << '=' << std::fixed << std::setprecision(1) << value
               << " outside safe range [-" << limit << ',' << limit << ']';
            error = os.str();
            return false;
        }
        return true;
    };
    if (!check_joint("grab_id1", target.pan, kArmLimit) ||
        !check_joint("grab_id2", config.grab_id2_deg, kArmLimit) ||
        !check_joint("grab_id3", config.grab_id3_deg, kArmLimit) ||
        !check_joint("grab_id4", target.grab_wrist, kWristLimit) ||
        !check_joint("grab_id5", config.grab_id5_deg, kArmLimit) ||
        !check_joint("pre_grab_id4", target.pre_wrist, kWristLimit) ||
        !check_joint("carry_id1", config.carry_id1_deg, kArmLimit) ||
        !check_joint("carry_id2", config.carry_id2_deg, kArmLimit) ||
        !check_joint("carry_id3", config.carry_id3_deg, kArmLimit) ||
        !check_joint("carry_id4", config.carry_id4_deg, kWristLimit) ||
        !check_joint("carry_id5", config.carry_id5_deg, kArmLimit)) {
        return false;
    }

    auto check_point = [&](const char* segment, float x, float y) {
        const float radius = std::sqrt(x * x + y * y);
        if (!std::isfinite(radius) || radius < std::abs(kL1 - kL2) ||
            radius > kL1 + kL2) {
            std::ostringstream os;
            os << segment << " Cartesian radius " << std::fixed
               << std::setprecision(4) << radius << " m is unreachable";
            error = os.str();
            return false;
        }
        float shoulder = 0.0f;
        float elbow = 0.0f;
        solve_inverse_kinematics(x, y, shoulder, elbow);
        return check_joint("path shoulder", shoulder, kArmLimit) &&
               check_joint("path elbow", elbow, kArmLimit);
    };
    auto check_segment = [&](const char* name, float x0, float y0,
                             float x1, float y1) {
        for (int i = 0; i <= 100; i++) {
            const float t = i / 100.0f;
            if (!check_point(name, x0 + t * (x1 - x0), y0 + t * (y1 - y0))) {
                return false;
            }
        }
        return true;
    };
    return check_segment("HOME->PRE_GRAB", config.home_x, config.home_y,
                         target.x, target.pre_y) &&
           check_segment("PRE_GRAB->GRAB", target.x, target.pre_y,
                         target.x, target.y);
}

static bool validate_place_target(const LeKiwiPickConfig& config,
                                  const ResolvedPlaceTarget& target,
                                  std::string& error) {
    (void)config;
    constexpr float kArmLimit = 85.0f;
    constexpr float kWristLimit = 80.0f;
    constexpr float kMinRadius = std::abs(0.1159f - 0.1350f);
    constexpr float kMaxRadius = 0.1159f + 0.1350f;
    auto check_joint = [&](const char* name, float value, float limit) {
        if (!std::isfinite(value) || value < -limit || value > limit) {
            std::ostringstream os;
            os << name << '=' << std::fixed << std::setprecision(1) << value
               << " outside safe range [-" << limit << ',' << limit << ']';
            error = os.str();
            return false;
        }
        return true;
    };
    auto check_point = [&](const char* name, float x, float y) {
        const float radius = std::sqrt(x * x + y * y);
        if (!std::isfinite(radius) || radius < kMinRadius || radius > kMaxRadius) {
            std::ostringstream os;
            os << name << " radius " << std::fixed << std::setprecision(4)
               << radius << " m is unreachable";
            error = os.str();
            return false;
        }
        return true;
    };
    if (!check_point("place release", target.release_x, target.release_y) ||
        !check_point("place approach", target.approach_x, target.approach_y)) {
        return false;
    }
    if (!check_joint("place release pan", target.release_pan, kArmLimit) ||
        !check_joint("place release shoulder", target.release_shoulder, kArmLimit) ||
        !check_joint("place release elbow", target.release_elbow, kArmLimit) ||
        !check_joint("place release wrist", target.release_wrist, kWristLimit) ||
        !check_joint("place release roll", target.release_roll, kArmLimit) ||
        !check_joint("place approach pan", target.approach_pan, kArmLimit) ||
        !check_joint("place approach shoulder", target.approach_shoulder, kArmLimit) ||
        !check_joint("place approach elbow", target.approach_elbow, kArmLimit) ||
        !check_joint("place approach wrist", target.approach_wrist, kWristLimit) ||
        !check_joint("place approach roll", target.approach_roll, kArmLimit)) {
        return false;
    }
    if (target.release_y > target.approach_y - 0.005f) {
        error = "place release pose must be at least 0.5 cm below approach";
        return false;
    }
    // SMOOTH_POSE interpolates exact joint targets. Verify every intermediate
    // command and its real end-effector curve.
    for (int i = 0; i <= 100; i++) {
        const float t = i / 100.0f;
        const float pan = target.approach_pan +
            t * (target.release_pan - target.approach_pan);
        const float shoulder = target.approach_shoulder +
            t * (target.release_shoulder - target.approach_shoulder);
        const float elbow = target.approach_elbow +
            t * (target.release_elbow - target.approach_elbow);
        const float wrist = target.approach_wrist +
            t * (target.release_wrist - target.approach_wrist);
        const float roll = target.approach_roll +
            t * (target.release_roll - target.approach_roll);
        if (!check_joint("place path pan", pan, kArmLimit) ||
            !check_joint("place path shoulder", shoulder, kArmLimit) ||
            !check_joint("place path elbow", elbow, kArmLimit) ||
            !check_joint("place path wrist", wrist, kWristLimit) ||
            !check_joint("place path roll", roll, kArmLimit)) {
            return false;
        }

        float actual_x = 0.0f;
        float actual_y = 0.0f;
        solve_forward_kinematics(shoulder, elbow, actual_x, actual_y);
        const float max_endpoint_x = std::max(target.approach_x,
                                               target.release_x);
        if (actual_y < target.release_y - 0.005f ||
            actual_x > max_endpoint_x + 0.005f) {
            std::ostringstream os;
            os << "place joint path sweeps outside safe corridor at t="
               << std::fixed << std::setprecision(2) << t;
            error = os.str();
            return false;
        }
    }
    return true;
}
} // namespace

bool LeKiwiPickConfig::load(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    bool has_new_grab_pose = false;
    bool has_new_duration = false;
    bool has_new_settle = false;
    float legacy_grab_x = 0.1200f;
    float legacy_grab_y = -0.0600f;
    float legacy_pre_grab_y = 0.1211f;
    float legacy_pan = -12.0f;
    float legacy_pitch = 80.0f;
    bool has_legacy_grab = false;

    std::string line;
    while (std::getline(ifs, line)) {
        size_t comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);
        auto trim = [](std::string s) {
            const char* ws = " \t\r\n";
            size_t b = s.find_first_not_of(ws);
            if (b == std::string::npos) return std::string();
            size_t e = s.find_last_not_of(ws);
            return s.substr(b, e - b + 1);
        };
        key = trim(key);
        value = trim(value);
        if (key.empty() || value.empty()) continue;
        float v = std::strtof(value.c_str(), nullptr);

        if      (key == "grab_id1_deg") { grab_id1_deg = v; has_new_grab_pose = true; }
        else if (key == "grab_id2_deg") { grab_id2_deg = v; has_new_grab_pose = true; }
        else if (key == "grab_id3_deg") { grab_id3_deg = v; has_new_grab_pose = true; }
        else if (key == "grab_id4_deg") { grab_id4_deg = v; has_new_grab_pose = true; }
        else if (key == "grab_id5_deg") { grab_id5_deg = v; has_new_grab_pose = true; }
        else if (key == "grab_forward_offset_cm") grab_forward_offset_cm = v;
        else if (key == "grab_lateral_offset_cm") grab_lateral_offset_cm = v;
        else if (key == "grab_height_offset_cm") grab_height_offset_cm = v;
        else if (key == "grab_pitch_offset_deg") grab_pitch_offset_deg = v;
        else if (key == "carry_id1_deg") carry_id1_deg = v;
        else if (key == "carry_id2_deg") carry_id2_deg = v;
        else if (key == "carry_id3_deg") carry_id3_deg = v;
        else if (key == "carry_id4_deg") carry_id4_deg = v;
        else if (key == "carry_id5_deg") carry_id5_deg = v;
        else if (key == "place_id1_deg") place_id1_deg = v;
        else if (key == "place_id2_deg") place_id2_deg = v;
        else if (key == "place_id3_deg") place_id3_deg = v;
        else if (key == "place_id4_deg") place_id4_deg = v;
        else if (key == "place_id5_deg") place_id5_deg = v;
        else if (key == "bucket_stop_size_px")
            bucket_stop_size_px = std::max(1, (int)v);
        else if (key == "arm_speed_scale")
            arm_speed_scale = std::max(0.1f, std::min(1.0f, v));
        else if (key == "gripper_open_delta_deg") gripper_open_delta_deg = v;
        else if (key == "gripper_close_delta_deg") gripper_close_delta_deg = v;
        else if (key == "carry_duration_ms") {
            carry_duration_ms = std::max(50, (int)v);
            has_new_duration = true;
        }
        else if (key == "carry_settle_ms") {
            carry_settle_ms = std::max(0, (int)v);
            has_new_settle = true;
        }
        else if (key == "ball_stop_size_px") ball_stop_size_px = std::max(1, (int)v);
        else if (key == "ball_stop_tolerance_px") ball_stop_tolerance_px = std::max(0, (int)v);
        else if (key == "ball_center_tolerance_px") ball_center_tolerance_px = std::max(0, (int)v);
        else if (key == "ball_stable_frames") ball_stable_frames = std::max(1, (int)v);

        // Legacy names are accepted so an old root filesystem can still boot
        // once with a newly built binary. save() always writes the new format.
        else if (key == "home_x") home_x = v;
        else if (key == "home_y") home_y = v;
        else if (key == "pre_grab_y") { legacy_pre_grab_y = v; has_legacy_grab = true; }
        else if (key == "grab_x") { legacy_grab_x = v; has_legacy_grab = true; }
        else if (key == "grab_y") { legacy_grab_y = v; has_legacy_grab = true; }
        else if (key == "lift_x") lift_x = v;
        else if (key == "lift_y") lift_y = v;
        else if (key == "shoulder_pan_delta") { legacy_pan = v; has_legacy_grab = true; }
        else if (key == "gripper_open_delta") gripper_open_delta_deg = v;
        else if (key == "gripper_close_delta") gripper_close_delta_deg = v;
        else if (key == "wrist_pick_pitch") { legacy_pitch = v; has_legacy_grab = true; }
        else if (key == "wrist_lift_pitch") wrist_lift_pitch = v;
        else if (key == "carry_shoulder_pan") carry_id1_deg = v;
        else if (key == "carry_shoulder_lift") carry_id2_deg = v;
        else if (key == "carry_elbow_flex") carry_id3_deg = v;
        else if (key == "carry_wrist_flex") carry_id4_deg = v;
        else if (key == "carry_wrist_roll") carry_id5_deg = v;
        else if (key == "carry_duration_ticks" && !has_new_duration)
            carry_duration_ms = std::max(1, (int)v) * 50;
        else if (key == "carry_settle_ticks" && !has_new_settle)
            carry_settle_ms = std::max(0, (int)v) * 50;
        else if (key == "ball_target_size") ball_stop_size_px = std::max(1, (int)v);
        else if (key == "ball_size_tolerance") ball_stop_tolerance_px = std::max(0, (int)v);
        else if (key == "ball_center_tolerance") ball_center_tolerance_px = std::max(0, (int)v);
        else if (key == "stable_frames") ball_stable_frames = std::max(1, (int)v);
    }

    if (!has_new_grab_pose && has_legacy_grab) {
        solve_inverse_kinematics(legacy_grab_x, legacy_grab_y,
                                 grab_id2_deg, grab_id3_deg);
        grab_id1_deg = legacy_pan;
        grab_id4_deg = legacy_pitch - grab_id2_deg - grab_id3_deg;
        grab_id5_deg = 0.0f;
        pre_grab_clearance_m = legacy_pre_grab_y - legacy_grab_y;
    }
    return true;
}

bool LeKiwiPickConfig::save(const std::string& path) const {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << "# LeKiwi 抓球动作配置。修改后重启程序即可生效，不需要重新编译。\n";
    ofs << "# 姿态角度均为标定后的度数，不是舵机 0～4095 原始值。\n\n";
    ofs << "# 电机 ID：1肩部水平，2肩部抬升，3肘部，4腕部俯仰，5腕部旋转，\n";
    ofs << "#          6夹爪，7左轮，8后轮，9右轮。\n\n";
    ofs << std::fixed << std::setprecision(1);
    ofs << "# 夹球基础姿态：四项位置偏移均为0时，ID1～ID5到达这里后ID6闭合。\n";
    ofs << "grab_id1_deg = " << grab_id1_deg << "\n";
    ofs << "grab_id2_deg = " << grab_id2_deg << "\n";
    ofs << "grab_id3_deg = " << grab_id3_deg << "\n";
    ofs << "grab_id4_deg = " << grab_id4_deg << "\n";
    ofs << "grab_id5_deg = " << grab_id5_deg << "\n\n";
    ofs << "# 夹球位置快速修正。单位为厘米，建议每次只改0.5；0表示使用上面的基础姿态。\n";
    ofs << "# 正数向机器人前方伸远，负数向机器人方向收近。\n";
    ofs << "grab_forward_offset_cm = " << grab_forward_offset_cm << "\n";
    ofs << "# 正数向机器人左侧移动，负数向机器人右侧移动。\n";
    ofs << "grab_lateral_offset_cm = " << grab_lateral_offset_cm << "\n";
    ofs << "# 正数升高，负数降低。\n";
    ofs << "grab_height_offset_cm = " << grab_height_offset_cm << "\n";
    ofs << "# 只修正夹球时的腕部俯仰角，单位为度，建议每次改5。\n";
    ofs << "grab_pitch_offset_deg = " << grab_pitch_offset_deg << "\n\n";
    ofs << "# 收球姿态：夹住并抬离地面后，机械臂平滑收至该姿态，随后才允许车轮启动。\n";
    ofs << "carry_id1_deg = " << carry_id1_deg << "\n";
    ofs << "carry_id2_deg = " << carry_id2_deg << "\n";
    ofs << "carry_id3_deg = " << carry_id3_deg << "\n";
    ofs << "carry_id4_deg = " << carry_id4_deg << "\n";
    ofs << "carry_id5_deg = " << carry_id5_deg << "\n\n";
    ofs << "# 放球最终姿态：夹爪位于桶口上方，ID6保持夹球状态。\n";
    ofs << "place_id1_deg = " << place_id1_deg << "\n";
    ofs << "place_id2_deg = " << place_id2_deg << "\n";
    ofs << "place_id3_deg = " << place_id3_deg << "\n";
    ofs << "place_id4_deg = " << place_id4_deg << "\n";
    ofs << "place_id5_deg = " << place_id5_deg << "\n\n";
    ofs << "# 桶检测框较短边达到该像素值后停车；增大表示更靠近桶。\n";
    ofs << "bucket_stop_size_px = " << bucket_stop_size_px << "\n\n";
    ofs << "# ID6夹爪开合量。保持现有力度时不要修改。\n";
    ofs << "gripper_open_delta_deg = " << gripper_open_delta_deg << "\n";
    ofs << "gripper_close_delta_deg = " << gripper_close_delta_deg << "\n\n";
    ofs << "# 从抬球位置收至carry姿态所需时间，以及到位后的额外稳定时间，单位毫秒。\n";
    ofs << "# 时间越大动作越慢、更柔和；时间过小会使收臂显得突然。\n";
    ofs << "carry_duration_ms = " << carry_duration_ms << "\n";
    ofs << "carry_settle_ms = " << carry_settle_ms << "\n\n";
    ofs << "# 机械臂动作总速度倍率：0.3调试，0.5稳定运行，最大1.0。\n";
    ofs << "arm_speed_scale = " << arm_speed_scale << "\n\n";
    ofs << "# 视觉停车参数。检测框尺寸取网球框宽、高中的较大值。\n";
    ofs << "# 目标尺寸：增大表示靠球更近才停车；减小表示离球更远就停车。\n";
    ofs << "ball_stop_size_px = " << ball_stop_size_px << "\n";
    ofs << "# 尺寸容差：允许尺寸在目标值±该数值内。155±5即150～160像素。\n";
    ofs << "# 增大更容易停车但距离误差更大；减小更准确但可能前后反复调整。\n";
    ofs << "ball_stop_tolerance_px = " << ball_stop_tolerance_px << "\n";
    ofs << "# 球心左右容差：球中心距离画面目标中心不超过该像素值才算对正。\n";
    ofs << "# 增大更容易进入抓取但左右误差更大；减小对得更正但可能左右反复调整。\n";
    ofs << "ball_center_tolerance_px = " << ball_center_tolerance_px << "\n";
    ofs << "# 连续多少个检测帧同时满足距离和球心条件后才启动机械臂。\n";
    ofs << "# 增大更稳但等待更久；Starry约2.3fps时，2帧约需0.9秒。\n";
    ofs << "ball_stable_frames = " << ball_stable_frames << "\n";
    return true;
}

bool LeKiwiPickConfig::validate(std::string& error) const {
    error.clear();
    if (bucket_stop_size_px < 50 || bucket_stop_size_px > 470) {
        error = "bucket_stop_size_px must be in [50,470] for a 640x480 frame";
        return false;
    }
    const ResolvedGrabTarget grab = resolve_grab_target(*this);
    if (!validate_pick_target(*this, grab, error)) return false;
    const ResolvedPlaceTarget place = resolve_place_target(*this);
    return validate_place_target(*this, place, error);
}

LeKiwiMoveController::LeKiwiMoveController(int frame_width, int frame_height) {
    config_.load();
    update_geometry(frame_width, frame_height);
}

void LeKiwiMoveController::update_geometry(int frame_width, int frame_height) {
    frame_width_ = std::max(1, frame_width);
    frame_height_ = std::max(1, frame_height);
    int side = std::min(frame_width_, frame_height_);
    target_w_ = side / 3 - 10;
    target_h_ = side / 3;
    left_ = std::max(0, (frame_width_ - target_w_) / 2);
    right_ = std::min(frame_width_, left_ + target_w_);
    target_cx_ = left_ + target_w_ / 2;
    target_position_ = std::max(target_w_, target_h_);
    if (config_.ball_stop_size_px > 0) target_position_ = config_.ball_stop_size_px;
    bucket_target_position_ = std::max(1, config_.bucket_stop_size_px);
}

void LeKiwiMoveController::reset() {
    stable_count_ = 0;
    last_target_cx_ = -1;
}

void LeKiwiMoveController::remember_ball(int cx) {
    last_target_cx_ = std::max(0, std::min(frame_width_ - 1, cx));
}

const detection* LeKiwiMoveController::choose_best_ball(const std::vector<detection>& detections,
                                                        int target_cx) {
    if (detections.empty()) return nullptr;
    const detection* best = &detections[0];
    float best_score = -1.0e9f;
    for (const auto& det : detections) {
        int cx = (int)det.bbox.x;
        float area = det.bbox.w * det.bbox.h;
        float score = area - std::abs(target_cx - cx) * det.bbox.h * 0.5f;
        if (score > best_score) {
            best_score = score;
            best = &det;
        }
    }
    return best;
}

LeKiwiMoveController::Command LeKiwiMoveController::update_ball(
    const std::vector<detection>& detections) {
    const detection* best = choose_best_ball(detections, target_cx_);
    if (!best) return control_target(TargetBox{}, false);

    TargetBox target;
    target.visible = true;
    target.cx = (int)best->bbox.x;
    target.w = (int)best->bbox.w;
    target.h = (int)best->bbox.h;
    return control_target(target, false);
}

LeKiwiMoveController::Command LeKiwiMoveController::update_bucket(bool visible,
                                                                  int cx,
                                                                  int box_w,
                                                                  int box_h) {
    TargetBox target;
    target.visible = visible;
    target.cx = cx;
    target.w = box_w;
    target.h = box_h;
    return control_target(target, true);
}

LeKiwiMoveController::Command LeKiwiMoveController::control_target(const TargetBox& target,
                                                                   bool bucket) {
    if (!target.visible) {
        stable_count_ = 0;
        if (bucket) {
            return diff_drive_cmd("BUCKET_SEARCH", 12, -12);
        }
        if (last_target_cx_ >= 0) {
            int frame_center = target_cx_;
            return last_target_cx_ < frame_center
                ? diff_drive_cmd("SEARCH_LEFT", -30, 30)
                : diff_drive_cmd("SEARCH_RIGHT", 30, -30);
        }
        return diff_drive_cmd("IDLE", 0, 0);
    }

    last_target_cx_ = target.cx;
    int position = bucket ? std::min(target.w, target.h) : std::max(target.w, target.h);

    if (target.cx < left_) {
        stable_count_ = 0;
        int near = std::abs(target_cx_ - target.cx) < target_w_ * (bucket ? 1 : 3) / (bucket ? 1 : 2);
        int spd = near ? 12 : 30;
        return diff_drive_cmd(bucket ? "BUCKET_LEFT" : "BALL_LEFT", -spd, spd);
    }

    if (target.cx > right_) {
        stable_count_ = 0;
        int near = std::abs(target_cx_ - target.cx) < target_w_ * (bucket ? 1 : 3) / (bucket ? 1 : 2);
        int spd = near ? 12 : 30;
        return diff_drive_cmd(bucket ? "BUCKET_RIGHT" : "BALL_RIGHT", spd, -spd);
    }

    if (!bucket) {
        int center_error = target.cx - target_cx_;
        if (std::abs(center_error) > config_.ball_center_tolerance_px) {
            stable_count_ = 0;
            int spd = 8;
            return center_error < 0
                ? diff_drive_cmd("BALL_FINE_LEFT", -spd, spd)
                : diff_drive_cmd("BALL_FINE_RIGHT", spd, -spd);
        }
        if (position < target_position_ - config_.ball_stop_tolerance_px) {
            stable_count_ = 0;
            // Slow down after the detected ball reaches 80% of the target size.
            // Keep integer arithmetic so the threshold is deterministic on both
            // Linux and StarryOS.
            int spd = (position * 10 > target_position_ * 8) ? 8 : 35;
            return diff_drive_cmd("BALL_FORWARD", spd, spd);
        }
        if (position > target_position_ + config_.ball_stop_tolerance_px) {
            stable_count_ = 0;
            return diff_drive_cmd("BALL_BACKWARD", -12, -12);
        }
    } else {
        if (position < bucket_target_position_) {
            stable_count_ = 0;
            const int speed = position * 10 >= bucket_target_position_ * 9
                ? 12 : 25;
            return diff_drive_cmd("BUCKET_FORWARD", speed, speed);
        }
        const int too_close_margin = std::max(30,
                                              bucket_target_position_ / 5);
        if (position > bucket_target_position_ + too_close_margin) {
            stable_count_ = 0;
            return diff_drive_cmd("BUCKET_BACKWARD", -12, -12);
        }
    }

    stable_count_++;
    Command cmd = diff_drive_cmd(bucket ? "BUCKET_READY" : "BALL_READY", 0, 0);
    constexpr int kBucketStableFrames = 2;
    cmd.reached = stable_count_ >=
        (bucket ? kBucketStableFrames : config_.ball_stable_frames);
    return cmd;
}

LeKiwiArmController::LeKiwiArmController(FeetechArm& arm) : arm_(arm) {
    config_.load();
    targets_ = {
        {"arm_shoulder_pan", 0.0f},
        {"arm_shoulder_lift", -31.70f},
        {"arm_elbow_flex", 27.69f},
        {"arm_wrist_flex", 80.0f},
        {"arm_wrist_roll", 0.0f},
        {"arm_gripper", 10.0f},
    };
    last_error_.clear();
}

std::vector<LeKiwiArmController::Step> LeKiwiArmController::pick_sequence(
    const LeKiwiPickConfig& config) {
    const ResolvedGrabTarget target = resolve_grab_target(config);
    fprintf(stderr,
            "[LeKiwiArmController] grab ids=(%.1f,%.1f,%.1f,%.1f,%.1f) "
            "offset_cm=(forward=%.1f,lateral=%.1f,height=%.1f) pitch_offset=%.1f "
            "resolved=(pan=%.1f,x=%.4f,y=%.4f,pre_y=%.4f,pre_id4=%.1f,grab_id4=%.1f)\n",
            config.grab_id1_deg, config.grab_id2_deg, config.grab_id3_deg,
            config.grab_id4_deg, config.grab_id5_deg,
            config.grab_forward_offset_cm, config.grab_lateral_offset_cm,
            config.grab_height_offset_cm, config.grab_pitch_offset_deg,
            target.pan, target.x, target.y, target.pre_y,
            target.pre_wrist, target.grab_wrist);
    std::vector<Step> seq = {
        {Kind::HOME, "", config.home_x, config.home_y},
        {Kind::JOINT_TARGET, "arm_shoulder_pan", target.pan, 0.0f},
        {Kind::JOINT_DELTA, "arm_gripper", config.gripper_open_delta_deg, 0.0f},
        {Kind::JOINT_TARGET, "arm_wrist_roll", config.grab_id5_deg, 0.0f},
        {Kind::MOVE_TO, "pre_grab", target.x, target.pre_y, target.pre_wrist},
        {Kind::MOVE_TO, "grab", target.x, target.y, target.grab_wrist},
        {Kind::GAP, "", 0.0f, 0.0f},
        {Kind::JOINT_DELTA, "arm_gripper", config.gripper_close_delta_deg, 0.0f},
        {Kind::GAP, "", 0.0f, 0.0f},
        {Kind::MOVE_TO, "clear", target.x, target.pre_y, target.pre_wrist},
        {Kind::SMOOTH_POSE, "carry", 30.0f, (float)config.carry_settle_ms},
    };
    return seq;
}

std::vector<LeKiwiArmController::Step> LeKiwiArmController::put_sequence(
    const LeKiwiPickConfig& config) {
    (void)config;
    return {
        {Kind::SMOOTH_POSE, "place_approach", 30.0f, 300.0f},
        {Kind::SMOOTH_POSE, "place_release", 16.0f, (float)kPlaceSettleMs},
        {Kind::SMOOTH_POSE, "release_gripper", 40.0f, (float)kPlaceSettleMs},
        {Kind::SMOOTH_POSE, "place_approach", 20.0f, 300.0f},
        {Kind::SMOOTH_POSE, "carry", 30.0f, (float)config.carry_settle_ms},
        {Kind::SMOOTH_POSE, "close_gripper", 40.0f, 100.0f},
    };
}

bool LeKiwiArmController::build_named_pose(const std::string& name,
                                           std::map<std::string, float>& pose,
                                           std::string& error) const {
    pose.clear();
    if (name == "carry") {
        pose = {
            {"arm_shoulder_pan", config_.carry_id1_deg},
            {"arm_shoulder_lift", config_.carry_id2_deg},
            {"arm_elbow_flex", config_.carry_id3_deg},
            {"arm_wrist_flex", config_.carry_id4_deg},
            {"arm_wrist_roll", config_.carry_id5_deg},
        };
        return true;
    }
    if (name == "release_gripper") {
        auto current = observed_.find("arm_gripper");
        if (current == observed_.end()) {
            error = "missing gripper feedback";
            return false;
        }
        pose["arm_gripper"] = std::min(100.0f,
            current->second + config_.gripper_open_delta_deg);
        return true;
    }
    if (name == "close_gripper") {
        pose["arm_gripper"] = 10.0f;
        return true;
    }
    if (name == "place_approach" || name == "place_release") {
        const ResolvedPlaceTarget target = resolve_place_target(config_);
        const bool approach = name == "place_approach";
        pose = {
            {"arm_shoulder_pan", approach ? target.approach_pan
                                            : target.release_pan},
            {"arm_shoulder_lift", approach ? target.approach_shoulder
                                            : target.release_shoulder},
            {"arm_elbow_flex", approach ? target.approach_elbow
                                         : target.release_elbow},
            {"arm_wrist_flex", approach ? target.approach_wrist
                                         : target.release_wrist},
            {"arm_wrist_roll", approach ? target.approach_roll
                                          : target.release_roll},
        };
        return true;
    }
    error = "unknown smooth pose: " + name;
    return false;
}

void LeKiwiArmController::reset() {
    sequence_.clear();
    step_index_ = 0;
    active_ = false;
    done_ = false;
    failed_ = false;
    config_.load();
    gap_ticks_ = 0;
    step_initialized_ = false;
    step_hold_ticks_ = 0;
    current_x_ = config_.home_x;
    current_y_ = config_.home_y;
    pitch_ = config_.grab_id2_deg + config_.grab_id3_deg +
             config_.grab_id4_deg + config_.grab_pitch_offset_deg;
    move_start_distance_ = 0.0f;
    move_start_wrist_ = 0.0f;
    step_start_measured_x_ = 0.0f;
    step_start_measured_y_ = 0.0f;
    step_best_distance_ = 1.0e9f;
    step_last_progress_tick_ = 0;
    previous_gripper_position_ = 0.0f;
    gripper_stable_ticks_ = 0;
    have_previous_gripper_position_ = false;
    gripper_contact_ = false;
    carry_start_targets_.clear();
    smooth_start_targets_.clear();
    smooth_goal_targets_.clear();
    smooth_duration_ticks_ = 0;
    smooth_settle_ticks_ = 0;
    targets_ = {
        {"arm_shoulder_pan", 0.0f},
        {"arm_shoulder_lift", -31.70f},
        {"arm_elbow_flex", 27.69f},
        {"arm_wrist_flex", 80.0f},
        {"arm_wrist_roll", 0.0f},
        {"arm_gripper", 10.0f},
    };
    observed_.clear();
    commanded_.clear();
}

bool LeKiwiArmController::begin_pick() {
    LeKiwiPickConfig config;
    config.load();
    return begin_pick(config);
}

bool LeKiwiArmController::begin_pick(const LeKiwiPickConfig& config) {
    config_ = config;
    reset();
    config_ = config;
    const ResolvedGrabTarget target = resolve_grab_target(config_);
    std::string validation_error;
    if (!validate_pick_target(config_, target, validation_error)) {
        return fail("unsafe pick trajectory: " + validation_error);
    }
    sequence_ = pick_sequence(config_);
    active_ = true;
    return load_current_positions();
}

bool LeKiwiArmController::begin_put() {
    reset();
    const ResolvedPlaceTarget target = resolve_place_target(config_);
    std::string validation_error;
    if (!validate_place_target(config_, target, validation_error)) {
        return fail("unsafe place trajectory: " + validation_error);
    }
    fprintf(stderr,
            "[LeKiwiArmController] place approach_ids=(%.1f,%.1f,%.1f,%.1f,%.1f) "
            "release_ids=(%.1f,%.1f,%.1f,%.1f,%.1f) "
            "xy=(approach=%.4f,%.4f release=%.4f,%.4f)\n",
            target.approach_pan, target.approach_shoulder,
            target.approach_elbow, target.approach_wrist,
            target.approach_roll,
            config_.place_id1_deg, config_.place_id2_deg, config_.place_id3_deg,
            config_.place_id4_deg, config_.place_id5_deg,
            target.approach_x, target.approach_y,
            target.release_x, target.release_y);
    sequence_ = put_sequence(config_);
    active_ = true;
    return load_current_positions();
}

bool LeKiwiArmController::begin_stage(const std::string& stage) {
    reset();
    if (stage.rfind("place-", 0) == 0) {
        const ResolvedPlaceTarget target = resolve_place_target(config_);
        std::string validation_error;
        if (!validate_place_target(config_, target, validation_error)) {
            return fail("unsafe place trajectory: " + validation_error);
        }
    }
    if (stage == "home") {
        sequence_ = {{Kind::HOME, "home", config_.home_x, config_.home_y}};
    } else if (stage == "carry") {
        sequence_ = {{Kind::SMOOTH_POSE, "carry", 30.0f,
                      (float)config_.carry_settle_ms}};
    } else if (stage == "place-approach" || stage == "place-hover") {
        sequence_ = {{Kind::SMOOTH_POSE, "place_approach", 30.0f, 300.0f}};
    } else if (stage == "place-release") {
        sequence_ = {
            {Kind::SMOOTH_POSE, "place_approach", 30.0f, 300.0f},
            {Kind::SMOOTH_POSE, "place_release", 16.0f,
             (float)kPlaceSettleMs},
        };
    } else if (stage == "place-cycle") {
        const ResolvedPlaceTarget target = resolve_place_target(config_);
        std::string validation_error;
        if (!validate_place_target(config_, target, validation_error)) {
            return fail("unsafe place trajectory: " + validation_error);
        }
        sequence_ = put_sequence(config_);
    } else {
        return fail("unknown arm stage: " + stage);
    }
    active_ = true;
    return load_current_positions();
}

bool LeKiwiArmController::load_current_positions() {
    observed_.clear();
    commanded_.clear();
    std::map<std::string, float> positions;
    bool gripper_overloaded = false;
    if (!arm_.get_joint_degs_allow_gripper_overload(positions,
                                                     gripper_overloaded)) {
        return fail("load current positions failed: " + arm_.last_error());
    }
    for (const auto& position : positions) {
        observed_[position.first] = apply_joint_calibration(position.first,
                                                             position.second);
    }
    if (gripper_overloaded) {
        gripper_contact_ = true;
        fprintf(stderr,
                "[LeKiwiArmController] pre-existing gripper overload: "
                "holding position %.1f until the opening step\n",
                observed_["arm_gripper"]);
    }
    // Every sequence starts at the measured pose. Starting from a remembered
    // HOME target can otherwise create a large first-command jump, especially
    // when a put sequence begins while the arm is in CARRY.
    targets_ = observed_;
    commanded_ = observed_;
    forward_kinematics(observed_["arm_shoulder_lift"],
                       observed_["arm_elbow_flex"], current_x_, current_y_);
    last_error_.clear();
    return true;
}

bool LeKiwiArmController::fail(const std::string& message) {
    last_error_ = message;
    active_ = false;
    done_ = false;
    failed_ = true;
    fprintf(stderr, "[LeKiwiArmController] %s\n", last_error_.c_str());
    return false;
}

float LeKiwiArmController::apply_joint_calibration(const std::string& joint, float value) {
    struct Coeff { const char* joint; float offset; float scale; };
    static const Coeff coeffs[] = {
        {"arm_shoulder_pan", 6.0f, 1.0f},
        {"arm_shoulder_lift", 2.0f, 0.97f},
        {"arm_elbow_flex", 0.0f, 1.05f},
        {"arm_wrist_flex", 0.0f, 0.94f},
        {"arm_wrist_roll", 0.0f, 0.5f},
        {"arm_gripper", 0.0f, 1.0f},
    };
    for (const auto& c : coeffs) {
        if (joint == c.joint) return (value - c.offset) * c.scale;
    }
    return value;
}

float LeKiwiArmController::remove_joint_calibration(const std::string& joint, float value) {
    struct Coeff { const char* joint; float offset; float scale; };
    static const Coeff coeffs[] = {
        {"arm_shoulder_pan", 6.0f, 1.0f},
        {"arm_shoulder_lift", 2.0f, 0.97f},
        {"arm_elbow_flex", 0.0f, 1.05f},
        {"arm_wrist_flex", 0.0f, 0.94f},
        {"arm_wrist_roll", 0.0f, 0.5f},
        {"arm_gripper", 0.0f, 1.0f},
    };
    for (const auto& c : coeffs) {
        if (joint == c.joint) return value / c.scale + c.offset;
    }
    return value;
}

void LeKiwiArmController::inverse_kinematics(float x,
                                             float y,
                                             float& shoulder_lift,
                                             float& elbow_flex) {
    solve_inverse_kinematics(x, y, shoulder_lift, elbow_flex);
}

void LeKiwiArmController::forward_kinematics(float shoulder_lift,
                                             float elbow_flex,
                                             float& x,
                                             float& y) {
    solve_forward_kinematics(shoulder_lift, elbow_flex, x, y);
}

bool LeKiwiArmController::send_current_targets() {
    std::map<std::string, float> positions;
    const bool homing = step_index_ < sequence_.size() &&
                        sequence_[step_index_].kind == Kind::HOME;
    const bool gripper_opening = step_index_ < sequence_.size() &&
        ((sequence_[step_index_].kind == Kind::JOINT_DELTA &&
          sequence_[step_index_].joint == "arm_gripper" &&
          sequence_[step_index_].a > 0.0f) ||
         (sequence_[step_index_].kind == Kind::SMOOTH_POSE &&
          sequence_[step_index_].joint == "release_gripper"));
    const bool gripper_closing = step_index_ < sequence_.size() &&
        ((sequence_[step_index_].kind == Kind::JOINT_DELTA &&
          sequence_[step_index_].joint == "arm_gripper" &&
          sequence_[step_index_].a < 0.0f) ||
         (sequence_[step_index_].kind == Kind::SMOOTH_POSE &&
          sequence_[step_index_].joint == "close_gripper"));
    bool gripper_overloaded = false;
    const bool allow_gripper_overload = homing || gripper_opening ||
                                        gripper_closing || gripper_contact_;
    const bool read_ok = allow_gripper_overload
        ? arm_.get_joint_degs_allow_gripper_overload(positions, gripper_overloaded)
        : arm_.get_joint_degs(positions);
    if (!read_ok) {
        return fail("read current positions failed: " + arm_.last_error());
    }

    if (homing && gripper_overloaded) {
        const float current = apply_joint_calibration(
            "arm_gripper", positions["arm_gripper"]);
        gripper_contact_ = true;
        targets_["arm_gripper"] = current;
        commanded_["arm_gripper"] = current;
    } else if (gripper_overloaded && !gripper_opening &&
               !gripper_closing && !gripper_contact_) {
        return fail("unexpected gripper overload outside a contact/release step");
    }

    if (gripper_closing && !gripper_contact_) {
        const float current = apply_joint_calibration(
            "arm_gripper", positions["arm_gripper"]);
        const float residual = std::abs(targets_["arm_gripper"] - current);
        const bool stable = have_previous_gripper_position_ &&
            std::abs(current - previous_gripper_position_) < 0.5f;
        if (stable && residual > 12.0f) gripper_stable_ticks_++;
        else gripper_stable_ticks_ = 0;
        previous_gripper_position_ = current;
        have_previous_gripper_position_ = true;

        if (gripper_overloaded || gripper_stable_ticks_ >= 6) {
            gripper_contact_ = true;
            targets_["arm_gripper"] = current;
            commanded_["arm_gripper"] = current;
            fprintf(stderr,
                    "[LeKiwiArmController] gripper contact: position=%.1f "
                    "residual=%.1f source=%s; holding current position\n",
                    current, residual,
                    gripper_overloaded ? "overload-0x20" : "settled-residual");
        }
    }

    std::map<std::string, float> action;
    constexpr float kArmKp = 0.55f;
    constexpr float kGripperKp = 0.8f;
    constexpr float kFinalArmTargetThresholdDeg = 8.0f;
    constexpr float kHomeArmStepDeg = 1.25f;      // 25 deg/s at 20 Hz
    constexpr float kHomeGripperStepDeg = 2.0f;  // 40 deg/s at 20 Hz
    for (const auto& kv : targets_) {
        auto it = positions.find(kv.first);
        if (it == positions.end()) continue;
        float current = apply_joint_calibration(kv.first, it->second);
        if (!std::isfinite(current) ||
            (kv.first != "arm_gripper" && std::abs(current) > 98.0f)) {
            return fail("unsafe joint feedback for " + kv.first);
        }
        observed_[kv.first] = current;
        float error = kv.second - current;
        float next = current;
        if (homing) {
            const float max_step = kv.first == "arm_gripper"
                ? kHomeGripperStepDeg : kHomeArmStepDeg;
            auto previous = commanded_.find(kv.first);
            const float last_command = previous == commanded_.end()
                ? current : previous->second;
            const float command_error = kv.second - last_command;
            const float step = std::max(-max_step,
                                        std::min(max_step, command_error));
            next = last_command + step;
        } else {
            const float kp = kv.first == "arm_gripper" ? kGripperKp : kArmKp;
            next += kp * error;
        }
        if (!homing && kv.first != "arm_gripper" &&
            std::abs(error) < kFinalArmTargetThresholdDeg) {
            next = kv.second;
        }
        action[kv.first] = next;
        commanded_[kv.first] = next;
    }
    std::map<std::string, float> servo_action;
    for (const auto& kv : action) {
        servo_action[kv.first] = remove_joint_calibration(kv.first, kv.second);
    }
    if (!arm_.write_degrees(servo_action, 0)) {
        return fail("write current targets failed: " + arm_.last_error());
    }
    return true;
}

bool LeKiwiArmController::advance_step(const Step& step) {
    if (step.kind == Kind::GAP) {
        const int gap_ticks = step.a > 0.0f
            ? std::max(1, (int)std::ceil(step.a / 50.0f)) : 6;
        if (++gap_ticks_ >= gap_ticks) {
            gap_ticks_ = 0;
            step_hold_ticks_ = 0;
            return true;
        }
        return false;
    }

    gap_ticks_ = 0;
    if (step.kind == Kind::HOME) {
        step_hold_ticks_++;
        if (!step_initialized_) {
            targets_["arm_shoulder_pan"] = 0.0f;
            targets_["arm_shoulder_lift"] = -31.70f;
            targets_["arm_elbow_flex"] = 27.69f;
            targets_["arm_wrist_flex"] = 80.0f;
            targets_["arm_wrist_roll"] = 0.0f;
            if (!gripper_contact_) targets_["arm_gripper"] = 10.0f;
            step_initialized_ = true;
        }
        if (!send_current_targets()) return false;
        float arm_error = 0.0f;
        float gripper_error = 0.0f;
        for (const auto& target : targets_) {
            auto current = observed_.find(target.first);
            if (current == observed_.end()) continue;
            float error = std::abs(target.second - current->second);
            if (target.first == "arm_gripper") gripper_error = error;
            else arm_error += error;
        }
        if (arm_error < 5.0f && gripper_error < 12.0f) {
            current_x_ = step.a;
            current_y_ = step.b;
            return true;
        }
        if (step_hold_ticks_ >= 400) {
            return fail("return to HOME timed out, arm error=" + std::to_string(arm_error) +
                        ", gripper error=" + std::to_string(gripper_error));
        }
        return false;
    }

    step_hold_ticks_++;

    if (step.kind == Kind::MOVE_TO) {
        float target_x = step.a;
        float target_y = step.b;
        float err_x = target_x - current_x_;
        float err_y = target_y - current_y_;
        float dist = std::sqrt(err_x * err_x + err_y * err_y);
        if (!step_initialized_) {
            move_start_distance_ = dist;
            move_start_wrist_ = targets_["arm_wrist_flex"];
            forward_kinematics(observed_["arm_shoulder_lift"],
                               observed_["arm_elbow_flex"],
                               step_start_measured_x_, step_start_measured_y_);
            step_best_distance_ = std::sqrt(
                (step.a - step_start_measured_x_) *
                (step.a - step_start_measured_x_) +
                (step.b - step_start_measured_y_) *
                (step.b - step_start_measured_y_));
            step_last_progress_tick_ = step_hold_ticks_;
            step_initialized_ = true;
        }
        float step_size = 0.0f;
        if (dist < 0.0005f) {
            step_size = 0.0f;
        } else if (dist > 0.05f) {
            step_size = 0.01f;
        } else {
            step_size = 0.01f * (dist / 0.05f);
            step_size = std::max(0.001f, std::min(step_size, dist));
        }
        if (dist > 1.0e-6f) {
            current_x_ += err_x / dist * step_size;
            current_y_ += err_y / dist * step_size;
        }
        float shoulder = 0.0f, elbow = 0.0f;
        inverse_kinematics(current_x_, current_y_, shoulder, elbow);
        targets_["arm_shoulder_lift"] = shoulder;
        targets_["arm_elbow_flex"] = elbow;
    } else if (step.kind == Kind::SMOOTH_POSE) {
        if (!step_initialized_) {
            std::string pose_error;
            if (!build_named_pose(step.joint, smooth_goal_targets_, pose_error)) {
                return fail("prepare " + step.joint + " failed: " + pose_error);
            }
            smooth_start_targets_.clear();
            float max_delta = 0.0f;
            bool gripper_only = smooth_goal_targets_.size() == 1 &&
                smooth_goal_targets_.find("arm_gripper") != smooth_goal_targets_.end();
            for (const auto& goal : smooth_goal_targets_) {
                auto current = observed_.find(goal.first);
                if (current == observed_.end()) {
                    return fail("prepare " + step.joint +
                                " failed: missing feedback for " + goal.first);
                }
                smooth_start_targets_[goal.first] = current->second;
                max_delta = std::max(max_delta,
                                     std::abs(goal.second - current->second));
            }
            const float scale = gripper_only ? 1.0f : config_.arm_speed_scale;
            const float speed = std::max(1.0f, step.a * scale);
            smooth_duration_ticks_ = std::max(4,
                (int)std::ceil(max_delta / speed * 20.0f));
            smooth_settle_ticks_ = std::max(0,
                (int)std::ceil(step.b / 50.0f));
            step_best_distance_ = max_delta;
            step_last_progress_tick_ = step_hold_ticks_;
            if (step.joint == "release_gripper") {
                gripper_contact_ = false;
                gripper_stable_ticks_ = 0;
                have_previous_gripper_position_ = false;
            }
            fprintf(stderr,
                    "[LeKiwiArmController] %s S-curve duration=%.2fs "
                    "settle=%.2fs speed<=%.1fdeg/s\n",
                    step.joint.c_str(), smooth_duration_ticks_ * 0.05f,
                    smooth_settle_ticks_ * 0.05f, speed);
            step_initialized_ = true;
        }
        float t = std::min(1.0f,
            step_hold_ticks_ / (float)std::max(1, smooth_duration_ticks_));
        const float smooth = t * t * t *
                             (10.0f + t * (-15.0f + 6.0f * t));
        for (const auto& goal : smooth_goal_targets_) {
            const float start = smooth_start_targets_[goal.first];
            targets_[goal.first] = start + smooth * (goal.second - start);
        }
    } else if (step.kind == Kind::CARRY) {
        if (!step_initialized_) {
            carry_start_targets_ = observed_;
            step_initialized_ = true;
        }
        int duration = config_.carry_duration_ticks();
        float t = std::min(1.0f, step_hold_ticks_ / (float)duration);
        float smooth = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
        const std::map<std::string, float> carry_targets = {
            {"arm_shoulder_pan", config_.carry_id1_deg},
            {"arm_shoulder_lift", config_.carry_id2_deg},
            {"arm_elbow_flex", config_.carry_id3_deg},
            {"arm_wrist_flex", config_.carry_id4_deg},
            {"arm_wrist_roll", config_.carry_id5_deg},
        };
        for (const auto& target : carry_targets) {
            auto start = carry_start_targets_.find(target.first);
            float start_value = start == carry_start_targets_.end()
                ? target.second : start->second;
            targets_[target.first] =
                start_value + smooth * (target.second - start_value);
        }
    } else if (!step_initialized_) {
        if (step.kind == Kind::JOINT_DELTA) {
            if (step.joint == "arm_gripper" && step.a > 0.0f) {
                // An opening command releases any object left from a previous
                // interrupted run and starts a fresh contact-detection cycle.
                gripper_contact_ = false;
                gripper_stable_ticks_ = 0;
                have_previous_gripper_position_ = false;
            }
            targets_[step.joint] += step.a;
        } else if (step.kind == Kind::JOINT_TARGET) {
            targets_[step.joint] = step.a;
        } else if (step.kind == Kind::WRIST_FLEX) {
            pitch_ = step.a;
        }
        step_initialized_ = true;
    }

    if (step.kind == Kind::CARRY) {
        // CARRY directly controls all arm joints, including the wrist.
    } else if (step.kind == Kind::MOVE_TO &&
               (step.joint == "pre_grab" || step.joint == "grab" ||
                step.joint == "clear")) {
        float remaining = std::sqrt((step.a - current_x_) * (step.a - current_x_) +
                                    (step.b - current_y_) * (step.b - current_y_));
        float progress = move_start_distance_ > 1.0e-6f
            ? 1.0f - remaining / move_start_distance_ : 1.0f;
        progress = std::max(0.0f, std::min(1.0f, progress));
        const float smooth = progress * progress * progress *
                             (10.0f + progress * (-15.0f + 6.0f * progress));
        targets_["arm_wrist_flex"] =
            move_start_wrist_ + smooth * (step.c - move_start_wrist_);
    } else if (step.kind == Kind::MOVE_TO && step.joint == "lift") {
        float final_shoulder = 0.0f, final_elbow = 0.0f;
        inverse_kinematics(step.a, step.b, final_shoulder, final_elbow);
        float final_wrist = -final_shoulder - final_elbow + config_.wrist_lift_pitch;
        final_wrist = std::max(-100.0f, std::min(100.0f, final_wrist));
        float remaining = std::sqrt((step.a - current_x_) * (step.a - current_x_) +
                                    (step.b - current_y_) * (step.b - current_y_));
        float progress = move_start_distance_ > 1.0e-6f
            ? 1.0f - remaining / move_start_distance_ : 1.0f;
        progress = std::max(0.0f, std::min(1.0f, progress));
        targets_["arm_wrist_flex"] =
            move_start_wrist_ + progress * (final_wrist - move_start_wrist_);
    } else if (step.kind == Kind::MOVE_TO || step.kind == Kind::WRIST_FLEX) {
        // Keep the wrist orientation coupled to shoulder/elbow motion. Do not
        // force a temporary wrist-only pose while the arm is still at HOME.
        float wrist = -targets_["arm_shoulder_lift"] - targets_["arm_elbow_flex"] + pitch_;
        targets_["arm_wrist_flex"] = std::max(-100.0f, std::min(100.0f, wrist));
    }

    if (!send_current_targets()) return false;

    if (step.kind == Kind::MOVE_TO) {
        float measured_x = 0.0f, measured_y = 0.0f;
        forward_kinematics(observed_["arm_shoulder_lift"],
                           observed_["arm_elbow_flex"],
                           measured_x, measured_y);
        const float distance = std::sqrt((step.a - measured_x) *
                                         (step.a - measured_x) +
                                         (step.b - measured_y) *
                                         (step.b - measured_y));
        if (distance + 0.001f < step_best_distance_) {
            step_best_distance_ = distance;
            step_last_progress_tick_ = step_hold_ticks_;
        }
    } else if (step.kind == Kind::SMOOTH_POSE) {
        float max_residual = 0.0f;
        for (const auto& goal : smooth_goal_targets_) {
            auto current = observed_.find(goal.first);
            if (current != observed_.end()) {
                max_residual = std::max(max_residual,
                    std::abs(goal.second - current->second));
            }
        }
        if (max_residual + 0.3f < step_best_distance_) {
            step_best_distance_ = max_residual;
            step_last_progress_tick_ = step_hold_ticks_;
        }
    }
    if (step.kind == Kind::JOINT_DELTA || step.kind == Kind::JOINT_TARGET ||
        step.kind == Kind::WRIST_FLEX) {
        int minimum_ticks = (step.joint == "arm_gripper") ? 8 : 4;
        if (step_hold_ticks_ < minimum_ticks) return false;
    }
    bool reached = step_reached(step);
    if (reached && step.kind == Kind::MOVE_TO && step.joint == "lift") {
        pitch_ = config_.wrist_lift_pitch;
    }
    int timeout_ticks = 100;
    if (step.kind == Kind::SMOOTH_POSE) {
        timeout_ticks = smooth_duration_ticks_ + smooth_settle_ticks_ + 60;
    }
    const bool still_progressing =
        step_hold_ticks_ - step_last_progress_tick_ < 40;
    if (!reached && step_hold_ticks_ >= timeout_ticks && !still_progressing) {
        if (step.kind == Kind::MOVE_TO && step.joint == "clear" &&
            gripper_contact_) {
            // The ball did not reach the minimum safe lift height. Keep the
            // base stopped, release it at the current low-speed arm pose, then
            // return HOME instead of terminating with a loaded gripper.
            sequence_.resize(step_index_ + 1);
            sequence_.push_back(
                {Kind::SMOOTH_POSE, "release_gripper", 40.0f, 300.0f});
            sequence_.push_back(
                {Kind::HOME, "home", config_.home_x, config_.home_y});
            fprintf(stderr,
                    "[LeKiwiArmController] clear stalled before safe height; "
                    "releasing ball and recovering HOME\n");
            return true;
        }
        std::ostringstream error;
        error << "step timed out: "
              << (step.joint.empty() ? step_kind_label(step.kind)
                                     : step.joint.c_str())
              << ", residuals=";
        bool first = true;
        for (const auto& target : targets_) {
            auto current = observed_.find(target.first);
            if (current == observed_.end()) continue;
            if (!first) error << ',';
            error << target.first << ':' << std::fixed << std::setprecision(1)
                  << (target.second - current->second);
            first = false;
        }
        return fail(error.str());
    }
    return reached;
}

bool LeKiwiArmController::step_reached(const Step& step) const {
    if (step.kind == Kind::SMOOTH_POSE) {
        if (step_hold_ticks_ < smooth_duration_ticks_ + smooth_settle_ticks_) {
            return false;
        }
        float max_error = 0.0f;
        const bool gripper_only = smooth_goal_targets_.size() == 1 &&
            smooth_goal_targets_.find("arm_gripper") != smooth_goal_targets_.end();
        for (const auto& goal : smooth_goal_targets_) {
            auto current = observed_.find(goal.first);
            if (current == observed_.end()) return false;
            max_error = std::max(max_error,
                                 std::abs(goal.second - current->second));
        }
        if (gripper_only) return max_error <= 12.0f;

        bool geometry_safe = true;
        if (step.joint == "place_approach" || step.joint == "place_release") {
            auto shoulder = smooth_goal_targets_.find("arm_shoulder_lift");
            auto elbow = smooth_goal_targets_.find("arm_elbow_flex");
            if (shoulder == smooth_goal_targets_.end() ||
                elbow == smooth_goal_targets_.end()) return false;
            float target_x = 0.0f, target_y = 0.0f;
            float measured_x = 0.0f, measured_y = 0.0f;
            forward_kinematics(shoulder->second, elbow->second,
                               target_x, target_y);
            forward_kinematics(observed_.at("arm_shoulder_lift"),
                               observed_.at("arm_elbow_flex"),
                               measured_x, measured_y);
            geometry_safe = std::sqrt((target_x - measured_x) *
                                      (target_x - measured_x) +
                                      (target_y - measured_y) *
                                      (target_y - measured_y)) <= 0.020f;
        }
        if (!geometry_safe) return false;
        if (max_error <= 8.0f) return true;

        // Loaded joints may settle with a larger angle residual even when the
        // end effector is already inside the safe task region. Accept that
        // stable state inside a hard envelope instead of summing small errors.
        const bool settled = step_hold_ticks_ - step_last_progress_tick_ >= 10;
        const float hard_limit = step.joint == "carry" ? 12.0f : 15.0f;
        return settled && max_error <= hard_limit;
    }
    if (step.kind == Kind::CARRY) {
        int minimum_ticks = config_.carry_duration_ticks() +
                            config_.carry_settle_ticks();
        if (step_hold_ticks_ < minimum_ticks) return false;
        float arm_error = 0.0f;
        for (const char* joint : {"arm_shoulder_pan", "arm_shoulder_lift",
                                  "arm_elbow_flex", "arm_wrist_flex",
                                  "arm_wrist_roll"}) {
            auto target = targets_.find(joint);
            auto current = observed_.find(joint);
            if (target == targets_.end() || current == observed_.end()) return false;
            arm_error += std::abs(target->second - current->second);
        }
        return arm_error < 8.0f;
    }
    if (step.kind == Kind::MOVE_TO) {
        if (std::abs(current_x_ - step.a) >= 0.002f ||
            std::abs(current_y_ - step.b) >= 0.002f) {
            return false;
        }
        float measured_x = 0.0f, measured_y = 0.0f;
        forward_kinematics(observed_.at("arm_shoulder_lift"),
                           observed_.at("arm_elbow_flex"),
                           measured_x, measured_y);
        if (step.joint == "clear" && gripper_contact_) {
            const float required_lift = std::max(
                0.060f, config_.pre_grab_clearance_m * 0.70f);
            const float lifted = measured_y - step_start_measured_y_;
            return lifted >= required_lift &&
                   std::abs(measured_x - step.a) <= 0.025f;
        }
        const float position_error = std::sqrt((measured_x - step.a) *
                                               (measured_x - step.a) +
                                               (measured_y - step.b) *
                                               (measured_y - step.b));
        if (position_error > 0.015f) return false;
        float max_arm_error = 0.0f;
        for (const char* joint : {"arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex"}) {
            auto target = targets_.find(joint);
            auto current = observed_.find(joint);
            if (target == targets_.end() || current == observed_.end()) return false;
            max_arm_error = std::max(max_arm_error,
                                     std::abs(target->second - current->second));
        }
        return max_arm_error <= 8.0f;
    }
    if (step.kind == Kind::JOINT_DELTA || step.kind == Kind::JOINT_TARGET ||
        step.kind == Kind::WRIST_FLEX) {
        auto target = targets_.find(step.joint);
        auto current = observed_.find(step.joint);
        if (target == targets_.end() || current == observed_.end()) return false;
        float tolerance = step.joint == "arm_gripper" ? 12.0f : 2.0f;
        if (std::abs(target->second - current->second) >= tolerance) return false;
        if (step.kind == Kind::WRIST_FLEX) {
            float arm_error = 0.0f;
            for (const char* joint : {"arm_shoulder_pan", "arm_shoulder_lift",
                                      "arm_elbow_flex", "arm_wrist_flex",
                                      "arm_wrist_roll"}) {
                auto arm_target = targets_.find(joint);
                auto arm_current = observed_.find(joint);
                if (arm_target == targets_.end() || arm_current == observed_.end()) return false;
                arm_error += std::abs(arm_target->second - arm_current->second);
            }
            return arm_error < 5.0f;
        }
        return true;
    }
    return true;
}

bool LeKiwiArmController::tick() {
    if (!active_ || done_ || failed_) return done_ && !failed_;
    if (step_index_ >= sequence_.size()) {
        active_ = false;
        done_ = true;
        return true;
    }

    bool advanced = advance_step(sequence_[step_index_]);
    if (advanced) {
        step_index_++;
        step_initialized_ = false;
        step_hold_ticks_ = 0;
    }
    if (step_index_ >= sequence_.size()) {
        active_ = false;
        done_ = true;
    }
    return !failed_;
}

bool LeKiwiArmController::verify_grab(float* gripper_pos) {
    float pos = 0.0f;
    bool gripper_overloaded = false;
    const bool ok = gripper_contact_
        ? arm_.get_gripper_deg_allow_overload(pos, gripper_overloaded)
        : arm_.get_joint_deg("arm_gripper", pos);
    if (!ok) {
        if (gripper_pos) *gripper_pos = 0.0f;
        return fail("verify gripper position failed: " + arm_.last_error());
    }
    if (gripper_pos) *gripper_pos = pos;
    return pos > 25.0f;
}

const char* LeKiwiArmController::step_kind_label(Kind kind) {
    switch (kind) {
        case Kind::HOME:        return "home";
        case Kind::MOVE_TO:     return "move_to";
        case Kind::JOINT_DELTA: return "joint_delta";
        case Kind::JOINT_TARGET:return "joint_target";
        case Kind::WRIST_FLEX:  return "wrist_flex";
        case Kind::CARRY:       return "carry";
        case Kind::SMOOTH_POSE: return "smooth_pose";
        case Kind::GAP:         return "gap";
    }
    return "unknown";
}

const char* LeKiwiArmController::current_step_label() const {
    if (step_index_ >= sequence_.size()) return "done";
    if ((sequence_[step_index_].kind == Kind::SMOOTH_POSE ||
         sequence_[step_index_].kind == Kind::MOVE_TO) &&
        !sequence_[step_index_].joint.empty()) {
        return sequence_[step_index_].joint.c_str();
    }
    return step_kind_label(sequence_[step_index_].kind);
}
