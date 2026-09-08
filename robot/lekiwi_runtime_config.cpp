#include "lekiwi_runtime_config.hpp"

#include "robot/lekiwi_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>

namespace {

struct PickValues {
    float grab_id1 = -18.0f;
    float grab_id2 = 37.7f;
    float grab_id3 = 20.0f;
    float grab_id4 = 40.0f;
    float grab_id5 = 0.0f;
    float grab_forward_cm = -0.5f;
    float grab_lateral_cm = -0.5f;
    float grab_height_cm = -1.0f;
    float grab_pitch_offset = 0.0f;
    float carry[5] = {-11.3f, -18.3f, -45.0f, 51.8f, 0.1f};
    float place[5] = {-14.3f, 20.4f, -25.7f, 70.0f, 0.0f};
    float gripper_open_delta = 60.0f;
    float gripper_close_delta = -60.0f;
    int motion_speed_level = 4;
    int ball_target = 155;
    int ball_tolerance = 5;
    int ball_center = 30;
    int ball_stable = 2;
    int bucket_target = 380;
    int bucket_center = 20;
    int bucket_stable = 3;
    int ball_far = 65;
    int ball_near = 20;
    int ball_reverse = 25;
    int ball_turn = 15;
    int ball_search = 35;
    int bucket_far = 70;
    int bucket_near = 25;
    int bucket_reverse = 25;
    int bucket_turn = 18;
};

struct GrabTarget {
    float x;
    float y;
    float pre_y;
    float pan;
    float pre_wrist;
    float grab_wrist;
};

static std::string trim(std::string value) {
    const char* whitespace = " \t\r\n";
    const size_t begin = value.find_first_not_of(whitespace);
    if (begin == std::string::npos) return {};
    const size_t end = value.find_last_not_of(whitespace);
    return value.substr(begin, end - begin + 1U);
}

static bool load_values(const std::string& path, PickValues& values,
                        std::string& error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        error = "cannot open pick configuration: " + path;
        return false;
    }

    std::map<std::string, float> parsed;
    std::string line;
    while (std::getline(input, line)) {
        const size_t comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        const size_t equals = line.find('=');
        if (equals == std::string::npos) continue;
        const std::string key = trim(line.substr(0, equals));
        const std::string text = trim(line.substr(equals + 1U));
        if (key.empty() || text.empty()) continue;
        char* end = nullptr;
        const float value = std::strtof(text.c_str(), &end);
        if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
            error = "invalid numeric value for " + key;
            return false;
        }
        parsed[key] = value;
    }

    auto set_float = [&](const char* key, float& target) {
        const auto found = parsed.find(key);
        if (found != parsed.end()) target = found->second;
    };
    auto set_int = [&](const char* key, int& target) {
        const auto found = parsed.find(key);
        if (found != parsed.end()) target = static_cast<int>(found->second);
    };
    set_float("grab_id1_deg", values.grab_id1);
    set_float("grab_id2_deg", values.grab_id2);
    set_float("grab_id3_deg", values.grab_id3);
    set_float("grab_id4_deg", values.grab_id4);
    set_float("grab_id5_deg", values.grab_id5);
    set_float("grab_forward_offset_cm", values.grab_forward_cm);
    set_float("grab_lateral_offset_cm", values.grab_lateral_cm);
    set_float("grab_height_offset_cm", values.grab_height_cm);
    set_float("grab_pitch_offset_deg", values.grab_pitch_offset);
    set_float("carry_id1_deg", values.carry[0]);
    set_float("carry_id2_deg", values.carry[1]);
    set_float("carry_id3_deg", values.carry[2]);
    set_float("carry_id4_deg", values.carry[3]);
    set_float("carry_id5_deg", values.carry[4]);
    set_float("place_id1_deg", values.place[0]);
    set_float("place_id2_deg", values.place[1]);
    set_float("place_id3_deg", values.place[2]);
    set_float("place_id4_deg", values.place[3]);
    set_float("place_id5_deg", values.place[4]);
    set_float("gripper_open_delta_deg", values.gripper_open_delta);
    set_float("gripper_close_delta_deg", values.gripper_close_delta);
    set_int("motion_speed_level", values.motion_speed_level);
    set_int("ball_stop_size_px", values.ball_target);
    set_int("ball_stop_tolerance_px", values.ball_tolerance);
    set_int("ball_center_tolerance_px", values.ball_center);
    set_int("ball_stable_frames", values.ball_stable);
    set_int("bucket_stop_size_px", values.bucket_target);
    set_int("bucket_center_tolerance_px", values.bucket_center);
    set_int("bucket_stable_frames", values.bucket_stable);
    return true;
}

static void apply_motion_profile(PickValues& values) {
    switch (values.motion_speed_level) {
    case 1:
        values.ball_far = values.bucket_far = 20;
        values.ball_near = values.bucket_near = 8;
        values.ball_reverse = values.bucket_reverse = 8;
        values.ball_turn = values.bucket_turn = 6;
        values.ball_search = 12;
        break;
    case 2:
        values.ball_far = values.bucket_far = 30;
        values.ball_near = values.bucket_near = 10;
        values.ball_reverse = values.bucket_reverse = 10;
        values.ball_turn = values.bucket_turn = 8;
        values.ball_search = 20;
        break;
    case 3:
        values.ball_far = values.bucket_far = 40;
        values.ball_near = values.bucket_near = 15;
        values.ball_reverse = values.bucket_reverse = 15;
        values.ball_turn = values.bucket_turn = 10;
        values.ball_search = 25;
        break;
    default:
        values.ball_far = 65;
        values.ball_near = 20;
        values.ball_reverse = 25;
        values.ball_turn = 15;
        values.ball_search = 35;
        values.bucket_far = 70;
        values.bucket_near = 25;
        values.bucket_reverse = 25;
        values.bucket_turn = 18;
        break;
    }
}

static void inverse_kinematics(float x, float y, float& shoulder, float& elbow) {
    constexpr float l1 = 0.1159f;
    constexpr float l2 = 0.1350f;
    const float theta1_offset = std::atan2(0.028f, 0.11257f);
    const float theta2_offset = std::atan2(0.0052f, 0.1349f) + theta1_offset;
    float radius = std::sqrt(x * x + y * y);
    const float maximum = l1 + l2;
    const float minimum = std::abs(l1 - l2);
    if (radius > maximum && radius > 0.0f) {
        const float scale = maximum / radius;
        x *= scale;
        y *= scale;
        radius = maximum;
    }
    if (radius < minimum && radius > 0.0f) {
        const float scale = minimum / radius;
        x *= scale;
        y *= scale;
        radius = minimum;
    }
    float cosine = -(radius * radius - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    cosine = std::max(-1.0f, std::min(1.0f, cosine));
    const float theta2 = static_cast<float>(M_PI) - std::acos(cosine);
    const float theta1 = std::atan2(y, x) +
        std::atan2(l2 * std::sin(theta2), l1 + l2 * std::cos(theta2));
    shoulder = 90.0f - (theta1 + theta1_offset) * 180.0f /
        static_cast<float>(M_PI);
    elbow = (theta2 + theta2_offset) * 180.0f /
        static_cast<float>(M_PI) - 90.0f;
}

static void forward_kinematics(float shoulder, float elbow, float& x, float& y) {
    constexpr float l1 = 0.1159f;
    constexpr float l2 = 0.1350f;
    const float theta1_offset = std::atan2(0.028f, 0.11257f);
    const float theta2_offset = std::atan2(0.0052f, 0.1349f) + theta1_offset;
    const float theta1 = (90.0f - shoulder) * static_cast<float>(M_PI) / 180.0f -
                         theta1_offset;
    const float theta2 = (elbow + 90.0f) * static_cast<float>(M_PI) / 180.0f -
                         theta2_offset;
    x = l1 * std::cos(theta1) + l2 * std::cos(theta1 - theta2);
    y = l1 * std::sin(theta1) + l2 * std::sin(theta1 - theta2);
}

static GrabTarget resolve_grab(const PickValues& values) {
    float reference_x = 0.0f;
    float reference_y = 0.0f;
    forward_kinematics(values.grab_id2, values.grab_id3, reference_x, reference_y);
    const float forward = reference_x + values.grab_forward_cm / 100.0f;
    const float lateral = values.grab_lateral_cm / 100.0f;
    GrabTarget target{};
    target.x = std::sqrt(forward * forward + lateral * lateral);
    target.y = reference_y + values.grab_height_cm / 100.0f;
    target.pre_y = target.y + 0.1811f;
    target.pan = values.grab_id1 + std::atan2(lateral, std::max(0.01f, forward)) *
                 180.0f / static_cast<float>(M_PI);
    target.grab_wrist = values.grab_id4 + values.grab_pitch_offset;
    float pre_shoulder = 0.0f;
    float pre_elbow = 0.0f;
    inverse_kinematics(target.x, target.pre_y, pre_shoulder, pre_elbow);
    const float final_pitch = values.grab_id2 + values.grab_id3 + target.grab_wrist;
    target.pre_wrist = std::max(-80.0f, std::min(80.0f,
        final_pitch - pre_shoulder - pre_elbow));
    return target;
}

static float remove_controller_correction(size_t joint, float value) {
    static constexpr float offsets[ROBOT_JOINT_COUNT] = {6.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static constexpr float scales[ROBOT_JOINT_COUNT] = {1.0f, 0.97f, 1.05f, 0.94f, 0.5f, 1.0f};
    return value / scales[joint] + offsets[joint];
}

static bool to_raw(const LekiwiCalibration& calibration, size_t joint,
                   float logical, uint16_t& raw, std::string& error) {
    static const char* names[ROBOT_JOINT_COUNT] = {
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll", "arm_gripper",
    };
    const JointCalibration* entry = calibration.get(names[joint]);
    if (entry == nullptr || entry->id != static_cast<int>(joint + 1U) ||
        entry->range_min < 0 || entry->range_max > 4095 ||
        entry->range_min >= entry->range_max) {
        error = std::string("invalid calibration for ") + names[joint];
        return false;
    }
    const float minimum = joint == 5U ? 0.0f : -100.0f;
    const float maximum = 100.0f;
    const float servo_value = std::max(minimum, std::min(maximum,
        remove_controller_correction(joint, logical)));
    const float ratio = (servo_value - minimum) / (maximum - minimum);
    const int value = static_cast<int>(entry->range_min +
        ratio * (entry->range_max - entry->range_min) + 0.5f);
    raw = static_cast<uint16_t>(std::max(entry->range_min,
        std::min(entry->range_max, value)));
    return true;
}

static bool set_pose(const LekiwiCalibration& calibration,
                     robot_runtime_config_v1& output, size_t pose,
                     const float values[ROBOT_JOINT_COUNT], std::string& error) {
    for (size_t joint = 0; joint < ROBOT_JOINT_COUNT; ++joint) {
        if (!to_raw(calibration, joint, values[joint],
                    output.poses[pose][joint], error)) {
            return false;
        }
    }
    return true;
}

static bool validate_values(const PickValues& values, const GrabTarget& grab,
                            std::string& error) {
    if (values.motion_speed_level < 1 || values.motion_speed_level > 4) {
        error = "motion_speed_level must be in [1,4]";
        return false;
    }
    if (values.ball_target < 50 || values.ball_target > 400 ||
        values.ball_tolerance < 0 || values.ball_tolerance > 100 ||
        values.ball_center < 0 || values.ball_center > 160 ||
        values.ball_stable < 1 || values.ball_stable > 30) {
        error = "ball control values are outside the safe range";
        return false;
    }
    if (values.bucket_target < 50 || values.bucket_target > 700 ||
        values.bucket_center < 5 || values.bucket_center > 100 ||
        values.bucket_stable < 1 || values.bucket_stable > 30) {
        error = "bucket control values are outside the safe range";
        return false;
    }
    const float radius = std::sqrt(grab.x * grab.x + grab.y * grab.y);
    const float pre_radius = std::sqrt(grab.x * grab.x + grab.pre_y * grab.pre_y);
    if (!std::isfinite(radius) || !std::isfinite(pre_radius) ||
        radius < 0.0191f || radius > 0.2509f ||
        pre_radius < 0.0191f || pre_radius > 0.2509f ||
        std::abs(grab.pan) > 85.0f || std::abs(grab.grab_wrist) > 80.0f ||
        std::abs(grab.pre_wrist) > 80.0f) {
        error = "resolved grab pose is outside the safe workspace";
        return false;
    }
    return true;
}

} // namespace

bool build_lekiwi_runtime_config(
    const std::string& calibration_path,
    const std::string& pick_config_path,
    robot_runtime_config_v1& output,
    std::string& error) {
    PickValues values;
    if (!load_values(pick_config_path, values, error)) return false;
    apply_motion_profile(values);

    LekiwiCalibration calibration;
    if (!calibration.load(calibration_path)) {
        error = calibration.last_error();
        return false;
    }

    const GrabTarget grab = resolve_grab(values);
    if (!validate_values(values, grab, error)) return false;
    float pre_shoulder = 0.0f;
    float pre_elbow = 0.0f;
    float grab_shoulder = 0.0f;
    float grab_elbow = 0.0f;
    inverse_kinematics(grab.x, grab.pre_y, pre_shoulder, pre_elbow);
    inverse_kinematics(grab.x, grab.y, grab_shoulder, grab_elbow);

    constexpr float closed_gripper = 10.0f;
    const float open_gripper = std::max(0.0f, std::min(100.0f,
        closed_gripper + values.gripper_open_delta));
    const float close_gripper = std::max(0.0f, std::min(100.0f,
        open_gripper + values.gripper_close_delta));
    constexpr float home[5] = {0.0f, -31.70f, 27.69f, 80.0f, 0.0f};
    constexpr float place_approach[5] = {-14.3f, -4.1f, -52.8f, 80.0f, 0.0f};

    std::memset(&output, 0, sizeof(output));
    output.magic = ROBOT_RUNTIME_CONFIG_MAGIC;
    output.version = ROBOT_RUNTIME_CONFIG_VERSION;
    output.size = sizeof(output);

    float pose[ROBOT_JOINT_COUNT] = {};
    for (size_t index = 0; index < 5U; ++index) pose[index] = home[index];
    pose[5] = open_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_HOME, pose, error)) return false;
    pose[5] = close_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_HOME_CLOSED, pose, error)) return false;

    const float pre[ROBOT_JOINT_COUNT] = {
        grab.pan, pre_shoulder, pre_elbow, grab.pre_wrist, values.grab_id5,
        open_gripper,
    };
    if (!set_pose(calibration, output, ROBOT_POSE_PRE, pre, error)) return false;
    const float grab_open[ROBOT_JOINT_COUNT] = {
        grab.pan, grab_shoulder, grab_elbow, grab.grab_wrist, values.grab_id5,
        open_gripper,
    };
    if (!set_pose(calibration, output, ROBOT_POSE_GRAB, grab_open, error)) return false;
    float grab_closed[ROBOT_JOINT_COUNT];
    std::copy(grab_open, grab_open + ROBOT_JOINT_COUNT, grab_closed);
    grab_closed[5] = close_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_CLOSED, grab_closed, error)) return false;
    float clear[ROBOT_JOINT_COUNT];
    std::copy(pre, pre + ROBOT_JOINT_COUNT, clear);
    clear[5] = close_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_CLEAR, clear, error)) return false;

    float carry[ROBOT_JOINT_COUNT] = {};
    for (size_t index = 0; index < 5U; ++index) carry[index] = values.carry[index];
    carry[5] = close_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_CARRY, carry, error)) return false;

    float approach[ROBOT_JOINT_COUNT] = {};
    for (size_t index = 0; index < 5U; ++index) approach[index] = place_approach[index];
    approach[0] = values.place[0];
    approach[5] = close_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_PLACE_APPROACH, approach, error)) return false;
    float release[ROBOT_JOINT_COUNT] = {};
    for (size_t index = 0; index < 5U; ++index) release[index] = values.place[index];
    release[5] = close_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_PLACE_RELEASE, release, error)) return false;
    float place_open[ROBOT_JOINT_COUNT];
    std::copy(release, release + ROBOT_JOINT_COUNT, place_open);
    place_open[5] = open_gripper;
    if (!set_pose(calibration, output, ROBOT_POSE_PLACE_OPEN, place_open, error)) return false;

    output.ball_center_tolerance = static_cast<uint16_t>(values.ball_center);
    output.ball_target_size = static_cast<uint16_t>(values.ball_target);
    output.ball_size_tolerance = static_cast<uint16_t>(values.ball_tolerance);
    output.ball_stable_frames = static_cast<uint16_t>(values.ball_stable);
    output.bucket_center_tolerance = static_cast<uint16_t>(values.bucket_center);
    output.bucket_target_size = static_cast<uint16_t>(values.bucket_target);
    output.bucket_size_tolerance = 15U;
    output.bucket_stable_frames = static_cast<uint16_t>(values.bucket_stable);
    output.ball_search_speed = static_cast<uint16_t>(values.ball_search);
    output.ball_far_speed = static_cast<uint16_t>(values.ball_far);
    output.ball_near_speed = static_cast<uint16_t>(values.ball_near);
    output.ball_reverse_speed = static_cast<uint16_t>(values.ball_reverse);
    output.ball_turn_speed = static_cast<uint16_t>(values.ball_turn);
    output.bucket_search_speed = static_cast<uint16_t>(values.bucket_turn);
    output.bucket_far_speed = static_cast<uint16_t>(values.bucket_far);
    output.bucket_near_speed = static_cast<uint16_t>(values.bucket_near);
    output.bucket_reverse_speed = static_cast<uint16_t>(values.bucket_reverse);
    output.bucket_turn_speed = static_cast<uint16_t>(values.bucket_turn);
    output.motion_speed_level = static_cast<uint16_t>(values.motion_speed_level);
    const JointCalibration* gripper = calibration.get("arm_gripper");
    output.gripper_range_min = static_cast<uint16_t>(gripper->range_min);
    output.gripper_range_max = static_cast<uint16_t>(gripper->range_max);
    output.gripper_hold_percent = 25U;
    return true;
}
