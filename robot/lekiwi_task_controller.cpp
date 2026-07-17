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
} // namespace

bool LeKiwiPickConfig::load(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

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

        if      (key == "home_x") home_x = v;
        else if (key == "home_y") home_y = v;
        else if (key == "pre_grab_x") pre_grab_x = v;
        else if (key == "pre_grab_y") pre_grab_y = v;
        else if (key == "grab_x") grab_x = v;
        else if (key == "grab_y") grab_y = v;
        else if (key == "lift_x") lift_x = v;
        else if (key == "lift_y") lift_y = v;
        else if (key == "shoulder_pan_delta") shoulder_pan_delta = v;
        else if (key == "gripper_open_delta") gripper_open_delta = v;
        else if (key == "gripper_close_delta") gripper_close_delta = v;
        else if (key == "wrist_pick_pitch") wrist_pick_pitch = v;
        else if (key == "wrist_lift_pitch") wrist_lift_pitch = v;
        else if (key == "return_shoulder_pan") return_shoulder_pan = (int)v;
        else if (key == "ball_target_size") ball_target_size = (int)v;
        else if (key == "ball_size_tolerance") ball_size_tolerance = (int)v;
        else if (key == "ball_center_tolerance") ball_center_tolerance = (int)v;
        else if (key == "stable_frames") stable_frames = (int)v;
    }
    return true;
}

bool LeKiwiPickConfig::save(const std::string& path) const {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << "# LeKiwi C++ 抓取调参文件。\n";
    ofs << "# x/y 单位是米，对应 Desktop-Wanderer 机械臂二维逆运动学平面。\n";
    ofs << "# 如果夹爪伸过球、在球前方闭合，减小 grab_x。\n";
    ofs << "# 如果夹爪够不到球，增大 grab_x。\n";
    ofs << "# 如果夹爪太高或太低，调整 grab_y。\n\n";
    ofs << std::fixed << std::setprecision(4);
    ofs << "home_x = " << home_x << "\n";
    ofs << "home_y = " << home_y << "\n\n";
    ofs << "pre_grab_x = " << pre_grab_x << "\n";
    ofs << "pre_grab_y = " << pre_grab_y << "\n";
    ofs << "grab_x = " << grab_x << "\n";
    ofs << "grab_y = " << grab_y << "\n\n";
    ofs << "lift_x = " << lift_x << "\n";
    ofs << "lift_y = " << lift_y << "\n\n";
    ofs << "shoulder_pan_delta = " << shoulder_pan_delta << "\n";
    ofs << "gripper_open_delta = " << gripper_open_delta << "\n";
    ofs << "gripper_close_delta = " << gripper_close_delta << "\n";
    ofs << "wrist_pick_pitch = " << wrist_pick_pitch << "\n";
    ofs << "wrist_lift_pitch = " << wrist_lift_pitch << "\n\n";
    ofs << "# 抓取时夹爪的腕部角度由 wrist_pick_pitch 控制。\n";
    ofs << "# 如果夹爪不是尽量垂直向下，而是明显前倾或后仰，可以优先调整该值。\n";
    ofs << "# 建议每次改 5，例如 80 -> 75 或 85，观察夹爪闭合时是否更贴合球。\n\n";
    ofs << "# 是否在夹爪闭合后把肩部水平转动关节转回原位。0 表示不转回，1 表示转回。\n";
    ofs << "return_shoulder_pan = " << return_shoulder_pan << "\n\n";
    ofs << "# 视觉伺服停车尺寸。当前默认 155 像素，让车比 168~170 像素时稍远一点停车。\n";
    ofs << "# 增大该值，车会离球更近再停；减小该值，车会离球更远就停。\n";
    ofs << "ball_target_size = " << ball_target_size << "\n";
    ofs << "ball_size_tolerance = " << ball_size_tolerance << "\n";
    ofs << "# 进入抓取前允许的球中心误差，单位是像素。\n";
    ofs << "ball_center_tolerance = " << ball_center_tolerance << "\n";
    ofs << "stable_frames = " << stable_frames << "\n";
    return true;
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
    if (config_.ball_target_size > 0) target_position_ = config_.ball_target_size;
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
        if (std::abs(center_error) > config_.ball_center_tolerance) {
            stable_count_ = 0;
            int spd = 8;
            return center_error < 0
                ? diff_drive_cmd("BALL_FINE_LEFT", -spd, spd)
                : diff_drive_cmd("BALL_FINE_RIGHT", spd, -spd);
        }
        if (position < target_position_ - config_.ball_size_tolerance) {
            stable_count_ = 0;
            int spd = (position * 8 > target_position_ * 10) ? 8 : 35;
            return diff_drive_cmd("BALL_FORWARD", spd, spd);
        }
        if (position > target_position_ + config_.ball_size_tolerance) {
            stable_count_ = 0;
            return diff_drive_cmd("BALL_BACKWARD", -12, -12);
        }
    } else {
        if (position < (int)(target_position_ * 2.1f)) {
            stable_count_ = 0;
            return diff_drive_cmd("BUCKET_FORWARD", 25, 25);
        }
        if (position > target_position_ * 3) {
            stable_count_ = 0;
            return diff_drive_cmd("BUCKET_BACKWARD", -12, -12);
        }
    }

    stable_count_++;
    Command cmd = diff_drive_cmd(bucket ? "BUCKET_READY" : "BALL_READY", 0, 0);
    cmd.reached = stable_count_ > config_.stable_frames;
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
    std::vector<Step> seq = {
        {Kind::HOME, "", config.home_x, config.home_y},
        {Kind::JOINT_DELTA, "arm_shoulder_pan", config.shoulder_pan_delta, 0.0f},
        {Kind::JOINT_DELTA, "arm_gripper", config.gripper_open_delta, 0.0f},
        {Kind::WRIST_FLEX, "arm_wrist_flex", config.wrist_pick_pitch, 0.0f},
        {Kind::MOVE_TO, "", config.pre_grab_x, config.pre_grab_y},
        {Kind::MOVE_TO, "", config.grab_x, config.grab_y},
        {Kind::GAP, "", 0.0f, 0.0f},
        {Kind::JOINT_DELTA, "arm_gripper", config.gripper_close_delta, 0.0f},
        {Kind::GAP, "", 0.0f, 0.0f},
        {Kind::MOVE_TO, "clear", config.pre_grab_x, config.pre_grab_y},
        {Kind::MOVE_TO, "lift", config.lift_x, config.lift_y},
        {Kind::WRIST_FLEX, "arm_wrist_flex", config.wrist_lift_pitch, 0.0f},
    };
    if (config.return_shoulder_pan) {
        seq.insert(seq.end() - 2, {Kind::JOINT_DELTA, "arm_shoulder_pan", -config.shoulder_pan_delta, 0.0f});
    }
    return seq;
}

std::vector<LeKiwiArmController::Step> LeKiwiArmController::put_sequence(
    const LeKiwiPickConfig& config) {
    return {
        {Kind::JOINT_DELTA, "arm_shoulder_lift", 50.0f, 0.0f},
        {Kind::GAP, "", 0.0f, 0.0f},
        {Kind::JOINT_DELTA, "arm_gripper", 60.0f, 0.0f},
        {Kind::GAP, "", 0.0f, 0.0f},
        {Kind::MOVE_TO, "", config.lift_x, config.lift_y},
        {Kind::JOINT_DELTA, "arm_gripper", -60.0f, 0.0f},
    };
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
    pitch_ = config_.wrist_pick_pitch;
    move_start_distance_ = 0.0f;
    move_start_wrist_ = 0.0f;
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
    sequence_ = pick_sequence(config_);
    active_ = true;
    return load_current_positions();
}

bool LeKiwiArmController::begin_put() {
    reset();
    sequence_ = put_sequence(config_);
    active_ = true;
    return load_current_positions();
}

bool LeKiwiArmController::load_current_positions() {
    observed_.clear();
    commanded_.clear();
    for (const auto& name : arm_.joint_names()) {
        float deg = 0.0f;
        if (arm_.get_joint_deg(name, deg)) {
            observed_[name] = apply_joint_calibration(name, deg);
            continue;
        }
        return fail("load current position for " + name + " failed: " + arm_.last_error());
    }
    if (targets_.empty()) targets_ = observed_;
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

void LeKiwiArmController::forward_kinematics(float shoulder_lift,
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

bool LeKiwiArmController::send_current_targets() {
    std::map<std::string, float> positions;
    if (!arm_.get_joint_degs(positions)) {
        return fail("read current positions failed: " + arm_.last_error());
    }

    std::map<std::string, float> action;
    float arm_kp = 0.55f;
    float gripper_kp = 0.8f;
    if (step_index_ < sequence_.size() && sequence_[step_index_].kind == Kind::HOME) {
        constexpr float kStartupKp = 0.04f;
        constexpr float kHomeKp = 0.50f;
        constexpr int kRampTicks = 20;
        float ramp = std::min(1.0f, step_hold_ticks_ / (float)kRampTicks);
        arm_kp = kStartupKp + (kHomeKp - kStartupKp) * ramp;
        gripper_kp = arm_kp;
    }
    for (const auto& kv : targets_) {
        auto it = positions.find(kv.first);
        if (it == positions.end()) continue;
        float current = apply_joint_calibration(kv.first, it->second);
        observed_[kv.first] = current;
        float error = kv.second - current;
        float kp = (kv.first == "arm_gripper") ? gripper_kp : arm_kp;
        float next = current + kp * error;
        bool homing = step_index_ < sequence_.size() &&
                      sequence_[step_index_].kind == Kind::HOME;
        if (homing && kv.first != "arm_gripper" && step_hold_ticks_ >= 20 &&
            std::abs(error) < 10.0f) {
            next = kv.second;
        } else if (!homing && kv.first != "arm_gripper" &&
                   std::abs(error) < 6.0f) {
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
        if (++gap_ticks_ >= 6) {
            gap_ticks_ = 0;
            step_hold_ticks_ = 0;
            return true;
        }
        return false;
    }

    gap_ticks_ = 0;
    if (step.kind == Kind::HOME) {
        step_hold_ticks_++;
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
        if (step_hold_ticks_ >= 100) {
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
    } else if (!step_initialized_) {
        if (step.kind == Kind::JOINT_DELTA) {
            targets_[step.joint] += step.a;
        } else if (step.kind == Kind::WRIST_FLEX) {
            pitch_ = step.a;
        }
        step_initialized_ = true;
    }

    if (step.kind == Kind::MOVE_TO && step.joint == "lift") {
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
    } else {
        float wrist = -targets_["arm_shoulder_lift"] - targets_["arm_elbow_flex"] + pitch_;
        targets_["arm_wrist_flex"] = std::max(-100.0f, std::min(100.0f, wrist));
    }

    if (!send_current_targets()) return false;
    if (step.kind == Kind::JOINT_DELTA || step.kind == Kind::WRIST_FLEX) {
        int minimum_ticks = (step.joint == "arm_gripper") ? 8 : 4;
        if (step_hold_ticks_ < minimum_ticks) return false;
    }
    bool reached = step_reached(step);
    if (reached && step.kind == Kind::MOVE_TO && step.joint == "lift") {
        pitch_ = config_.wrist_lift_pitch;
    }
    if (!reached && step_hold_ticks_ >= 100) {
        std::ostringstream error;
        error << "step timed out: " << step_kind_label(step.kind) << ", residuals=";
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
    if (step.kind == Kind::MOVE_TO) {
        if (std::abs(current_x_ - step.a) >= 0.002f ||
            std::abs(current_y_ - step.b) >= 0.002f) {
            return false;
        }
        float arm_error = 0.0f;
        for (const char* joint : {"arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex"}) {
            auto target = targets_.find(joint);
            auto current = observed_.find(joint);
            if (target == targets_.end() || current == observed_.end()) return false;
            arm_error += std::abs(target->second - current->second);
        }
        return arm_error < 5.0f;
    }
    if (step.kind == Kind::JOINT_DELTA || step.kind == Kind::WRIST_FLEX) {
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
    if (!arm_.get_joint_deg("arm_gripper", pos)) {
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
        case Kind::WRIST_FLEX:  return "wrist_flex";
        case Kind::GAP:         return "gap";
    }
    return "unknown";
}

const char* LeKiwiArmController::current_step_label() const {
    if (step_index_ >= sequence_.size()) return "done";
    return step_kind_label(sequence_[step_index_].kind);
}
