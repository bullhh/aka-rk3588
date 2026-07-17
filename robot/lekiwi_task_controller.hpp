#ifndef ROBOT_LEKIWI_TASK_CONTROLLER_HPP
#define ROBOT_LEKIWI_TASK_CONTROLLER_HPP

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "detect/detect.hpp"
#include "robot/feetech_arm.hpp"

struct LeKiwiPickConfig {
    // Recorded arm poses use calibrated degrees for Feetech IDs 1..5.
    float grab_id1_deg = -12.0f;
    float grab_id2_deg = 37.7f;
    float grab_id3_deg = 42.1f;
    float grab_id4_deg = 0.2f;
    float grab_id5_deg = 0.0f;
    float grab_forward_offset_cm = 0.0f;
    float grab_lateral_offset_cm = 0.0f;
    float grab_height_offset_cm = 0.0f;
    float grab_pitch_offset_deg = 0.0f;
    float carry_id1_deg = -11.3f;
    float carry_id2_deg = -18.3f;
    float carry_id3_deg = -45.0f;
    float carry_id4_deg = 51.8f;
    float carry_id5_deg = 0.1f;
    float gripper_open_delta_deg = 60.0f;
    float gripper_close_delta_deg = -60.0f;
    int carry_duration_ms = 2000;
    int carry_settle_ms = 500;
    int ball_stop_size_px = 155;
    int ball_stop_tolerance_px = 15;
    int ball_center_tolerance_px = 30;
    int ball_stable_frames = 2;

    // Internal trajectory geometry. These are intentionally not part of the
    // normal user-facing tuning file.
    float home_x = 0.0989f;
    float home_y = 0.1250f;
    float pre_grab_clearance_m = 0.1811f;
    float lift_x = -0.1000f;
    float lift_y = 0.2000f;
    float wrist_lift_pitch = -20.0f;

    bool load(const std::string& path = "config/lekiwi_pick_config.txt");
    bool save(const std::string& path = "config/lekiwi_pick_config.txt") const;
    int carry_duration_ticks() const { return std::max(1, (carry_duration_ms + 49) / 50); }
    int carry_settle_ticks() const { return std::max(0, (carry_settle_ms + 49) / 50); }
};

class LeKiwiMoveController {
public:
    struct Command {
        int left_speed = 0;
        int right_speed = 0;
        bool idle = true;
        bool reached = false;
        const char* label = "IDLE";
    };

    explicit LeKiwiMoveController(int frame_width = 640, int frame_height = 480);

    Command update_ball(const std::vector<detection>& detections);
    Command update_bucket(bool visible, int cx, int box_w, int box_h);
    void reset();
    void remember_ball(int cx);

    int target_left() const { return left_; }
    int target_right() const { return right_; }
    int target_position() const { return target_position_; }

private:
    struct TargetBox {
        bool visible = false;
        int cx = 0;
        int w = 0;
        int h = 0;
    };

    void update_geometry(int frame_width, int frame_height);
    Command control_target(const TargetBox& target, bool bucket);
    static const detection* choose_best_ball(const std::vector<detection>& detections,
                                             int target_cx);

    int frame_width_ = 640;
    int frame_height_ = 480;
    int target_w_ = 0;
    int target_h_ = 0;
    int left_ = 0;
    int right_ = 0;
    int target_cx_ = 0;
    int target_position_ = 0;
    int stable_count_ = 0;
    int last_target_cx_ = -1;
    LeKiwiPickConfig config_;
};

class LeKiwiArmController {
public:
    explicit LeKiwiArmController(FeetechArm& arm);

    bool begin_pick();
    bool begin_pick(const LeKiwiPickConfig& config);
    bool begin_put();
    bool tick();
    bool active() const { return active_; }
    bool done() const { return done_; }
    bool failed() const { return failed_; }
    void reset();
    bool verify_grab(float* gripper_pos = nullptr);
    const char* current_step_label() const;
    const std::string& last_error() const { return last_error_; }
    size_t step_index() const { return step_index_; }
    size_t step_count() const { return sequence_.size(); }

private:
    enum class Kind {
        HOME,
        MOVE_TO,
        JOINT_DELTA,
        JOINT_TARGET,
        WRIST_FLEX,
        CARRY,
        GAP,
    };

    struct Step {
        Kind kind;
        std::string joint;
        float a = 0.0f;
        float b = 0.0f;
        float c = 0.0f;
    };

    static std::vector<Step> pick_sequence(const LeKiwiPickConfig& config);
    static std::vector<Step> put_sequence(const LeKiwiPickConfig& config);
    static void inverse_kinematics(float x, float y, float& shoulder_lift, float& elbow_flex);
    static void forward_kinematics(float shoulder_lift, float elbow_flex, float& x, float& y);
    static float apply_joint_calibration(const std::string& joint, float value);
    static float remove_joint_calibration(const std::string& joint, float value);
    bool load_current_positions();
    bool send_current_targets();
    bool fail(const std::string& message);
    bool advance_step(const Step& step);
    bool step_reached(const Step& step) const;
    static const char* step_kind_label(Kind kind);

    FeetechArm& arm_;
    std::vector<Step> sequence_;
    size_t step_index_ = 0;
    bool active_ = false;
    bool done_ = false;
    bool failed_ = false;
    int gap_ticks_ = 0;
    bool step_initialized_ = false;
    int step_hold_ticks_ = 0;

    float current_x_ = 0.0989f;
    float current_y_ = 0.125f;
    float pitch_ = 80.0f;
    float move_start_distance_ = 0.0f;
    float move_start_wrist_ = 0.0f;
    std::map<std::string, float> carry_start_targets_;
    std::map<std::string, float> targets_;
    std::map<std::string, float> observed_;
    std::map<std::string, float> commanded_;
    LeKiwiPickConfig config_;
    std::string last_error_;
};

#endif // ROBOT_LEKIWI_TASK_CONTROLLER_HPP
