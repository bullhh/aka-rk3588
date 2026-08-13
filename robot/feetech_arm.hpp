#ifndef ROBOT_FEETECH_ARM_HPP
#define ROBOT_FEETECH_ARM_HPP

#include <map>
#include <string>
#include <vector>

#include "feetech/feetech_bus.hpp"
#include "robot/lekiwi_calibration.hpp"
#include "robot/lekiwi_arm_poses.hpp"

class FeetechArm {
public:
    explicit FeetechArm(feetech::FeetechBus& bus,
                        const std::string& calibration_path = "config/lekiwi_calibration.json");

    bool configure();
    bool load_calibration(const std::string& path);
    bool has_calibration() const { return calibrated_; }
    const std::string& calibration_error() const { return calibration_error_; }
    const std::string& last_error() const { return last_error_; }
    bool set_joint_deg(const std::string& name, float deg);
    bool set_joint_raw(const std::string& name, int raw);
    bool get_joint_deg(const std::string& name, float& deg);
    bool get_gripper_deg_allow_overload(float& deg, bool& gripper_overloaded);
    bool get_joint_degs(std::map<std::string, float>& positions);
    bool get_joint_degs_allow_gripper_overload(
        std::map<std::string, float>& positions, bool& gripper_overloaded);
    bool write_degrees(const std::map<std::string, float>& pose, int settle_ms = 0);
    bool verify_goal_delivery(const std::map<std::string, float>& pose);
    bool move_degrees_slow(const std::map<std::string, float>& pose, int settle_ms = 0);
    bool grab_pos();
    bool grab();
    bool release();
    bool release_pos();
    bool show();
    bool stop_torque();
    bool run_pose(const std::string& name, int settle_ms = 1000);
    bool has_pose(const std::string& name) const { return poses_.has(name); }
    std::vector<std::string> joint_names() const;

private:
    struct Joint {
        int id;
        float min_deg;
        float max_deg;
        bool gripper;
    };

    int deg_to_raw(const Joint& joint, float deg) const;
    float raw_to_deg(const Joint& joint, int raw) const;
    bool write_pose(const std::map<std::string, float>& pose, int settle_ms);
    bool write_raw_pose(const ArmRawPose& pose, int settle_ms);
    bool move_raw_pose_slow(const ArmRawPose& pose, int settle_ms);
    bool get_joint_degs_impl(std::map<std::string, float>& positions,
                             bool allow_gripper_overload,
                             bool* gripper_overloaded);
    bool get_joint_deg_impl(const std::string& name, float& deg,
                            uint8_t allowed_error_mask,
                            uint8_t* status_error);

    feetech::FeetechBus& bus_;
    std::map<std::string, Joint> joints_;
    LekiwiCalibration calibration_;
    LekiwiArmPoses poses_;
    bool calibrated_ = false;
    std::string calibration_error_;
    std::string last_error_;
};

#endif // ROBOT_FEETECH_ARM_HPP
