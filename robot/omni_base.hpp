#ifndef ROBOT_OMNI_BASE_HPP
#define ROBOT_OMNI_BASE_HPP

#include <string>

#include "feetech/feetech_bus.hpp"

class OmniBase {
public:
    explicit OmniBase(feetech::FeetechBus& bus);

    bool configure();
    bool drive_body(float x_mps, float y_mps, float theta_degps);
    bool stop();
    // Bounded feedback checks for a chassis raised off the ground.
    bool verify_wheel_motion(bool (*cancelled)() = nullptr);
    bool stop_and_verify(bool (*cancelled)() = nullptr);
    bool commands_ok() const { return command_error_.empty(); }
    const std::string& command_error() const { return command_error_; }

    bool forward(int level = 1);
    bool backward(int level = 1);
    bool left(int level = 1);
    bool right(int level = 1);
    bool rotate_left(int level = 1);
    bool rotate_right(int level = 1);

    void set_wheel_ids(int left_id, int back_id, int right_id);
    void set_limits(float wheel_radius_m, float base_radius_m, int max_raw);

private:
    struct SpeedLevel {
        float xy;
        float theta;
    };

    bool write_wheels(int left_raw, int back_raw, int right_raw);
    bool wait_for_wheels(int direction, bool (*cancelled)());
    static int degps_to_raw(float degps);
    SpeedLevel speed_level(int level) const;

    feetech::FeetechBus& bus_;
    int left_id_ = 7;
    int back_id_ = 8;
    int right_id_ = 9;
    float wheel_radius_m_ = 0.05f;
    float base_radius_m_ = 0.125f;
    int max_raw_ = 3000;
    // Preserve the first failed command even if subsequent stop writes succeed.
    std::string command_error_;
};

#endif // ROBOT_OMNI_BASE_HPP
