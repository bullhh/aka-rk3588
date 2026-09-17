#include "omni_base.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstdint>
#include <time.h>
#include <unistd.h>

OmniBase::OmniBase(feetech::FeetechBus& bus) : bus_(bus) {}

void OmniBase::set_wheel_ids(int left_id, int back_id, int right_id) {
    left_id_ = left_id;
    back_id_ = back_id;
    right_id_ = right_id;
}

void OmniBase::set_limits(float wheel_radius_m, float base_radius_m, int max_raw) {
    wheel_radius_m_ = wheel_radius_m;
    base_radius_m_ = base_radius_m;
    max_raw_ = max_raw;
}

bool OmniBase::configure() {
    for (int id : {left_id_, back_id_, right_id_}) {
        if (!bus_.enable_torque(id, false)) return false;
    }

    for (int id : {left_id_, back_id_, right_id_}) {
        if (!bus_.set_operating_mode(id, feetech::OperatingMode::VELOCITY) ||
            !bus_.set_acceleration(id, 80) ||
            !bus_.set_goal_velocity(id, 0)) {
            return false;
        }
    }

    for (int id : {left_id_, back_id_, right_id_}) {
        // Never enable a velocity-mode wheel with a stale non-zero goal.
        if (!bus_.enable_torque(id, true)) {
            for (int rollback_id : {left_id_, back_id_, right_id_}) {
                bus_.enable_torque(rollback_id, false);
            }
            return false;
        }
    }
    return true;
}

int OmniBase::degps_to_raw(float degps) {
    float steps_per_deg = 4096.0f / 360.0f;
    int raw = (int)std::round(degps * steps_per_deg);
    raw = std::max(-32767, std::min(32767, raw));
    return raw;
}

bool OmniBase::drive_body(float x_mps, float y_mps, float theta_degps) {
    const float pi = 3.14159265358979323846f;
    float theta_rad = theta_degps * pi / 180.0f;
    float angles_deg[3] = {240.0f - 90.0f, 0.0f - 90.0f, 120.0f - 90.0f};
    float wheel_degps[3];

    for (int i = 0; i < 3; i++) {
        float a = angles_deg[i] * pi / 180.0f;
        float linear = std::cos(a) * x_mps + std::sin(a) * y_mps + base_radius_m_ * theta_rad;
        float angular_radps = linear / wheel_radius_m_;
        wheel_degps[i] = angular_radps * 180.0f / pi;
    }

    float max_abs_raw = 0.0f;
    float steps_per_deg = 4096.0f / 360.0f;
    for (float degps : wheel_degps) {
        max_abs_raw = std::max(max_abs_raw, std::fabs(degps * steps_per_deg));
    }
    if (max_abs_raw > max_raw_ && max_abs_raw > 1.0f) {
        float scale = max_raw_ / max_abs_raw;
        for (float& degps : wheel_degps) degps *= scale;
    }

    return write_wheels(degps_to_raw(wheel_degps[0]),
                        degps_to_raw(wheel_degps[1]),
                        degps_to_raw(wheel_degps[2]));
}

bool OmniBase::write_wheels(int left_raw, int back_raw, int right_raw) {
    std::vector<std::pair<int, int>> values = {
        {left_id_, left_raw},
        {back_id_, back_raw},
        {right_id_, right_raw},
    };
    const bool ok = bus_.sync_write_u16(feetech::reg::GOAL_VELOCITY, values, true);
    if (!ok && command_error_.empty()) {
        command_error_ = bus_.last_error().empty() ? "wheel command failed" : bus_.last_error();
    }
    return ok;
}

bool OmniBase::stop() {
    return write_wheels(0, 0, 0);
}

bool OmniBase::wait_for_wheels(int direction, bool (*cancelled)()) {
    const auto now_ms = []() -> uint64_t {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return uint64_t(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
    };
    const uint64_t deadline = now_ms() + 2000;
    const int ids[] = {left_id_, back_id_, right_id_};
    int velocities[3] = {};
    int stable = 0;
    do {
        if (cancelled && cancelled()) return false;
        bool matched = true;
        for (int i = 0; i < 3; ++i) {
            if (!bus_.read_u16(ids[i], feetech::reg::PRESENT_VELOCITY, velocities[i], true)) {
                fprintf(stderr, "[WHEEL] feedback read failed id=%d: %s\n",
                        ids[i], bus_.last_error().c_str());
                return false;
            }
            matched = matched && (direction == 0 ? velocities[i] == 0
                                                 : velocities[i] * direction >= 20);
        }
        if (now_ms() >= deadline) break;
        stable = matched ? stable + 1 : 0;
        if (stable >= 3) {
            printf("[WHEEL] feedback direction=%d ids=%d,%d,%d velocity=%d,%d,%d samples=%d\n",
                   direction, ids[0], ids[1], ids[2], velocities[0], velocities[1], velocities[2], stable);
            return true;
        }
        usleep(50000);
    } while (now_ms() < deadline);
    fprintf(stderr, "[WHEEL] feedback timeout direction=%d velocity=%d,%d,%d\n",
            direction, velocities[0], velocities[1], velocities[2]);
    return false;
}

bool OmniBase::stop_and_verify(bool (*cancelled)()) {
    return stop() && wait_for_wheels(0, cancelled);
}

bool OmniBase::verify_wheel_motion(bool (*cancelled)()) {
    bool ok = commands_ok() && stop_and_verify(cancelled);
    for (int direction : {1, -1}) {
        if (!ok || (cancelled && cancelled())) { ok = false; break; }
        // Pure rotation commands all three wheels; no wheel can be omitted.
        ok = drive_body(0.0f, 0.0f, direction * 15.0f) &&
             wait_for_wheels(direction, cancelled);
        const bool stopped = stop_and_verify(cancelled);
        ok = ok && stopped;
    }
    // Also send a stop after a failed write/read, timeout or cancellation.
    const bool stopped = stop_and_verify(cancelled);
    return ok && stopped && commands_ok();
}

OmniBase::SpeedLevel OmniBase::speed_level(int level) const {
    static const SpeedLevel levels[] = {
        {0.02f, 15.0f},
        {0.05f, 30.0f},
        {0.15f, 40.0f},
        {0.25f, 50.0f},
    };
    level = std::max(0, std::min(level, (int)(sizeof(levels) / sizeof(levels[0])) - 1));
    return levels[level];
}

bool OmniBase::forward(int level) {
    SpeedLevel s = speed_level(level);
    return drive_body(s.xy, 0.0f, 0.0f);
}

bool OmniBase::backward(int level) {
    SpeedLevel s = speed_level(level);
    return drive_body(-s.xy, 0.0f, 0.0f);
}

bool OmniBase::left(int level) {
    SpeedLevel s = speed_level(level);
    return drive_body(0.0f, s.xy, 0.0f);
}

bool OmniBase::right(int level) {
    SpeedLevel s = speed_level(level);
    return drive_body(0.0f, -s.xy, 0.0f);
}

bool OmniBase::rotate_left(int level) {
    SpeedLevel s = speed_level(level);
    return drive_body(0.0f, 0.0f, s.theta);
}

bool OmniBase::rotate_right(int level) {
    SpeedLevel s = speed_level(level);
    return drive_body(0.0f, 0.0f, -s.theta);
}
