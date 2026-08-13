#include "robot/feetech_motion_policy.hpp"

#include <cmath>
#include <cstdlib>

namespace feetech_motion {

bool feedback_within_tolerance(int target_raw, int actual_raw,
                               float logical_tolerance,
                               int raw_span, float logical_span) {
    if (logical_tolerance < 0.0f || raw_span <= 0 || logical_span <= 0.0f) {
        return false;
    }

    // Targets and feedback both cross an integer register boundary. Round the
    // requested tolerance outwards once so the decision does not depend on
    // which side of a raw-count boundary the floating-point target occupies.
    const int raw_tolerance = static_cast<int>(std::ceil(
        static_cast<double>(logical_tolerance) * raw_span / logical_span));
    return std::abs(target_raw - actual_raw) <= raw_tolerance;
}

bool verify_or_repair_goal_delivery(
    GoalRegisterIo& io,
    const std::vector<std::pair<int, int>>& goals,
    std::vector<GoalRepair>& repairs,
    std::string& error) {
    repairs.clear();
    error.clear();

    for (const auto& goal : goals) {
        int observed = 0;
        if (!io.read_goal_position(goal.first, observed)) {
            error = "read goal for motor id=" + std::to_string(goal.first) +
                    " failed: " + io.last_error();
            return false;
        }
        if (observed == goal.second) continue;

        if (!io.write_goal_position(goal.first, goal.second)) {
            error = "repair goal for motor id=" + std::to_string(goal.first) +
                    " failed: " + io.last_error();
            return false;
        }

        int confirmed = 0;
        if (!io.read_goal_position(goal.first, confirmed)) {
            error = "confirm repaired goal for motor id=" +
                    std::to_string(goal.first) + " failed: " + io.last_error();
            return false;
        }
        if (confirmed != goal.second) {
            error = "goal repair for motor id=" + std::to_string(goal.first) +
                    " was acknowledged but read back " +
                    std::to_string(confirmed) + " instead of " +
                    std::to_string(goal.second);
            return false;
        }
        repairs.push_back({goal.first, observed, goal.second});
    }
    return true;
}

} // namespace feetech_motion
