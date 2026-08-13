#ifndef ROBOT_FEETECH_MOTION_POLICY_HPP
#define ROBOT_FEETECH_MOTION_POLICY_HPP

#include <string>
#include <utility>
#include <vector>

namespace feetech_motion {

bool feedback_within_tolerance(int target_raw, int actual_raw,
                               float logical_tolerance,
                               int raw_span, float logical_span);

class GoalRegisterIo {
public:
    virtual ~GoalRegisterIo() = default;

    virtual bool read_goal_position(int id, int& value) = 0;
    virtual bool write_goal_position(int id, int value) = 0;
    virtual const std::string& last_error() const = 0;
};

struct GoalRepair {
    int id;
    int observed;
    int expected;
};

bool verify_or_repair_goal_delivery(
    GoalRegisterIo& io,
    const std::vector<std::pair<int, int>>& goals,
    std::vector<GoalRepair>& repairs,
    std::string& error);

} // namespace feetech_motion

#endif // ROBOT_FEETECH_MOTION_POLICY_HPP
