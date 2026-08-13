#include "robot/feetech_motion_policy.hpp"

#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (condition) return true;
    std::fprintf(stderr, "FAIL: %s\n", message);
    return false;
}

class FakeGoalRegisterIo final : public feetech_motion::GoalRegisterIo {
public:
    bool read_goal_position(int id, int& value) override {
        const auto goal = goals.find(id);
        if (goal == goals.end()) {
            error = "missing fake motor";
            return false;
        }
        value = goal->second;
        return true;
    }

    bool write_goal_position(int id, int value) override {
        writes.push_back({id, value});
        if (apply_writes) goals[id] = value;
        return true;
    }

    const std::string& last_error() const override { return error; }

    std::map<int, int> goals;
    std::vector<std::pair<int, int>> writes;
    bool apply_writes = true;
    std::string error;
};

} // namespace

int main() {
    bool ok = true;

    // The board's elbow calibration spans 2069 counts over the normalized
    // -100..100 range. A 3-unit tolerance therefore covers 31.035 counts;
    // the smallest stable integer boundary is 32 counts. The old float-only
    // comparison rejects this observed board value even though it lies on
    // that quantized boundary.
    ok = expect(feetech_motion::feedback_within_tolerance(
                    2164, 2196, 3.0f, 2069, 200.0f),
                "quantized elbow boundary must be accepted") && ok;

    ok = expect(!feetech_motion::feedback_within_tolerance(
                    2164, 2199, 3.0f, 2069, 200.0f),
                "a residual beyond the quantized boundary must be rejected") && ok;

    FakeGoalRegisterIo io;
    io.goals = {{2, 1600}, {3, 2100}};
    std::vector<feetech_motion::GoalRepair> repairs;
    std::string error;
    ok = expect(feetech_motion::verify_or_repair_goal_delivery(
                    io, {{2, 1600}, {3, 2164}}, repairs, error),
                "a stale broadcast goal must be repaired with an acknowledged write") && ok;
    ok = expect(io.writes == std::vector<std::pair<int, int>>{{3, 2164}},
                "only the stale motor goal must be rewritten") && ok;
    ok = expect(repairs.size() == 1 && repairs[0].id == 3 &&
                    repairs[0].observed == 2100 && repairs[0].expected == 2164,
                "the repair report must identify the stale motor goal") && ok;

    FakeGoalRegisterIo ignored_write;
    ignored_write.goals = {{3, 2100}};
    ignored_write.apply_writes = false;
    repairs.clear();
    error.clear();
    ok = expect(!feetech_motion::verify_or_repair_goal_delivery(
                    ignored_write, {{3, 2164}}, repairs, error),
                "an acknowledged write with stale readback must fail") && ok;

    return ok ? 0 : 1;
}
