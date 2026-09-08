/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ROBOT_CONTROLLER_H_
#define ROBOT_CONTROLLER_H_

#include "feetech_bus.h"
#include "perception_result_v2.h"
#include "robot_runtime_config_v1.h"

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_INPUT_TIMEOUT_MS 350U

enum robot_state {
	ROBOT_STATE_STARTUP_HOME,
	ROBOT_STATE_SEARCH_BALL,
	ROBOT_STATE_PICK_HOME,
	ROBOT_STATE_PICK_PRE,
	ROBOT_STATE_PICK_GRAB,
	ROBOT_STATE_PICK_CLOSE,
	ROBOT_STATE_PICK_VERIFY,
	ROBOT_STATE_PICK_CLEAR,
	ROBOT_STATE_PICK_CARRY,
	ROBOT_STATE_SEARCH_BUCKET,
	ROBOT_STATE_PLACE_APPROACH,
	ROBOT_STATE_PLACE_RELEASE,
	ROBOT_STATE_PLACE_OPEN,
	ROBOT_STATE_PLACE_RETRACT,
	ROBOT_STATE_PLACE_CARRY,
	ROBOT_STATE_PLACE_CLOSE,
	ROBOT_STATE_RECOVER_OPEN,
	ROBOT_STATE_RECOVER_HOME,
	ROBOT_STATE_TEST_COMPLETE,
	ROBOT_STATE_FAULT,
};

struct arm_motion {
	uint16_t start[FEETECH_ARM_COUNT];
	uint16_t target[FEETECH_ARM_COUNT];
	int64_t started_ms;
	uint32_t duration_ms;
	bool active;
};

struct robot_controller {
	struct feetech_bus *bus;
	struct robot_runtime_config_v1 config;
	struct perception_result_v2 latest;
	struct arm_motion motion;
	enum robot_state state;
	int64_t last_input_ms;
	int64_t state_started_ms;
	uint64_t completed_cycles;
	uint32_t stable_frames;
	int16_t last_left;
	int16_t last_right;
	int8_t last_ball_side;
	bool have_input;
	bool wheel_command_valid;
	bool robot_ci_mode;
};

int robot_controller_init(struct robot_controller *controller,
				  struct feetech_bus *bus,
				  const struct robot_runtime_config_v1 *config);
void robot_controller_process_perception(
	struct robot_controller *controller,
	const struct perception_result_v2 *result, int64_t received_ms);
void robot_controller_arm_tick(struct robot_controller *controller,
			       int64_t now_ms);
bool robot_controller_input_timeout(struct robot_controller *controller,
				    int64_t now_ms);
const char *robot_controller_state_name(const struct robot_controller *controller);
bool robot_controller_prepare_config_update(struct robot_controller *controller);
int robot_controller_apply_config(
	struct robot_controller *controller,
	const struct robot_runtime_config_v1 *config);

#endif /* ROBOT_CONTROLLER_H_ */
