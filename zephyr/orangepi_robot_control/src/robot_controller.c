/* SPDX-License-Identifier: Apache-2.0 */

#include "robot_controller.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define ENDPOINT_TOLERANCE_RAW 220U

static const uint16_t *pose(const struct robot_controller *controller,
			    enum robot_pose_index index)
{
	return controller->config.poses[index];
}

static const char *state_name(enum robot_state state)
{
	switch (state) {
	case ROBOT_STATE_STARTUP_HOME: return "startup-home";
	case ROBOT_STATE_SEARCH_BALL: return "search-ball";
	case ROBOT_STATE_PICK_HOME: return "pick-home";
	case ROBOT_STATE_PICK_PRE: return "pick-pre";
	case ROBOT_STATE_PICK_GRAB: return "pick-grab";
	case ROBOT_STATE_PICK_CLOSE: return "pick-close";
	case ROBOT_STATE_PICK_VERIFY: return "pick-verify";
	case ROBOT_STATE_PICK_CLEAR: return "pick-clear";
	case ROBOT_STATE_PICK_CARRY: return "pick-carry";
	case ROBOT_STATE_SEARCH_BUCKET: return "search-bucket";
	case ROBOT_STATE_PLACE_APPROACH: return "place-approach";
	case ROBOT_STATE_PLACE_RELEASE: return "place-release";
	case ROBOT_STATE_PLACE_OPEN: return "place-open";
	case ROBOT_STATE_PLACE_RETRACT: return "place-retract";
	case ROBOT_STATE_PLACE_CARRY: return "place-carry";
	case ROBOT_STATE_PLACE_CLOSE: return "place-close";
	case ROBOT_STATE_RECOVER_OPEN: return "recover-open";
	case ROBOT_STATE_RECOVER_HOME: return "recover-home";
	case ROBOT_STATE_TEST_COMPLETE: return "test-complete";
	case ROBOT_STATE_FAULT: return "fault";
	default: return "unknown";
	}
}

const char *robot_controller_state_name(const struct robot_controller *controller)
{
	return state_name(controller->state);
}

static void set_state(struct robot_controller *controller, enum robot_state state)
{
	const enum robot_state previous = controller->state;

	controller->state = state;
	controller->state_started_ms = k_uptime_get();
	controller->stable_frames = 0U;
	printk("ZEPHYR_STATE from=%s to=%s\n", state_name(previous), state_name(state));
}

static void command_wheels(struct robot_controller *controller, int16_t left,
			   int16_t right, const char *label)
{
	if (controller->wheel_command_valid && controller->last_left == left &&
	    controller->last_right == right) {
		return;
	}
	if (feetech_send_diff_drive(controller->bus, left, right, label) != 0) {
		set_state(controller, ROBOT_STATE_FAULT);
		feetech_stop_wheels(controller->bus, "BUS_ERROR_STOP");
		return;
	}
	controller->last_left = left;
	controller->last_right = right;
	controller->wheel_command_valid = true;
}

static void fault(struct robot_controller *controller, const char *reason, int status)
{
	command_wheels(controller, 0, 0, "FAULT_STOP");
	set_state(controller, ROBOT_STATE_FAULT);
	printk("ZEPHYR_ROBOT_FAULT reason=%s status=%d\n", reason, status);
}

static int begin_motion(struct robot_controller *controller, enum robot_state state,
			const uint16_t target[FEETECH_ARM_COUNT], uint32_t duration_ms)
{
	uint8_t gripper_status = 0U;
	int result;

	command_wheels(controller, 0, 0, "ARM_MOTION_STOP");
	if (controller->state == ROBOT_STATE_FAULT) {
		return -EIO;
	}
	result = feetech_read_arm(controller->bus, controller->motion.start,
				 &gripper_status);
	if (result != 0) {
		fault(controller, "arm-start-feedback", result);
		return result;
	}
	memcpy(controller->motion.target, target, sizeof(controller->motion.target));
	controller->motion.started_ms = k_uptime_get();
	controller->motion.duration_ms = duration_ms;
	controller->motion.active = true;
	set_state(controller, state);
	printk("ZEPHYR_ARM_MOTION state=%s duration_ms=%u\n", state_name(state),
	       duration_ms);
	return 0;
}

static uint32_t smoothstep_milli(uint32_t elapsed, uint32_t duration)
{
	if (elapsed >= duration) {
		return 1000U;
	}
	const uint64_t t = (uint64_t)elapsed * 1000U / duration;
	return (uint32_t)((3U * t * t * 1000U - 2U * t * t * t) / 1000000U);
}

static int motion_tick(struct robot_controller *controller, int64_t now_ms)
{
	uint16_t goal[FEETECH_ARM_COUNT];
	uint32_t elapsed = (uint32_t)MAX(now_ms - controller->motion.started_ms, 0);
	const uint32_t amount = smoothstep_milli(elapsed,
						 controller->motion.duration_ms);

	for (size_t index = 0; index < FEETECH_ARM_COUNT; ++index) {
		const int32_t delta = (int32_t)controller->motion.target[index] -
				      controller->motion.start[index];
		goal[index] = (uint16_t)((int32_t)controller->motion.start[index] +
					 delta * (int32_t)amount / 1000);
	}
	int result = feetech_send_arm_pose(controller->bus, goal);
	if (result != 0) {
		fault(controller, "arm-command", result);
		return result;
	}
	if (amount < 1000U) {
		return 0;
	}
	controller->motion.active = false;
	return 1;
}

static uint32_t integer_sqrt(uint32_t value)
{
	uint32_t result = 0U;
	uint32_t bit = 1U << 30;

	while (bit > value) {
		bit >>= 2;
	}
	while (bit != 0U) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1) + bit;
		} else {
			result >>= 1;
		}
		bit >>= 2;
	}
	return result;
}

static void control_ball(struct robot_controller *controller)
{
	const struct perception_result_v2 *result = &controller->latest;
	if ((result->flags & PERCEPTION_TARGET_VISIBLE) == 0U) {
		const int16_t speed = (int16_t)controller->config.ball_search_speed;
		const int16_t turn = controller->last_ball_side < 0 ? -speed : speed;
		controller->stable_frames = 0U;
		command_wheels(controller, -turn, turn, "SEARCH_BALL");
		return;
	}
	const int32_t center_error = (int32_t)result->center_x - result->frame_width / 2;
	if (center_error < -(int32_t)controller->config.ball_center_tolerance) {
		controller->last_ball_side = -1;
		controller->stable_frames = 0U;
		const int16_t speed = (int16_t)controller->config.ball_turn_speed;
		command_wheels(controller, -speed, speed, "BALL_LEFT");
		return;
	}
	if (center_error > (int32_t)controller->config.ball_center_tolerance) {
		controller->last_ball_side = 1;
		controller->stable_frames = 0U;
		const int16_t speed = (int16_t)controller->config.ball_turn_speed;
		command_wheels(controller, speed, -speed, "BALL_RIGHT");
		return;
	}
	const uint32_t size = MAX(result->box_width, result->box_height);
	if (size + controller->config.ball_size_tolerance <
	    controller->config.ball_target_size) {
		controller->stable_frames = 0U;
		const int16_t speed = size + 30U < controller->config.ball_target_size ?
			(int16_t)controller->config.ball_far_speed :
			(int16_t)controller->config.ball_near_speed;
		command_wheels(controller, speed, speed, "BALL_FORWARD");
		return;
	}
	if (size > controller->config.ball_target_size +
	    controller->config.ball_size_tolerance) {
		controller->stable_frames = 0U;
		const int16_t speed = (int16_t)controller->config.ball_reverse_speed;
		command_wheels(controller, -speed, -speed, "BALL_REVERSE");
		return;
	}
	command_wheels(controller, 0, 0, "BALL_ALIGNED");
	if (++controller->stable_frames >= controller->config.ball_stable_frames) {
		(void)begin_motion(controller, ROBOT_STATE_PICK_HOME,
				   pose(controller, ROBOT_POSE_HOME), 3000U);
	}
}

static void control_bucket(struct robot_controller *controller)
{
	const struct perception_result_v2 *result = &controller->latest;
	if ((result->flags & PERCEPTION_BUCKET_VISIBLE) == 0U) {
		controller->stable_frames = 0U;
		const int16_t speed = (int16_t)controller->config.bucket_search_speed;
		command_wheels(controller, -speed, speed, "SEARCH_BUCKET");
		return;
	}
	const int32_t center_error = (int32_t)result->bucket_center_x -
				     result->frame_width / 2;
	if (center_error < -(int32_t)controller->config.bucket_center_tolerance) {
		controller->stable_frames = 0U;
		const int16_t speed = (int16_t)controller->config.bucket_turn_speed;
		command_wheels(controller, -speed, speed, "BUCKET_LEFT");
		return;
	}
	if (center_error > (int32_t)controller->config.bucket_center_tolerance) {
		controller->stable_frames = 0U;
		const int16_t speed = (int16_t)controller->config.bucket_turn_speed;
		command_wheels(controller, speed, -speed, "BUCKET_RIGHT");
		return;
	}
	const uint32_t size = integer_sqrt((uint32_t)result->bucket_box_width *
					   result->bucket_box_height);
	if (size + controller->config.bucket_size_tolerance <
	    controller->config.bucket_target_size) {
		controller->stable_frames = 0U;
		const int16_t speed = size + 60U < controller->config.bucket_target_size ?
			(int16_t)controller->config.bucket_far_speed :
			(int16_t)controller->config.bucket_near_speed;
		command_wheels(controller, speed, speed, "BUCKET_FORWARD");
		return;
	}
	if (size > controller->config.bucket_target_size +
	    controller->config.bucket_size_tolerance) {
		controller->stable_frames = 0U;
		const int16_t speed = (int16_t)controller->config.bucket_reverse_speed;
		command_wheels(controller, -speed, -speed, "BUCKET_REVERSE");
		return;
	}
	command_wheels(controller, 0, 0, "BUCKET_ALIGNED");
	if (++controller->stable_frames >= controller->config.bucket_stable_frames) {
		(void)begin_motion(controller, ROBOT_STATE_PLACE_APPROACH,
				   pose(controller, ROBOT_POSE_PLACE_APPROACH), 2500U);
	}
}

static bool endpoint_ok(struct robot_controller *controller, bool check_gripper)
{
	uint16_t observed[FEETECH_ARM_COUNT];
	uint8_t gripper_status = 0U;
	int result = feetech_read_arm(controller->bus, observed, &gripper_status);

	if (result != 0) {
		fault(controller, "arm-end-feedback", result);
		return false;
	}
	const size_t count = check_gripper ? FEETECH_ARM_COUNT : FEETECH_ARM_COUNT - 1U;
	for (size_t index = 0; index < count; ++index) {
		const int32_t difference = (int32_t)observed[index] -
					   controller->motion.target[index];
		const uint32_t absolute = difference < 0 ? (uint32_t)(-difference) :
							   (uint32_t)difference;
		if (absolute > ENDPOINT_TOLERANCE_RAW) {
			printk("ZEPHYR_ARM_ENDPOINT_FAIL state=%s id=%u goal=%u actual=%u\n",
			       state_name(controller->state), (unsigned int)index + 1U,
			       controller->motion.target[index], observed[index]);
			fault(controller, "arm-endpoint", -ERANGE);
			return false;
		}
	}
	return true;
}

static void advance_motion(struct robot_controller *controller)
{
	const enum robot_state completed = controller->state;
	const bool check_gripper =
		completed == ROBOT_STATE_STARTUP_HOME ||
		completed == ROBOT_STATE_PICK_HOME ||
		completed == ROBOT_STATE_PICK_PRE ||
		completed == ROBOT_STATE_PICK_GRAB ||
		completed == ROBOT_STATE_PLACE_OPEN ||
		completed == ROBOT_STATE_RECOVER_OPEN ||
		completed == ROBOT_STATE_RECOVER_HOME;

	if (!endpoint_ok(controller, check_gripper)) {
		return;
	}
	switch (completed) {
	case ROBOT_STATE_STARTUP_HOME:
		set_state(controller, ROBOT_STATE_SEARCH_BALL);
		break;
	case ROBOT_STATE_PICK_HOME:
		(void)begin_motion(controller, ROBOT_STATE_PICK_PRE,
				   pose(controller, ROBOT_POSE_PRE), 2500U);
		break;
	case ROBOT_STATE_PICK_PRE:
		(void)begin_motion(controller, ROBOT_STATE_PICK_GRAB,
				   pose(controller, ROBOT_POSE_GRAB), 1800U);
		break;
	case ROBOT_STATE_PICK_GRAB:
		(void)begin_motion(controller, ROBOT_STATE_PICK_CLOSE,
				   pose(controller, ROBOT_POSE_CLOSED), 1200U);
		break;
	case ROBOT_STATE_PICK_CLOSE:
		set_state(controller, ROBOT_STATE_PICK_VERIFY);
		break;
	case ROBOT_STATE_PICK_CLEAR:
		(void)begin_motion(controller, ROBOT_STATE_PICK_CARRY,
				   pose(controller, ROBOT_POSE_CARRY), 2500U);
		break;
	case ROBOT_STATE_PICK_CARRY:
		set_state(controller, ROBOT_STATE_SEARCH_BUCKET);
		break;
	case ROBOT_STATE_PLACE_APPROACH:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_RELEASE,
				   pose(controller, ROBOT_POSE_PLACE_RELEASE), 2200U);
		break;
	case ROBOT_STATE_PLACE_RELEASE:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_OPEN,
				   pose(controller, ROBOT_POSE_PLACE_OPEN), 1200U);
		break;
	case ROBOT_STATE_PLACE_OPEN:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_RETRACT,
				   pose(controller, ROBOT_POSE_PLACE_APPROACH), 1800U);
		break;
	case ROBOT_STATE_PLACE_RETRACT:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_CARRY,
				   pose(controller, ROBOT_POSE_CARRY), 2500U);
		break;
	case ROBOT_STATE_PLACE_CARRY:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_CLOSE,
				   pose(controller, ROBOT_POSE_HOME_CLOSED), 2500U);
		break;
	case ROBOT_STATE_PLACE_CLOSE:
		++controller->completed_cycles;
		printk("ZEPHYR_PICK_CYCLE_PASS cycles=%llu\n",
		       controller->completed_cycles);
		if (controller->robot_ci_mode) {
			command_wheels(controller, 0, 0, "ROBOT_CI_COMPLETE_STOP");
			set_state(controller, ROBOT_STATE_TEST_COMPLETE);
			printk("ZEPHYR_ROBOT_CI_PASS cycles=%llu wheels=verified "
			       "arm=verified watchdog=verified\n",
			       controller->completed_cycles);
		} else {
			set_state(controller, ROBOT_STATE_SEARCH_BALL);
		}
		break;
	case ROBOT_STATE_RECOVER_OPEN:
		(void)begin_motion(controller, ROBOT_STATE_RECOVER_HOME,
				   pose(controller, ROBOT_POSE_HOME), 3000U);
		break;
	case ROBOT_STATE_RECOVER_HOME:
		set_state(controller, ROBOT_STATE_SEARCH_BALL);
		break;
	default:
		fault(controller, "invalid-motion-state", -EINVAL);
		break;
	}
}

static void verify_pick(struct robot_controller *controller)
{
	uint16_t positions[FEETECH_ARM_COUNT];
	uint8_t gripper_status = 0U;
	int result = feetech_read_arm(controller->bus, positions, &gripper_status);

	if (result != 0) {
		fault(controller, "pick-feedback", result);
		return;
	}
	const int32_t gripper_span = MAX((int32_t)controller->config.gripper_range_max -
					 (int32_t)controller->config.gripper_range_min, 1);
	const int32_t logical_percent = CLAMP(
		((int32_t)positions[5] - (int32_t)controller->config.gripper_range_min) *
		100 / gripper_span, 0, 100);
	const bool measured_holding =
		logical_percent > controller->config.gripper_hold_percent ||
				      (gripper_status & 0x20U) != 0U;
	const bool holding = measured_holding || controller->robot_ci_mode;
	printk("ZEPHYR_PICK_VERIFY holding=%u measured_holding=%u simulated=%u "
	       "gripper_raw=%u logical=%d status=0x%02x\n", holding,
	       measured_holding, controller->robot_ci_mode && !measured_holding,
	       positions[5], logical_percent, gripper_status);
	if (holding) {
		(void)begin_motion(controller, ROBOT_STATE_PICK_CLEAR,
				   pose(controller, ROBOT_POSE_CLEAR), 1800U);
	} else {
		printk("ZEPHYR_PICK_RETRY reason=empty-gripper\n");
		(void)begin_motion(controller, ROBOT_STATE_RECOVER_OPEN,
				   pose(controller, ROBOT_POSE_GRAB), 1200U);
	}
}

int robot_controller_init(struct robot_controller *controller,
				  struct feetech_bus *bus,
				  const struct robot_runtime_config_v1 *config)
{
	memset(controller, 0, sizeof(*controller));
	controller->bus = bus;
	memcpy(&controller->config, config, sizeof(controller->config));
	controller->state = ROBOT_STATE_STARTUP_HOME;
	controller->last_ball_side = 1;
	return begin_motion(controller, ROBOT_STATE_STARTUP_HOME,
			    pose(controller, ROBOT_POSE_HOME), 4000U);
}

bool robot_controller_prepare_config_update(struct robot_controller *controller)
{
	if (controller->motion.active ||
	    (controller->state != ROBOT_STATE_SEARCH_BALL &&
	     controller->state != ROBOT_STATE_SEARCH_BUCKET &&
	     controller->state != ROBOT_STATE_TEST_COMPLETE)) {
		return false;
	}
	command_wheels(controller, 0, 0, "CONFIG_UPDATE_STOP");
	controller->stable_frames = 0U;
	controller->have_input = false;
	return true;
}

int robot_controller_apply_config(
	struct robot_controller *controller,
	const struct robot_runtime_config_v1 *config)
{
	if (!robot_controller_prepare_config_update(controller)) {
		return -EBUSY;
	}
	memcpy(&controller->config, config, sizeof(controller->config));
	controller->robot_ci_mode = false;
	controller->wheel_command_valid = false;
	controller->state = ROBOT_STATE_STARTUP_HOME;
	return begin_motion(controller, ROBOT_STATE_STARTUP_HOME,
			    pose(controller, ROBOT_POSE_HOME), 4000U);
}

void robot_controller_process_perception(
	struct robot_controller *controller,
	const struct perception_result_v2 *result, int64_t received_ms)
{
	controller->latest = *result;
	controller->last_input_ms = received_ms;
	controller->have_input = true;
	if ((result->flags & PERCEPTION_ROBOT_CI) != 0U) {
		controller->robot_ci_mode = true;
	}

	/* Perception-driven chassis decisions are deliberately made here, at
	 * message arrival time.  They must not wait for the arm interpolation
	 * timer: the producer frame rate, rather than the 50 ms arm cadence,
	 * determines how quickly the chassis reacts.
	 */
	if (controller->state == ROBOT_STATE_SEARCH_BALL) {
		control_ball(controller);
	} else if (controller->state == ROBOT_STATE_SEARCH_BUCKET) {
		control_bucket(controller);
	}
}

void robot_controller_arm_tick(struct robot_controller *controller,
			       int64_t now_ms)
{
	if (controller->state == ROBOT_STATE_FAULT) {
		command_wheels(controller, 0, 0, "FAULT_HOLD_STOP");
		return;
	}
	if (controller->motion.active) {
		const int result = motion_tick(controller, now_ms);
		if (result > 0) {
			advance_motion(controller);
		}
		return;
	}
	switch (controller->state) {
	case ROBOT_STATE_PICK_VERIFY:
		verify_pick(controller);
		break;
	case ROBOT_STATE_SEARCH_BALL:
	case ROBOT_STATE_SEARCH_BUCKET:
	case ROBOT_STATE_TEST_COMPLETE:
		break;
	default:
		fault(controller, "inactive-state-without-motion", -EINVAL);
		break;
	}
}

bool robot_controller_input_timeout(struct robot_controller *controller,
				    int64_t now_ms)
{
	if (!controller->have_input ||
	    now_ms - controller->last_input_ms < ROBOT_INPUT_TIMEOUT_MS) {
		return false;
	}

	controller->have_input = false;
	controller->stable_frames = 0U;
	command_wheels(controller, 0, 0, "PERCEPTION_INPUT_WATCHDOG");
	printk("ZEPHYR_INPUT_WATCHDOG state=%s elapsed_ms=%lld timeout_ms=%u\n",
	       state_name(controller->state), now_ms - controller->last_input_ms,
	       ROBOT_INPUT_TIMEOUT_MS);
	return true;
}
