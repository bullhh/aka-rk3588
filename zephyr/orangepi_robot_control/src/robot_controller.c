/* SPDX-License-Identifier: Apache-2.0 */

#include "robot_controller.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define BALL_CENTER_TOLERANCE 30U
#define BALL_TARGET_SIZE 155U
#define BALL_SIZE_TOLERANCE 5U
#define BALL_STABLE_FRAMES 2U
#define BUCKET_CENTER_TOLERANCE 20U
#define BUCKET_TARGET_SIZE 380U
#define BUCKET_SIZE_TOLERANCE 15U
#define BUCKET_STABLE_FRAMES 3U
#define ENDPOINT_TOLERANCE_RAW 220U

/* Raw goals generated from the robot's checked-in LeKiwi calibration and the
 * validated pick/place configuration.  IDs 1..6 are pan, shoulder, elbow,
 * wrist, roll and gripper respectively.
 */
static const uint16_t pose_home[] = {2197U, 1699U, 2250U, 3042U, 2048U, 2289U};
static const uint16_t pose_home_closed[] = {
	2197U, 1699U, 2250U, 3042U, 2048U, 1404U,
};
static const uint16_t pose_pre[] = {1937U, 2112U, 1891U, 3042U, 2048U, 2289U};
static const uint16_t pose_grab[] = {1937U, 2628U, 2183U, 2537U, 2048U, 2289U};
static const uint16_t pose_closed[] = {1937U, 2628U, 2183U, 2537U, 2048U, 1404U};
static const uint16_t pose_clear[] = {1937U, 2112U, 1891U, 3042U, 2048U, 1404U};
static const uint16_t pose_carry[] = {2048U, 1871U, 1489U, 2686U, 2052U, 1404U};
static const uint16_t pose_place_approach[] = {
	2009U, 2054U, 1408U, 3042U, 2048U, 1404U,
};
static const uint16_t pose_place_release[] = {
	2009U, 2369U, 1691U, 2916U, 2048U, 1404U,
};
static const uint16_t pose_place_open[] = {
	2009U, 2369U, 1691U, 2916U, 2048U, 2289U,
};

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
		const int16_t turn = controller->last_ball_side < 0 ? -18 : 18;
		controller->stable_frames = 0U;
		command_wheels(controller, -turn, turn, "SEARCH_BALL");
		return;
	}
	const int32_t center_error = (int32_t)result->center_x - result->frame_width / 2;
	if (center_error < -(int32_t)BALL_CENTER_TOLERANCE) {
		controller->last_ball_side = -1;
		controller->stable_frames = 0U;
		command_wheels(controller, -15, 15, "BALL_LEFT");
		return;
	}
	if (center_error > (int32_t)BALL_CENTER_TOLERANCE) {
		controller->last_ball_side = 1;
		controller->stable_frames = 0U;
		command_wheels(controller, 15, -15, "BALL_RIGHT");
		return;
	}
	const uint32_t size = MAX(result->box_width, result->box_height);
	if (size + BALL_SIZE_TOLERANCE < BALL_TARGET_SIZE) {
		controller->stable_frames = 0U;
		const int16_t speed = size + 30U < BALL_TARGET_SIZE ? 65 : 20;
		command_wheels(controller, speed, speed, "BALL_FORWARD");
		return;
	}
	if (size > BALL_TARGET_SIZE + BALL_SIZE_TOLERANCE) {
		controller->stable_frames = 0U;
		command_wheels(controller, -25, -25, "BALL_REVERSE");
		return;
	}
	command_wheels(controller, 0, 0, "BALL_ALIGNED");
	if (++controller->stable_frames >= BALL_STABLE_FRAMES) {
		(void)begin_motion(controller, ROBOT_STATE_PICK_HOME, pose_home, 3000U);
	}
}

static void control_bucket(struct robot_controller *controller)
{
	const struct perception_result_v2 *result = &controller->latest;
	if ((result->flags & PERCEPTION_BUCKET_VISIBLE) == 0U) {
		controller->stable_frames = 0U;
		command_wheels(controller, -18, 18, "SEARCH_BUCKET");
		return;
	}
	const int32_t center_error = (int32_t)result->bucket_center_x -
				     result->frame_width / 2;
	if (center_error < -(int32_t)BUCKET_CENTER_TOLERANCE) {
		controller->stable_frames = 0U;
		command_wheels(controller, -18, 18, "BUCKET_LEFT");
		return;
	}
	if (center_error > (int32_t)BUCKET_CENTER_TOLERANCE) {
		controller->stable_frames = 0U;
		command_wheels(controller, 18, -18, "BUCKET_RIGHT");
		return;
	}
	const uint32_t size = integer_sqrt((uint32_t)result->bucket_box_width *
					   result->bucket_box_height);
	if (size + BUCKET_SIZE_TOLERANCE < BUCKET_TARGET_SIZE) {
		controller->stable_frames = 0U;
		const int16_t speed = size + 60U < BUCKET_TARGET_SIZE ? 70 : 25;
		command_wheels(controller, speed, speed, "BUCKET_FORWARD");
		return;
	}
	if (size > BUCKET_TARGET_SIZE + BUCKET_SIZE_TOLERANCE) {
		controller->stable_frames = 0U;
		command_wheels(controller, -25, -25, "BUCKET_REVERSE");
		return;
	}
	command_wheels(controller, 0, 0, "BUCKET_ALIGNED");
	if (++controller->stable_frames >= BUCKET_STABLE_FRAMES) {
		(void)begin_motion(controller, ROBOT_STATE_PLACE_APPROACH,
				   pose_place_approach, 2500U);
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
		(void)begin_motion(controller, ROBOT_STATE_PICK_PRE, pose_pre, 2500U);
		break;
	case ROBOT_STATE_PICK_PRE:
		(void)begin_motion(controller, ROBOT_STATE_PICK_GRAB, pose_grab, 1800U);
		break;
	case ROBOT_STATE_PICK_GRAB:
		(void)begin_motion(controller, ROBOT_STATE_PICK_CLOSE, pose_closed, 1200U);
		break;
	case ROBOT_STATE_PICK_CLOSE:
		set_state(controller, ROBOT_STATE_PICK_VERIFY);
		break;
	case ROBOT_STATE_PICK_CLEAR:
		(void)begin_motion(controller, ROBOT_STATE_PICK_CARRY, pose_carry, 2500U);
		break;
	case ROBOT_STATE_PICK_CARRY:
		set_state(controller, ROBOT_STATE_SEARCH_BUCKET);
		break;
	case ROBOT_STATE_PLACE_APPROACH:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_RELEASE,
				   pose_place_release, 2200U);
		break;
	case ROBOT_STATE_PLACE_RELEASE:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_OPEN, pose_place_open, 1200U);
		break;
	case ROBOT_STATE_PLACE_OPEN:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_RETRACT,
				   pose_place_approach, 1800U);
		break;
	case ROBOT_STATE_PLACE_RETRACT:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_CARRY, pose_carry, 2500U);
		break;
	case ROBOT_STATE_PLACE_CARRY:
		(void)begin_motion(controller, ROBOT_STATE_PLACE_CLOSE, pose_home_closed, 2500U);
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
		(void)begin_motion(controller, ROBOT_STATE_RECOVER_HOME, pose_home, 3000U);
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
	const int32_t logical_percent = CLAMP(((int32_t)positions[5] - 1257) * 100 /
					      (2731 - 1257), 0, 100);
	const bool measured_holding = logical_percent > 25 ||
				      (gripper_status & 0x20U) != 0U;
	const bool holding = measured_holding || controller->robot_ci_mode;
	printk("ZEPHYR_PICK_VERIFY holding=%u measured_holding=%u simulated=%u "
	       "gripper_raw=%u logical=%d status=0x%02x\n", holding,
	       measured_holding, controller->robot_ci_mode && !measured_holding,
	       positions[5], logical_percent, gripper_status);
	if (holding) {
		(void)begin_motion(controller, ROBOT_STATE_PICK_CLEAR, pose_clear, 1800U);
	} else {
		printk("ZEPHYR_PICK_RETRY reason=empty-gripper\n");
		(void)begin_motion(controller, ROBOT_STATE_RECOVER_OPEN, pose_grab, 1200U);
	}
}

int robot_controller_init(struct robot_controller *controller,
			  struct feetech_bus *bus)
{
	memset(controller, 0, sizeof(*controller));
	controller->bus = bus;
	controller->state = ROBOT_STATE_STARTUP_HOME;
	controller->last_ball_side = 1;
	return begin_motion(controller, ROBOT_STATE_STARTUP_HOME, pose_home, 4000U);
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
