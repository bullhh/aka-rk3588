/* SPDX-License-Identifier: Apache-2.0 */

#include "feetech_bus.h"
#include "perception_result_v2.h"
#include "resettable_watchdog.h"
#include "robot_controller.h"

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <errno.h>
#include <string.h>

static struct feetech_bus fake_bus;
static int wheel_commands;
static int arm_commands;
static int16_t last_left;
static int16_t last_right;
static uint16_t last_arm_pose[FEETECH_ARM_COUNT];

int feetech_send_diff_drive(struct feetech_bus *bus, int16_t left,
			    int16_t right, const char *label)
{
	zassert_equal(bus, &fake_bus);
	zassert_not_null(label);
	++wheel_commands;
	last_left = left;
	last_right = right;
	return 0;
}

void feetech_stop_wheels(struct feetech_bus *bus, const char *label)
{
	(void)feetech_send_diff_drive(bus, 0, 0, label);
}

int feetech_send_arm_pose(struct feetech_bus *bus,
			 const uint16_t positions[FEETECH_ARM_COUNT])
{
	zassert_equal(bus, &fake_bus);
	++arm_commands;
	memcpy(last_arm_pose, positions, sizeof(last_arm_pose));
	return 0;
}

int feetech_read_arm(struct feetech_bus *bus,
		     uint16_t positions[FEETECH_ARM_COUNT],
		     uint8_t *gripper_status)
{
	zassert_equal(bus, &fake_bus);
	memcpy(positions, last_arm_pose, sizeof(last_arm_pose));
	*gripper_status = 0U;
	return 0;
}

static void reset_fakes(void)
{
	wheel_commands = 0;
	arm_commands = 0;
	last_left = 0;
	last_right = 0;
	memset(last_arm_pose, 0, sizeof(last_arm_pose));
}

static struct perception_result_v2 visible_ball(uint16_t center_x,
						 uint16_t box_size)
{
	return (struct perception_result_v2) {
		.magic = PERCEPTION_MAGIC,
		.version = PERCEPTION_VERSION,
		.flags = PERCEPTION_TARGET_VISIBLE,
		.frame_width = 640U,
		.frame_height = 480U,
		.center_x = center_x,
		.center_y = 240U,
		.box_width = box_size,
		.box_height = box_size,
	};
}

ZTEST(robot_control, test_perception_drives_chassis_without_arm_tick)
{
	struct robot_controller controller = {
		.bus = &fake_bus,
		.state = ROBOT_STATE_SEARCH_BALL,
		.last_ball_side = 1,
	};
	struct perception_result_v2 result = visible_ball(100U, 50U);

	reset_fakes();
	robot_controller_process_perception(&controller, &result, 1000);
	zassert_equal(wheel_commands, 1);
	zassert_equal(last_left, -15);
	zassert_equal(last_right, 15);
	zassert_equal(arm_commands, 0);

	result.center_x = 540U;
	robot_controller_process_perception(&controller, &result, 1010);
	zassert_equal(wheel_commands, 2);
	zassert_equal(last_left, 15);
	zassert_equal(last_right, -15);
}

ZTEST(robot_control, test_arm_tick_only_advances_interpolation)
{
	struct robot_controller controller = {
		.bus = &fake_bus,
		.state = ROBOT_STATE_PICK_HOME,
		.motion = {
			.started_ms = 0,
			.duration_ms = 100U,
			.active = true,
		},
	};
	struct perception_result_v2 result = visible_ball(100U, 50U);

	reset_fakes();
	for (size_t index = 0; index < FEETECH_ARM_COUNT; ++index) {
		controller.motion.start[index] = 1000U;
		controller.motion.target[index] = 2000U;
	}

	robot_controller_process_perception(&controller, &result, 40);
	zassert_equal(wheel_commands, 0);
	zassert_equal(arm_commands, 0);

	robot_controller_arm_tick(&controller, 50);
	zassert_equal(arm_commands, 1);
	zassert_equal(last_arm_pose[0], 1500U);
	zassert_equal(wheel_commands, 0);
}

ZTEST(robot_control, test_input_timeout_stops_once_and_new_input_recovers)
{
	struct robot_controller controller = {
		.bus = &fake_bus,
		.state = ROBOT_STATE_SEARCH_BALL,
		.last_ball_side = 1,
	};
	struct perception_result_v2 result = visible_ball(320U, 50U);

	reset_fakes();
	robot_controller_process_perception(&controller, &result, 1000);
	zassert_equal(last_left, 65);
	zassert_equal(last_right, 65);
	zassert_false(robot_controller_input_timeout(
		&controller, 1000 + ROBOT_INPUT_TIMEOUT_MS - 1U));
	zassert_equal(wheel_commands, 1);

	zassert_true(robot_controller_input_timeout(
		&controller, 1000 + ROBOT_INPUT_TIMEOUT_MS));
	zassert_equal(wheel_commands, 2);
	zassert_equal(last_left, 0);
	zassert_equal(last_right, 0);
	zassert_false(robot_controller_input_timeout(&controller, 2000));
	zassert_equal(wheel_commands, 2);

	robot_controller_process_perception(&controller, &result, 2001);
	zassert_equal(wheel_commands, 3);
	zassert_equal(last_left, 65);
	zassert_equal(last_right, 65);
}

static K_SEM_DEFINE(watchdog_fired, 0, 1);
static int watchdog_fires;

static void test_watchdog_handler(void *context)
{
	int *fires = context;

	++*fires;
	k_sem_give(&watchdog_fired);
}

ZTEST(robot_control, test_watchdog_is_resettable_and_one_shot)
{
	struct resettable_watchdog watchdog;

	watchdog_fires = 0;
	k_sem_reset(&watchdog_fired);
	resettable_watchdog_init(&watchdog, test_watchdog_handler,
				 &watchdog_fires);

	resettable_watchdog_restart(&watchdog, 60U);
	k_msleep(30);
	resettable_watchdog_restart(&watchdog, 60U);
	zassert_equal(k_sem_take(&watchdog_fired, K_MSEC(40)), -EAGAIN,
		      "first deadline was not reset");
	zassert_equal(k_sem_take(&watchdog_fired, K_MSEC(80)), 0,
		      "reset deadline did not fire");
	zassert_equal(watchdog_fires, 1);

	k_msleep(80);
	zassert_equal(watchdog_fires, 1, "watchdog was periodic, not one-shot");
	resettable_watchdog_stop(&watchdog);
}

ZTEST_SUITE(robot_control, NULL, NULL, NULL, NULL, NULL);
