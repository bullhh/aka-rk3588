/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ROBOT_FEETECH_BUS_H_
#define ROBOT_FEETECH_BUS_H_

#include <zephyr/device.h>

#include <stddef.h>
#include <stdint.h>

#define FEETECH_ARM_COUNT 6U

struct feetech_bus {
	const struct device *uart;
};

int feetech_bus_init(struct feetech_bus *bus, const struct device *uart);
int feetech_configure_wheels(struct feetech_bus *bus);
int feetech_configure_arm(struct feetech_bus *bus,
			 uint16_t current[FEETECH_ARM_COUNT]);
int feetech_read_arm(struct feetech_bus *bus,
		     uint16_t positions[FEETECH_ARM_COUNT],
		     uint8_t *gripper_status);
int feetech_send_arm_pose(struct feetech_bus *bus,
			 const uint16_t positions[FEETECH_ARM_COUNT]);
int feetech_send_diff_drive(struct feetech_bus *bus, int16_t left,
			    int16_t right, const char *label);
void feetech_stop_wheels(struct feetech_bus *bus, const char *label);
void feetech_arm_torque_off(struct feetech_bus *bus);

#endif /* ROBOT_FEETECH_BUS_H_ */
