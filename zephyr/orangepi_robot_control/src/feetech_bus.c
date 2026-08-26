/* SPDX-License-Identifier: Apache-2.0 */

#include "feetech_bus.h"

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#define FEETECH_BROADCAST_ID 0xfeU
#define FEETECH_INST_READ 0x02U
#define FEETECH_INST_SYNC_WRITE 0x83U

#define FEETECH_P_COEFFICIENT 21U
#define FEETECH_D_COEFFICIENT 22U
#define FEETECH_I_COEFFICIENT 23U
#define FEETECH_OPERATING_MODE 33U
#define FEETECH_TORQUE_ENABLE 40U
#define FEETECH_ACCELERATION 41U
#define FEETECH_GOAL_POSITION 42U
#define FEETECH_GOAL_VELOCITY 46U
#define FEETECH_PRESENT_POSITION 56U

#define FEETECH_POSITION_MODE 0U
#define FEETECH_VELOCITY_MODE 1U
#define FEETECH_STATUS_OVERLOAD 0x20U
#define FEETECH_RX_TIMEOUT_MS 120U
#define FEETECH_TX_TIMEOUT_MS 20U
LOG_MODULE_REGISTER(feetech_diag, LOG_LEVEL_DBG);

static const uint8_t arm_ids[FEETECH_ARM_COUNT] = {1U, 2U, 3U, 4U, 5U, 6U};
static const uint8_t wheel_ids[] = {7U, 8U, 9U};

static size_t flush_input(struct feetech_bus *bus)
{
	uint8_t byte;
	size_t count = 0U;

	while (uart_poll_in(bus->uart, &byte) == 0) {
		++count;
	}
	return count;
}

static int wait_tx_idle(struct feetech_bus *bus)
{
	const int64_t deadline_ms = k_uptime_get() + FEETECH_TX_TIMEOUT_MS;

	while (k_uptime_get() < deadline_ms) {
		const int ready = uart_irq_tx_complete(bus->uart);

		if (ready > 0) {
			return 0;
		}
		if (ready < 0) {
			return ready;
		}
		k_busy_wait(10U);
	}
	LOG_DBG("ZEPHYR_DIAG component=feetech event=tx-idle-timeout "
		"timeout_ms=%u uptime_ms=%lld",
		FEETECH_TX_TIMEOUT_MS, k_uptime_get());
	return -ETIMEDOUT;
}

static int send_bytes(struct feetech_bus *bus, const uint8_t *bytes, size_t length)
{
	for (size_t index = 0; index < length; ++index) {
		const int result = wait_tx_idle(bus);

		if (result != 0) {
			return result;
		}
		/* This application binds UART6 to the ns16550 driver.  Wait until
		 * THRE/TEMT is observed before entering its unbounded poll_out loop.
		 */
		uart_poll_out(bus->uart, bytes[index]);
	}
	return 0;
}

static int send_packet(struct feetech_bus *bus, uint8_t id, uint8_t instruction,
		       const uint8_t *params, size_t param_count)
{
	uint8_t packet[32];
	uint16_t sum;

	if (param_count + 6U > sizeof(packet)) {
		return -EMSGSIZE;
	}
	packet[0] = 0xffU;
	packet[1] = 0xffU;
	packet[2] = id;
	packet[3] = (uint8_t)(param_count + 2U);
	packet[4] = instruction;
	memcpy(&packet[5], params, param_count);
	sum = id + packet[3] + instruction;
	for (size_t index = 0; index < param_count; ++index) {
		sum += params[index];
	}
	packet[5U + param_count] = (uint8_t)(~sum);
	return send_bytes(bus, packet, param_count + 6U);
}

static int read_byte_until(struct feetech_bus *bus, uint8_t *byte, int64_t deadline_ms)
{
	while (k_uptime_get() < deadline_ms) {
		if (uart_poll_in(bus->uart, byte) == 0) {
			return 0;
		}
		k_busy_wait(100U);
	}
	return -ETIMEDOUT;
}

static int receive_status(struct feetech_bus *bus, uint8_t expected_id,
			  uint8_t *params, size_t capacity, size_t *length,
			  uint8_t *status)
{
	const int64_t deadline_ms = k_uptime_get() + FEETECH_RX_TIMEOUT_MS;
	uint8_t previous = 0U;
	uint8_t byte = 0U;

	while (read_byte_until(bus, &byte, deadline_ms) == 0) {
		if (previous == 0xffU && byte == 0xffU) {
			break;
		}
		previous = byte;
	}
	if (previous != 0xffU || byte != 0xffU) {
		return -ETIMEDOUT;
	}

	uint8_t id;
	uint8_t packet_length;
	uint8_t error;
	if (read_byte_until(bus, &id, deadline_ms) != 0 ||
	    read_byte_until(bus, &packet_length, deadline_ms) != 0 ||
	    read_byte_until(bus, &error, deadline_ms) != 0) {
		return -ETIMEDOUT;
	}
	if (id != expected_id || packet_length < 2U ||
	    packet_length - 2U > capacity) {
		return -EPROTO;
	}

	const size_t param_count = packet_length - 2U;
	uint16_t sum = id + packet_length + error;
	for (size_t index = 0; index < param_count; ++index) {
		if (read_byte_until(bus, &params[index], deadline_ms) != 0) {
			return -ETIMEDOUT;
		}
		sum += params[index];
	}
	uint8_t checksum;
	if (read_byte_until(bus, &checksum, deadline_ms) != 0) {
		return -ETIMEDOUT;
	}
	if (checksum != (uint8_t)(~sum)) {
		return -EBADMSG;
	}
	*length = param_count;
	*status = error;
	return 0;
}

static int transact(struct feetech_bus *bus, uint8_t id, uint8_t instruction,
		    const uint8_t *params, size_t param_count, uint8_t *reply,
		    size_t capacity, size_t *reply_length, uint8_t allowed_status,
		    uint8_t *actual_status)
{
	uint8_t status = 0U;
	int result;
	(void)flush_input(bus);
	result = send_packet(bus, id, instruction, params, param_count);
	if (result != 0 || id == FEETECH_BROADCAST_ID) {
		return result;
	}
	result = receive_status(bus, id, reply, capacity, reply_length, &status);
	if (actual_status != NULL) {
		*actual_status = status;
	}
	if (result == 0 && (status & (uint8_t)~allowed_status) != 0U) {
		result = -EIO;
	}
	return result;
}

static int read_u16(struct feetech_bus *bus, uint8_t id, uint8_t address,
		    uint16_t *value, uint8_t allowed_status, uint8_t *status)
{
	const uint8_t params[] = {address, 2U};
	uint8_t reply[4];
	size_t reply_length = 0U;
	int result;

	result = transact(bus, id, FEETECH_INST_READ, params, sizeof(params), reply,
			sizeof(reply), &reply_length, allowed_status, status);
	if (result != 0 || reply_length != 2U) {
		return result != 0 ? result : -EPROTO;
	}
	*value = (uint16_t)reply[0] | ((uint16_t)reply[1] << 8);
	return 0;
}

static int sync_write_u16(struct feetech_bus *bus, uint8_t address,
			  const uint8_t *ids, const uint16_t *values, size_t count)
{
	uint8_t params[2U + 3U * FEETECH_ARM_COUNT];
	uint8_t reply[1];
	size_t reply_length = 0U;

	if (count > FEETECH_ARM_COUNT) {
		return -EINVAL;
	}
	params[0] = address;
	params[1] = 2U;
	for (size_t index = 0; index < count; ++index) {
		params[2U + index * 3U] = ids[index];
		params[3U + index * 3U] = (uint8_t)(values[index] & 0xffU);
		params[4U + index * 3U] = (uint8_t)(values[index] >> 8);
	}
	return transact(bus, FEETECH_BROADCAST_ID, FEETECH_INST_SYNC_WRITE, params,
			2U + count * 3U, reply, sizeof(reply), &reply_length, 0U, NULL);
}

static int sync_write_u8(struct feetech_bus *bus, uint8_t address,
			 const uint8_t *ids, uint8_t value, size_t count)
{
	uint8_t params[2U + 2U * FEETECH_ARM_COUNT];
	uint8_t reply[1];
	size_t reply_length = 0U;

	if (count > FEETECH_ARM_COUNT) {
		return -EINVAL;
	}
	params[0] = address;
	params[1] = 1U;
	for (size_t index = 0; index < count; ++index) {
		params[2U + index * 2U] = ids[index];
		params[3U + index * 2U] = value;
	}
	return transact(bus, FEETECH_BROADCAST_ID, FEETECH_INST_SYNC_WRITE, params,
			2U + count * 2U, reply, sizeof(reply), &reply_length, 0U, NULL);
}

static uint16_t encode_velocity(int32_t value)
{
	const uint32_t absolute = value < 0 ? (uint32_t)(-value) : (uint32_t)value;
	uint16_t magnitude = (uint16_t)MIN(absolute, 0x7fffU);

	return (value < 0 ? 0x8000U : 0U) | magnitude;
}

int feetech_bus_init(struct feetech_bus *bus, const struct device *uart)
{
	if (bus == NULL || uart == NULL || !device_is_ready(uart)) {
		return -ENODEV;
	}
	bus->uart = uart;
	return 0;
}

int feetech_read_arm(struct feetech_bus *bus,
		     uint16_t positions[FEETECH_ARM_COUNT], uint8_t *gripper_status)
{
	for (size_t index = 0; index < FEETECH_ARM_COUNT; ++index) {
		uint8_t status = 0U;
		const uint8_t allowed = index + 1U == FEETECH_ARM_COUNT ?
					FEETECH_STATUS_OVERLOAD : 0U;
		int result = read_u16(bus, arm_ids[index], FEETECH_PRESENT_POSITION,
				      &positions[index], allowed, &status);

		if (result != 0) {
			printk("ZEPHYR_FEETECH_ERROR op=read-position id=%u status=%d\n",
			       arm_ids[index], result);
			return result;
		}
		if (index + 1U == FEETECH_ARM_COUNT && gripper_status != NULL) {
			*gripper_status = status;
		}
	}
	return 0;
}

int feetech_configure_wheels(struct feetech_bus *bus)
{
	int result;

	feetech_stop_wheels(bus, "INIT_STOP");
	result = sync_write_u8(bus, FEETECH_TORQUE_ENABLE, wheel_ids, 0U,
			       ARRAY_SIZE(wheel_ids));
	result = result == 0 ? sync_write_u8(bus, FEETECH_OPERATING_MODE,
					     wheel_ids, FEETECH_VELOCITY_MODE,
					     ARRAY_SIZE(wheel_ids)) : result;
	result = result == 0 ? sync_write_u8(bus, FEETECH_ACCELERATION, wheel_ids,
					     80U, ARRAY_SIZE(wheel_ids)) : result;
	result = result == 0 ? sync_write_u8(bus, FEETECH_TORQUE_ENABLE, wheel_ids,
					     1U, ARRAY_SIZE(wheel_ids)) : result;
	if (result != 0) {
		feetech_stop_wheels(bus, "INIT_FAILED_STOP");
		printk("ZEPHYR_FEETECH_ERROR op=configure-wheels status=%d\n", result);
		return result;
	}
	feetech_stop_wheels(bus, "WHEELS_READY_STOP");
	printk("ZEPHYR_WHEELS_READY ids=7,8,9 mode=velocity acceleration=80\n");
	return 0;
}

void feetech_arm_torque_off(struct feetech_bus *bus)
{
	(void)sync_write_u8(bus, FEETECH_TORQUE_ENABLE, arm_ids, 0U,
			    FEETECH_ARM_COUNT);
}

int feetech_configure_arm(struct feetech_bus *bus,
			 uint16_t current[FEETECH_ARM_COUNT])
{
	int result = feetech_read_arm(bus, current, NULL);

	if (result != 0) {
		return result;
	}
	feetech_arm_torque_off(bus);
	result = sync_write_u8(bus, FEETECH_OPERATING_MODE, arm_ids,
			       FEETECH_POSITION_MODE, FEETECH_ARM_COUNT);
	result = result == 0 ? sync_write_u8(bus, FEETECH_P_COEFFICIENT, arm_ids,
					     16U, FEETECH_ARM_COUNT) : result;
	result = result == 0 ? sync_write_u8(bus, FEETECH_I_COEFFICIENT, arm_ids,
					     0U, FEETECH_ARM_COUNT) : result;
	result = result == 0 ? sync_write_u8(bus, FEETECH_D_COEFFICIENT, arm_ids,
					     32U, FEETECH_ARM_COUNT) : result;
	result = result == 0 ? sync_write_u8(bus, FEETECH_ACCELERATION, arm_ids,
					     80U, FEETECH_ARM_COUNT) : result;
	if (result != 0) {
		feetech_arm_torque_off(bus);
		printk("ZEPHYR_FEETECH_ERROR op=configure-arm status=%d\n", result);
		return result;
	}
	result = sync_write_u16(bus, FEETECH_GOAL_POSITION, arm_ids, current,
				FEETECH_ARM_COUNT);
	if (result != 0) {
		feetech_arm_torque_off(bus);
		return result;
	}
	k_msleep(20);
	result = sync_write_u8(bus, FEETECH_TORQUE_ENABLE, arm_ids, 1U,
			       FEETECH_ARM_COUNT);
	if (result != 0) {
		feetech_arm_torque_off(bus);
		return result;
	}
	printk("ZEPHYR_ARM_READY ids=1,2,3,4,5,6 seeded=current-position\n");
	return 0;
}

int feetech_send_arm_pose(struct feetech_bus *bus,
			 const uint16_t positions[FEETECH_ARM_COUNT])
{
	return sync_write_u16(bus, FEETECH_GOAL_POSITION, arm_ids, positions,
			      FEETECH_ARM_COUNT);
}

int feetech_send_diff_drive(struct feetech_bus *bus, int16_t left,
			    int16_t right, const char *label)
{
	const int32_t average = ((int32_t)left + right) / 2;
	const int32_t difference = (int32_t)right - left;
	const int32_t raw[] = {
		(-1355 * average + 995 * difference) / 100,
		(995 * difference) / 100,
		(1355 * average + 995 * difference) / 100,
	};
	uint16_t encoded[ARRAY_SIZE(wheel_ids)];

	for (size_t index = 0; index < ARRAY_SIZE(encoded); ++index) {
		encoded[index] = encode_velocity(CLAMP(raw[index], -3000, 3000));
	}
	int result = sync_write_u16(bus, FEETECH_GOAL_VELOCITY, wheel_ids, encoded,
				    ARRAY_SIZE(wheel_ids));
	printk("ZEPHYR_CONTROL command=%s diff=%d,%d wheels=%d,%d,%d status=%d\n",
	       label, left, right, (int)raw[0], (int)raw[1], (int)raw[2], result);
	return result;
}

void feetech_stop_wheels(struct feetech_bus *bus, const char *label)
{
	(void)feetech_send_diff_drive(bus, 0, 0, label);
}
