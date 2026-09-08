/* SPDX-License-Identifier: Apache-2.0 */

#include "feetech_bus.h"
#include "ivc_transport.h"
#include "perception_result_v2.h"
#include "resettable_watchdog.h"
#include "robot_config_receiver.h"
#include "robot_controller.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#define ROBOT_UART_NODE DT_ALIAS(robot_uart)
#define ARM_INTERPOLATION_PERIOD_MS 50U
#define IVC_IDLE_POLL_MS 1U
#define IVC_RECONNECT_IDLE_MS 1500U
#define STARTUP_SETTLE_MS 30000U
#define HEARTBEAT_MS 10000U
#define FEETECH_UART_BAUD 1000000U
#define SLOW_CONTROLLER_LOCK_MS 5U
#define SLOW_CONTROL_PROCESS_MS 20U

LOG_MODULE_REGISTER(robot_control_diag, LOG_LEVEL_DBG);

static const struct device *const robot_uart = DEVICE_DT_GET(ROBOT_UART_NODE);
static struct robot_controller controller;
static struct resettable_watchdog input_watchdog;
static struct k_timer arm_timer;
static struct k_work arm_work;
K_MUTEX_DEFINE(controller_lock);

static int64_t next_arm_deadline_ms;
static uint64_t arm_cycles;
static uint32_t max_arm_timer_late_ms;

BUILD_ASSERT(sizeof(struct perception_result_v2) == 48U);
BUILD_ASSERT(sizeof(struct robot_config_message_v1) == 48U);

static int receive_initial_config(struct robot_ivc *ivc,
				  struct robot_config_receiver *receiver,
				  struct robot_runtime_config_v1 *config)
{
	uint64_t idle_polls = 0U;

	printk("ZEPHYR_ROBOT_CONFIG_WAIT source=axvisor-ivc\n");
	for (;;) {
		struct robot_config_message_v1 message = {0};
		struct robot_config_message_v1 response = {0};
		size_t length = 0U;
		int status = robot_ivc_try_receive(ivc, &message, sizeof(message), &length);

		if (status == -EAGAIN) {
			if (++idle_polls % 1000U == 0U) {
				printk("ZEPHYR_ROBOT_CONFIG_WAIT polls=%llu\n", idle_polls);
			}
			k_msleep(IVC_IDLE_POLL_MS);
			continue;
		}
		if (status != 0) {
			printk("ZEPHYR_ROBOT_CONFIG_FAILED reason=receive status=%d\n",
			       status);
			return status;
		}
		if (!robot_config_is_message(&message, length)) {
			printk("ZEPHYR_ROBOT_CONFIG_DROP reason=expected-config len=%zu\n",
			       length);
			continue;
		}

		bool applied = false;
		robot_config_receiver_handle(receiver, &message, &response, config,
					     &applied);
		if (applied) {
			robot_config_receiver_mark_applied(receiver);
		}
		status = robot_ivc_send(ivc, &response, sizeof(response), 500);
		if (status != 0) {
			printk("ZEPHYR_ROBOT_CONFIG_FAILED reason=ack-send chunk=%u "
			       "status=%d\n", message.chunk_index, status);
			return status;
		}
		if (response.type == ROBOT_CONFIG_REJECTED) {
			printk("ZEPHYR_ROBOT_CONFIG_REJECTED session=%u chunk=%u "
			       "status=%u expected=%u\n", response.session_id,
			       response.chunk_index, response.status,
			       response.chunk_count);
		} else if (applied) {
			printk("ZEPHYR_ROBOT_CONFIG_APPLIED session=%u chunks=%u "
			       "crc=0x%08x speed_level=%u\n", response.session_id,
			       response.chunk_count, response.config_crc32,
			       config->motion_speed_level);
			return 0;
		} else {
			printk("ZEPHYR_ROBOT_CONFIG_ACK session=%u chunk=%u "
			       "next=%u\n", response.session_id,
			       response.chunk_index, response.chunk_count);
		}
	}
}

static void input_watchdog_handler(void *context)
{
	struct robot_controller *robot = context;
	const int64_t now_ms = k_uptime_get();
	const int64_t lock_started_ms = now_ms;

	LOG_DBG("ZEPHYR_DIAG component=watchdog event=handler-enter uptime_ms=%lld",
		now_ms);
	k_mutex_lock(&controller_lock, K_FOREVER);
	const int64_t lock_wait_ms = k_uptime_get() - lock_started_ms;
	const bool expired = robot_controller_input_timeout(robot, now_ms);
	if (!expired && robot->have_input) {
		/* An expiry work item can already be queued when a new frame resets
		 * the timer.  Recompute the remaining time so that this stale work
		 * item cannot cancel the new frame's deadline.  This also makes an
		 * early, tick-rounded callback harmless.
		 */
		const int64_t elapsed_ms = MAX(now_ms - robot->last_input_ms, 0);
		const uint32_t remaining_ms =
			ROBOT_INPUT_TIMEOUT_MS -
			(uint32_t)MIN(elapsed_ms, ROBOT_INPUT_TIMEOUT_MS - 1U);
		resettable_watchdog_restart(&input_watchdog, remaining_ms);
	}
	k_mutex_unlock(&controller_lock);
	LOG_DBG("ZEPHYR_DIAG component=watchdog event=handler-exit expired=%u "
		"lock_wait_ms=%lld elapsed_ms=%lld",
		expired, lock_wait_ms, k_uptime_get() - now_ms);
}

static void arm_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	const int64_t now_ms = k_uptime_get();
	k_mutex_lock(&controller_lock, K_FOREVER);
	if (now_ms > next_arm_deadline_ms) {
		max_arm_timer_late_ms =
			MAX(max_arm_timer_late_ms,
			    (uint32_t)(now_ms - next_arm_deadline_ms));
	}
	do {
		next_arm_deadline_ms += ARM_INTERPOLATION_PERIOD_MS;
	} while (next_arm_deadline_ms <= now_ms);

	robot_controller_arm_tick(&controller, now_ms);
	++arm_cycles;
	k_mutex_unlock(&controller_lock);
}

static void arm_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	(void)k_work_submit(&arm_work);
}

static bool valid_result(const struct perception_result_v2 *result)
{
	return result->magic == PERCEPTION_MAGIC &&
	       result->version == PERCEPTION_VERSION &&
	       result->frame_width > 0U && result->frame_height > 0U;
}

int main(void)
{
	struct feetech_bus bus = {0};
	struct robot_ivc ivc = {0};
	struct robot_config_receiver config_receiver;
	struct robot_runtime_config_v1 runtime_config;
	uint16_t initial_arm[FEETECH_ARM_COUNT];
	uint64_t received = 0U;
	uint64_t processed = 0U;
	uint64_t invalid = 0U;
	uint64_t heartbeat = 0U;
	int64_t next_heartbeat_ms;
	bool status_window_active = false;
	int64_t status_window_started_ms = 0;
	uint64_t status_window_started_received = 0U;
	uint64_t status_window_started_processed = 0U;
	int status;

	printk("ZEPHYR_ROBOT_CONTROL_START protocol=2 source=axvisor-ivc uart=uart6\n");
	printk("ZEPHYR_TIMER_READY driver=arm_arch_timer tick_hz=%u\n",
	       CONFIG_SYS_CLOCK_TICKS_PER_SEC);
	printk("ZEPHYR_ROBOT_CONTROL_WAIT settle_ms=%u\n", STARTUP_SETTLE_MS);
	const int64_t startup_started_ms = k_uptime_get();
	k_msleep(STARTUP_SETTLE_MS);
	const int64_t startup_elapsed_ms = k_uptime_get() - startup_started_ms;
	printk("ZEPHYR_TIMER_WAKE phase=startup requested_ms=%u actual_ms=%lld\n",
	       STARTUP_SETTLE_MS, startup_elapsed_ms);

	status = feetech_bus_init(&bus, robot_uart);
	if (status != 0) {
		printk("ZEPHYR_ROBOT_CONTROL_FAILED reason=uart6-not-ready status=%d\n",
		       status);
		return 1;
	}
	const struct uart_config config = {
		/* 16 MHz input / (16 * DLL=1) = 1 Mbps. */
		.baudrate = FEETECH_UART_BAUD,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	status = uart_configure(robot_uart, &config);
	if (status != 0) {
		printk("ZEPHYR_ROBOT_CONTROL_FAILED reason=uart6-config status=%d\n",
		       status);
		return 1;
	}
	printk("ZEPHYR_UART6_READY clock_hz=16000000 baud=1000000 divisor=1 dlf=0\n");

	status = feetech_configure_wheels(&bus);
	if (status != 0) {
		return 1;
	}
	status = feetech_configure_arm(&bus, initial_arm);
	if (status != 0) {
		feetech_stop_wheels(&bus, "ARM_INIT_FAILED_STOP");
		printk("ZEPHYR_ROBOT_CONTROL_FAILED reason=arm-config status=%d\n", status);
		return 1;
	}
	status = robot_ivc_subscribe(&ivc);
	if (status != 0) {
		feetech_stop_wheels(&bus, "IVC_FAILED_STOP");
		printk("ZEPHYR_ROBOT_CONTROL_FAILED reason=ivc-subscribe status=%d\n", status);
		return 1;
	}
	robot_config_receiver_init(&config_receiver);
	status = receive_initial_config(&ivc, &config_receiver, &runtime_config);
	if (status != 0) {
		feetech_stop_wheels(&bus, "CONFIG_FAILED_STOP");
		feetech_arm_torque_off(&bus);
		(void)robot_ivc_close(&ivc);
		return 1;
	}
	status = robot_controller_init(&controller, &bus, &runtime_config);
	if (status != 0) {
		feetech_stop_wheels(&bus, "CONTROLLER_INIT_FAILED_STOP");
		(void)robot_ivc_close(&ivc);
		return 1;
	}
	resettable_watchdog_init(&input_watchdog, input_watchdog_handler,
				 &controller);
	k_work_init(&arm_work, arm_work_handler);
	k_timer_init(&arm_timer, arm_timer_expiry, NULL);
	next_arm_deadline_ms = k_uptime_get() + ARM_INTERPOLATION_PERIOD_MS;
	k_timer_start(&arm_timer, K_MSEC(ARM_INTERPOLATION_PERIOD_MS),
		      K_MSEC(ARM_INTERPOLATION_PERIOD_MS));
	printk("ZEPHYR_ROBOT_CONTROL_READY source=axvisor-ivc arm_period_ms=%u "
	       "ivc_poll_ms=%u watchdog_ms=%u watchdog=resettable-one-shot\n",
	       ARM_INTERPOLATION_PERIOD_MS, IVC_IDLE_POLL_MS,
	       ROBOT_INPUT_TIMEOUT_MS);
	next_heartbeat_ms = k_uptime_get() + HEARTBEAT_MS;

	for (;;) {
		struct perception_result_v2 newest;
		bool have_newest = false;
		bool transport_disconnected = false;
		const uint64_t received_before_poll = received;
		const uint64_t processed_before_poll = processed;

		for (;;) {
			union {
				struct perception_result_v2 perception;
				struct robot_config_message_v1 config;
				uint8_t bytes[ROBOT_CONFIG_MESSAGE_SIZE];
			} message = {0};
			size_t length = 0U;
			status = robot_ivc_try_receive(&ivc, message.bytes,
					       sizeof(message.bytes),
					       &length);
			if (status == -EAGAIN) {
				break;
			}
			if (status != 0) {
				++invalid;
				printk("ZEPHYR_IVC_DROP status=%d len=%zu invalid=%llu\n",
				       status, length,
				       invalid);
				transport_disconnected = true;
				break;
			}
			if (robot_config_is_message(message.bytes, length)) {
				struct robot_config_message_v1 response = {0};
				struct robot_runtime_config_v1 candidate = {0};
				bool applied = false;
				bool ready = true;

				if (message.config.type == ROBOT_CONFIG_BEGIN) {
					k_mutex_lock(&controller_lock, K_FOREVER);
					ready = robot_controller_prepare_config_update(&controller);
					k_mutex_unlock(&controller_lock);
				}
				if (!ready) {
					robot_config_make_rejected(
						&message.config, &response,
						ROBOT_CONFIG_STATUS_BUSY,
						config_receiver.expected_chunk);
				} else {
					robot_config_receiver_handle(
						&config_receiver, &message.config, &response,
						&candidate, &applied);
				}
				if (applied) {
					k_mutex_lock(&controller_lock, K_FOREVER);
					status = robot_controller_apply_config(&controller,
									       &candidate);
					k_mutex_unlock(&controller_lock);
					if (status != 0) {
						robot_config_make_rejected(
							&message.config, &response,
							ROBOT_CONFIG_STATUS_BUSY,
								config_receiver.expected_chunk);
						applied = false;
					} else {
						robot_config_receiver_mark_applied(
							&config_receiver);
					}
				}
				status = robot_ivc_send(&ivc, &response,
							sizeof(response), 500);
				if (status != 0) {
					printk("ZEPHYR_ROBOT_CONFIG_FAILED reason=ack-send "
					       "chunk=%u status=%d\n",
					       message.config.chunk_index, status);
				} else if (applied) {
					printk("ZEPHYR_ROBOT_CONFIG_APPLIED session=%u "
					       "chunks=%u crc=0x%08x speed_level=%u "
					       "restart=1\n", response.session_id,
					       response.chunk_count, response.config_crc32,
					       candidate.motion_speed_level);
				} else if (response.type == ROBOT_CONFIG_REJECTED) {
					printk("ZEPHYR_ROBOT_CONFIG_REJECTED session=%u "
					       "chunk=%u status=%u expected=%u\n",
					       response.session_id, response.chunk_index,
					       response.status, response.chunk_count);
				} else {
					printk("ZEPHYR_ROBOT_CONFIG_ACK session=%u chunk=%u "
					       "next=%u restart=1\n", response.session_id,
					       response.chunk_index, response.chunk_count);
				}
				continue;
			}
			if (length != sizeof(message.perception) ||
			    !valid_result(&message.perception)) {
				++invalid;
				printk("ZEPHYR_IVC_DROP status=0 len=%zu payload_seq=%llu "
				       "invalid=%llu\n", length,
				       message.perception.sequence, invalid);
				continue;
			}
			newest = message.perception;
			have_newest = true;
			++received;
		}

		const int64_t now_ms = k_uptime_get();
		if (have_newest) {
			if (!status_window_active) {
				status_window_active = true;
				status_window_started_ms = now_ms;
				status_window_started_received = received_before_poll;
				status_window_started_processed = processed_before_poll;
			}
			const int64_t control_started_ms = k_uptime_get();
			k_mutex_lock(&controller_lock, K_FOREVER);
			const int64_t control_lock_wait_ms =
				k_uptime_get() - control_started_ms;
			robot_controller_process_perception(&controller, &newest,
						    now_ms);
			++processed;
			k_mutex_unlock(&controller_lock);
			const int64_t control_elapsed_ms =
				k_uptime_get() - control_started_ms;
			if (control_lock_wait_ms >= SLOW_CONTROLLER_LOCK_MS ||
			    control_elapsed_ms >= SLOW_CONTROL_PROCESS_MS) {
				LOG_DBG_RATELIMIT_RATE(
					1000,
					"ZEPHYR_DIAG component=control event=slow seq=%llu "
					"lock_wait_ms=%lld elapsed_ms=%lld",
					newest.sequence, control_lock_wait_ms,
					control_elapsed_ms);
			}
			resettable_watchdog_restart(&input_watchdog,
						 ROBOT_INPUT_TIMEOUT_MS);
			const uint64_t status_messages =
				received - status_window_started_received;
			if (CONFIG_ROBOT_STATUS_EVERY_MESSAGES > 0 &&
			    status_messages >= CONFIG_ROBOT_STATUS_EVERY_MESSAGES) {
				const uint64_t window_ms = (uint64_t)MAX(
					now_ms - status_window_started_ms, 1);
				const uint64_t status_processed =
					processed - status_window_started_processed;
				const uint64_t coalesced =
					status_messages - MIN(status_messages, status_processed);
				const uint64_t rx_fps_centi =
					status_messages * 100000U / window_ms;
				const uint64_t control_fps_centi =
					status_processed * 100000U / window_ms;

				k_mutex_lock(&controller_lock, K_FOREVER);
				printk("ZEPHYR_CONTROL_STATUS messages=%llu "
				       "window_s=%llu.%02llu rx_fps=%llu.%02llu "
				       "control_fps=%llu.%02llu coalesced=%llu "
				       "seq=%llu received=%llu processed=%llu invalid=%llu "
				       "state=%s arm_cycles=%llu max_arm_timer_late_ms=%u "
				       "frame=%ux%u ball_visible=%u "
				       "ball_confidence_milli=%u ball_center=%u,%u "
				       "ball_box=%ux%u bucket_visible=%u "
				       "bucket_center=%u,%u bucket_box=%ux%u\n",
				       status_messages, window_ms / 1000U,
				       (window_ms % 1000U) / 10U, rx_fps_centi / 100U,
				       rx_fps_centi % 100U, control_fps_centi / 100U,
				       control_fps_centi % 100U, coalesced, newest.sequence,
				       received, processed, invalid,
				       robot_controller_state_name(&controller), arm_cycles,
				       max_arm_timer_late_ms, newest.frame_width,
				       newest.frame_height,
				       (newest.flags & PERCEPTION_TARGET_VISIBLE) != 0U,
				       newest.confidence_milli, newest.center_x,
				       newest.center_y, newest.box_width, newest.box_height,
				       (newest.flags & PERCEPTION_BUCKET_VISIBLE) != 0U,
				       newest.bucket_center_x, newest.bucket_center_y,
				       newest.bucket_box_width, newest.bucket_box_height);
				k_mutex_unlock(&controller_lock);

				status_window_started_ms = now_ms;
				status_window_started_received = received;
				status_window_started_processed = processed;
			}
		}
		if (!have_newest) {
			bool reconnect = false;
			k_mutex_lock(&controller_lock, K_FOREVER);
			if ((transport_disconnected ||
			     (received > 0U &&
			      now_ms - controller.last_input_ms >= IVC_RECONNECT_IDLE_MS))) {
				reconnect = robot_controller_prepare_config_update(&controller);
			}
			k_mutex_unlock(&controller_lock);
			if (reconnect) {
				resettable_watchdog_stop(&input_watchdog);
				printk("ZEPHYR_IVC_RECONNECT reason=%s elapsed_ms=%lld\n",
				       transport_disconnected ? "transport-error" : "publisher-idle",
				       now_ms - controller.last_input_ms);
				(void)robot_ivc_close(&ivc);
				status = robot_ivc_subscribe(&ivc);
				if (status != 0) {
					printk("ZEPHYR_ROBOT_CONTROL_FAILED reason=ivc-reconnect "
					       "status=%d\n", status);
					return 1;
				}
				robot_config_receiver_init(&config_receiver);
				status = receive_initial_config(&ivc, &config_receiver,
							&runtime_config);
				if (status != 0) {
					printk("ZEPHYR_ROBOT_CONTROL_FAILED reason=config-reconnect "
					       "status=%d\n", status);
					return 1;
				}
				k_mutex_lock(&controller_lock, K_FOREVER);
				status = robot_controller_apply_config(&controller,
							       &runtime_config);
				k_mutex_unlock(&controller_lock);
				if (status != 0) {
					printk("ZEPHYR_ROBOT_CONTROL_FAILED reason=config-apply-reconnect "
					       "status=%d\n", status);
					return 1;
				}
				printk("ZEPHYR_IVC_RECONNECTED config_crc=0x%08x\n",
				       robot_config_crc32(&runtime_config,
						  sizeof(runtime_config)));
				status_window_active = false;
				continue;
			}
		}
		if (IS_ENABLED(CONFIG_ROBOT_CONTROL_ALIVE_LOG) &&
		    k_uptime_get() >= next_heartbeat_ms) {
			++heartbeat;
			k_mutex_lock(&controller_lock, K_FOREVER);
			printk("ZEPHYR_ROBOT_CONTROL_ALIVE heartbeat=%llu state=%s "
			       "pick_cycles=%llu received=%llu processed=%llu invalid=%llu "
			       "arm_cycles=%llu max_arm_timer_late_ms=%u\n", heartbeat,
			       robot_controller_state_name(&controller),
			       controller.completed_cycles, received, processed, invalid,
			       arm_cycles, max_arm_timer_late_ms);
			k_mutex_unlock(&controller_lock);
			next_heartbeat_ms = k_uptime_get() + HEARTBEAT_MS;
		}

		/* The current AxVisor IVC transport exposes a shared ring rather than
		 * a Zephyr waitable object.  Poll it at 1 ms while idle; this bounds
		 * receive latency without tying it to the 50 ms arm timer.
		 */
		if (!have_newest) {
			k_msleep(IVC_IDLE_POLL_MS);
		} else {
			k_yield();
		}
	}
}
