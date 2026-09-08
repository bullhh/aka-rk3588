/* SPDX-License-Identifier: Apache-2.0 */

#ifndef AKA_RK3588_ROBOT_RUNTIME_CONFIG_V1_H_
#define AKA_RK3588_ROBOT_RUNTIME_CONFIG_V1_H_

#include <stddef.h>
#include <stdint.h>

#define ROBOT_RUNTIME_CONFIG_MAGIC 0x31474652U
#define ROBOT_RUNTIME_CONFIG_VERSION 1U
#define ROBOT_CONFIG_MESSAGE_MAGIC 0x31474643U
#define ROBOT_CONFIG_MESSAGE_VERSION 1U
#define ROBOT_CONFIG_MESSAGE_SIZE 48U
#define ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE 20U
#define ROBOT_POSE_COUNT 10U
#define ROBOT_JOINT_COUNT 6U

enum robot_pose_index {
	ROBOT_POSE_HOME = 0,
	ROBOT_POSE_HOME_CLOSED,
	ROBOT_POSE_PRE,
	ROBOT_POSE_GRAB,
	ROBOT_POSE_CLOSED,
	ROBOT_POSE_CLEAR,
	ROBOT_POSE_CARRY,
	ROBOT_POSE_PLACE_APPROACH,
	ROBOT_POSE_PLACE_RELEASE,
	ROBOT_POSE_PLACE_OPEN,
};

enum robot_config_message_type {
	ROBOT_CONFIG_BEGIN = 1,
	ROBOT_CONFIG_CHUNK = 2,
	ROBOT_CONFIG_COMMIT = 3,
	ROBOT_CONFIG_ACK = 4,
	ROBOT_CONFIG_APPLIED = 5,
	ROBOT_CONFIG_REJECTED = 6,
};

enum robot_config_status {
	ROBOT_CONFIG_STATUS_OK = 0,
	ROBOT_CONFIG_STATUS_INVALID_MESSAGE = 1,
	ROBOT_CONFIG_STATUS_INVALID_SIZE = 2,
	ROBOT_CONFIG_STATUS_OUT_OF_ORDER = 3,
	ROBOT_CONFIG_STATUS_CRC_MISMATCH = 4,
	ROBOT_CONFIG_STATUS_INVALID_CONFIG = 5,
	ROBOT_CONFIG_STATUS_WRONG_SESSION = 6,
	ROBOT_CONFIG_STATUS_BUSY = 7,
};

/* Resolved, hardware-ready values derived from the persistent calibration and
 * pick configuration files. All poses contain raw Feetech goals for IDs 1..6.
 */
struct robot_runtime_config_v1 {
	uint32_t magic;
	uint16_t version;
	uint16_t size;
	uint16_t poses[ROBOT_POSE_COUNT][ROBOT_JOINT_COUNT];
	uint16_t ball_center_tolerance;
	uint16_t ball_target_size;
	uint16_t ball_size_tolerance;
	uint16_t ball_stable_frames;
	uint16_t bucket_center_tolerance;
	uint16_t bucket_target_size;
	uint16_t bucket_size_tolerance;
	uint16_t bucket_stable_frames;
	uint16_t ball_search_speed;
	uint16_t ball_far_speed;
	uint16_t ball_near_speed;
	uint16_t ball_reverse_speed;
	uint16_t ball_turn_speed;
	uint16_t bucket_search_speed;
	uint16_t bucket_far_speed;
	uint16_t bucket_near_speed;
	uint16_t bucket_reverse_speed;
	uint16_t bucket_turn_speed;
	uint16_t motion_speed_level;
	uint16_t gripper_range_min;
	uint16_t gripper_range_max;
	uint16_t gripper_hold_percent;
	uint16_t reserved[4];
};

/* One fixed IVC slot. Configuration messages use stop-and-wait: every BEGIN,
 * CHUNK and COMMIT receives a response on the reverse ring before the sender
 * advances.
 */
struct robot_config_message_v1 {
	uint32_t magic;
	uint16_t version;
	uint8_t type;
	uint8_t status;
	uint32_t session_id;
	uint16_t chunk_index;
	uint16_t chunk_count;
	uint16_t payload_size;
	uint16_t reserved;
	uint32_t config_size;
	uint32_t config_crc32;
	uint8_t payload[ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE];
};

static inline uint32_t robot_config_crc32(const void *data, size_t size)
{
	const uint8_t *bytes = (const uint8_t *)data;
	uint32_t crc = 0xffffffffU;

	for (size_t index = 0; index < size; ++index) {
		crc ^= bytes[index];
		for (unsigned int bit = 0; bit < 8U; ++bit) {
			const uint32_t mask = 0U - (crc & 1U);
			crc = (crc >> 1U) ^ (0xedb88320U & mask);
		}
	}
	return ~crc;
}

#if defined(__cplusplus)
static_assert(sizeof(struct robot_runtime_config_v1) == 180U,
	      "robot runtime configuration wire size changed");
static_assert(sizeof(struct robot_config_message_v1) == ROBOT_CONFIG_MESSAGE_SIZE,
	      "robot configuration message must fill one IVC slot");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(struct robot_runtime_config_v1) == 180U,
	       "robot runtime configuration wire size changed");
_Static_assert(sizeof(struct robot_config_message_v1) == ROBOT_CONFIG_MESSAGE_SIZE,
	       "robot configuration message must fill one IVC slot");
#endif

#endif /* AKA_RK3588_ROBOT_RUNTIME_CONFIG_V1_H_ */
