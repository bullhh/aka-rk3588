/* SPDX-License-Identifier: Apache-2.0 */

#include "robot_config_receiver.h"

#include <string.h>

static uint16_t required_chunks(uint32_t size)
{
	return (uint16_t)((size + ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE - 1U) /
			  ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE);
}

static void make_response(const struct robot_config_message_v1 *message,
			  struct robot_config_message_v1 *response,
			  uint8_t type, uint8_t status, uint16_t expected)
{
	memset(response, 0, sizeof(*response));
	response->magic = ROBOT_CONFIG_MESSAGE_MAGIC;
	response->version = ROBOT_CONFIG_MESSAGE_VERSION;
	response->type = type;
	response->status = status;
	response->session_id = message->session_id;
	response->chunk_index = message->chunk_index;
	response->chunk_count = expected;
	response->config_size = message->config_size;
	response->config_crc32 = message->config_crc32;
}

void robot_config_make_rejected(
	const struct robot_config_message_v1 *message,
	struct robot_config_message_v1 *response,
	uint8_t status, uint16_t expected_chunk)
{
	make_response(message, response, ROBOT_CONFIG_REJECTED, status,
		      expected_chunk);
}

static bool valid_runtime_config(const struct robot_runtime_config_v1 *config)
{
	if (config->magic != ROBOT_RUNTIME_CONFIG_MAGIC ||
	    config->version != ROBOT_RUNTIME_CONFIG_VERSION ||
	    config->size != sizeof(*config) ||
	    config->motion_speed_level < 1U || config->motion_speed_level > 4U ||
	    config->ball_target_size < 50U || config->ball_target_size > 400U ||
	    config->ball_size_tolerance > 100U ||
	    config->ball_center_tolerance > 160U ||
	    config->ball_stable_frames == 0U || config->ball_stable_frames > 30U ||
	    config->bucket_target_size < 50U || config->bucket_target_size > 700U ||
	    config->bucket_center_tolerance < 5U ||
	    config->bucket_center_tolerance > 100U ||
	    config->bucket_stable_frames == 0U ||
	    config->bucket_stable_frames > 30U ||
	    config->gripper_range_min >= config->gripper_range_max ||
	    config->gripper_range_max > 4095U ||
	    config->gripper_hold_percent > 100U) {
		return false;
	}
	for (size_t pose = 0; pose < ROBOT_POSE_COUNT; ++pose) {
		for (size_t joint = 0; joint < ROBOT_JOINT_COUNT; ++joint) {
			if (config->poses[pose][joint] > 4095U) {
				return false;
			}
		}
		const uint16_t gripper = config->poses[pose][ROBOT_JOINT_COUNT - 1U];
		if (gripper < config->gripper_range_min ||
		    gripper > config->gripper_range_max) {
			return false;
		}
	}
	const uint16_t *speeds = &config->ball_search_speed;
	for (size_t index = 0; index < 10U; ++index) {
		if (speeds[index] == 0U || speeds[index] > 100U) {
			return false;
		}
	}
	return true;
}

void robot_config_receiver_init(struct robot_config_receiver *receiver)
{
	memset(receiver, 0, sizeof(*receiver));
}

bool robot_config_is_message(const void *payload, size_t length)
{
	if (payload == NULL || length != sizeof(struct robot_config_message_v1)) {
		return false;
	}
	const struct robot_config_message_v1 *message = payload;
	return message->magic == ROBOT_CONFIG_MESSAGE_MAGIC &&
	       message->version == ROBOT_CONFIG_MESSAGE_VERSION;
}

void robot_config_receiver_handle(
	struct robot_config_receiver *receiver,
	const struct robot_config_message_v1 *message,
	struct robot_config_message_v1 *response,
	struct robot_runtime_config_v1 *output,
	bool *applied)
{
	*applied = false;
	if (message->magic != ROBOT_CONFIG_MESSAGE_MAGIC ||
	    message->version != ROBOT_CONFIG_MESSAGE_VERSION) {
		make_response(message, response, ROBOT_CONFIG_REJECTED,
			      ROBOT_CONFIG_STATUS_INVALID_MESSAGE,
			      receiver->expected_chunk);
		return;
	}

	if (message->type == ROBOT_CONFIG_BEGIN) {
		const uint16_t chunks = required_chunks(message->config_size);
		if (message->config_size != sizeof(struct robot_runtime_config_v1) ||
		    message->chunk_count != chunks || chunks == 0U) {
			make_response(message, response, ROBOT_CONFIG_REJECTED,
				      ROBOT_CONFIG_STATUS_INVALID_SIZE, 0U);
			return;
		}
		memset(receiver->bytes, 0, sizeof(receiver->bytes));
		receiver->session_id = message->session_id;
		receiver->config_crc32 = message->config_crc32;
		receiver->chunk_count = chunks;
		receiver->expected_chunk = 0U;
		receiver->receiving = true;
		make_response(message, response, ROBOT_CONFIG_ACK,
			      ROBOT_CONFIG_STATUS_OK, 0U);
		return;
	}

	if (message->type == ROBOT_CONFIG_COMMIT &&
	    message->session_id == receiver->applied_session_id &&
	    message->config_crc32 == receiver->applied_crc32) {
		make_response(message, response, ROBOT_CONFIG_APPLIED,
			      ROBOT_CONFIG_STATUS_OK, message->chunk_count);
		return;
	}

	if (!receiver->receiving || message->session_id != receiver->session_id) {
		make_response(message, response, ROBOT_CONFIG_REJECTED,
			      ROBOT_CONFIG_STATUS_WRONG_SESSION,
			      receiver->expected_chunk);
		return;
	}

	if (message->type == ROBOT_CONFIG_CHUNK) {
		if (message->chunk_index > receiver->expected_chunk ||
		    message->chunk_index >= receiver->chunk_count) {
			make_response(message, response, ROBOT_CONFIG_REJECTED,
				      ROBOT_CONFIG_STATUS_OUT_OF_ORDER,
				      receiver->expected_chunk);
			return;
		}
		if (message->chunk_index < receiver->expected_chunk) {
			make_response(message, response, ROBOT_CONFIG_ACK,
				      ROBOT_CONFIG_STATUS_OK,
				      receiver->expected_chunk);
			return;
		}
		const size_t offset =
			(size_t)message->chunk_index * ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE;
		const size_t remaining = sizeof(receiver->bytes) - offset;
		const size_t expected_size =
			remaining < ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE ? remaining :
			ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE;
		if (message->payload_size != expected_size) {
			make_response(message, response, ROBOT_CONFIG_REJECTED,
				      ROBOT_CONFIG_STATUS_INVALID_SIZE,
				      receiver->expected_chunk);
			return;
		}
		memcpy(receiver->bytes + offset, message->payload, expected_size);
		++receiver->expected_chunk;
		make_response(message, response, ROBOT_CONFIG_ACK,
			      ROBOT_CONFIG_STATUS_OK, receiver->expected_chunk);
		return;
	}

	if (message->type == ROBOT_CONFIG_COMMIT) {
		if (receiver->expected_chunk != receiver->chunk_count) {
			make_response(message, response, ROBOT_CONFIG_REJECTED,
				      ROBOT_CONFIG_STATUS_OUT_OF_ORDER,
				      receiver->expected_chunk);
			return;
		}
		const uint32_t crc = robot_config_crc32(receiver->bytes,
						       sizeof(receiver->bytes));
		if (crc != receiver->config_crc32 || crc != message->config_crc32) {
			make_response(message, response, ROBOT_CONFIG_REJECTED,
				      ROBOT_CONFIG_STATUS_CRC_MISMATCH,
				      receiver->expected_chunk);
			return;
		}
		struct robot_runtime_config_v1 candidate;
		memcpy(&candidate, receiver->bytes, sizeof(candidate));
		if (!valid_runtime_config(&candidate)) {
			make_response(message, response, ROBOT_CONFIG_REJECTED,
				      ROBOT_CONFIG_STATUS_INVALID_CONFIG,
				      receiver->expected_chunk);
			return;
		}
		memcpy(output, &candidate, sizeof(*output));
		*applied = true;
		make_response(message, response, ROBOT_CONFIG_APPLIED,
			      ROBOT_CONFIG_STATUS_OK, receiver->expected_chunk);
		return;
	}

	make_response(message, response, ROBOT_CONFIG_REJECTED,
		      ROBOT_CONFIG_STATUS_INVALID_MESSAGE,
		      receiver->expected_chunk);
}

void robot_config_receiver_mark_applied(
	struct robot_config_receiver *receiver)
{
	receiver->applied_session_id = receiver->session_id;
	receiver->applied_crc32 = receiver->config_crc32;
	receiver->receiving = false;
}
