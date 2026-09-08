/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ROBOT_CONFIG_RECEIVER_H_
#define ROBOT_CONFIG_RECEIVER_H_

#include "robot_runtime_config_v1.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct robot_config_receiver {
	uint8_t bytes[sizeof(struct robot_runtime_config_v1)];
	uint32_t session_id;
	uint32_t config_crc32;
	uint16_t chunk_count;
	uint16_t expected_chunk;
	uint32_t applied_session_id;
	uint32_t applied_crc32;
	bool receiving;
};

void robot_config_receiver_init(struct robot_config_receiver *receiver);

bool robot_config_is_message(const void *payload, size_t length);

void robot_config_make_rejected(
	const struct robot_config_message_v1 *message,
	struct robot_config_message_v1 *response,
	uint8_t status, uint16_t expected_chunk);

/* Handle one configuration message and always produce an ACK/APPLIED/REJECTED
 * response. On a successful COMMIT, writes the validated configuration to
 * output and sets applied=true.
 */
void robot_config_receiver_handle(
	struct robot_config_receiver *receiver,
	const struct robot_config_message_v1 *message,
	struct robot_config_message_v1 *response,
	struct robot_runtime_config_v1 *output,
	bool *applied);

/* Record a validated COMMIT only after the controller accepted the candidate.
 * This makes a retried COMMIT truthful when hardware preparation fails.
 */
void robot_config_receiver_mark_applied(
	struct robot_config_receiver *receiver);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONFIG_RECEIVER_H_ */
