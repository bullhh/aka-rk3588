#pragma once

#include "protocol/robot_runtime_config_v1.h"

enum class IvcReply { Ignore, Accept, Reject };

inline IvcReply config_reply_policy(const robot_config_message_v1& reply, size_t size,
                                   const robot_config_message_v1& request,
                                   uint8_t expected_type, uint16_t expected_next) {
    if (size != sizeof(reply) || reply.magic != ROBOT_CONFIG_MESSAGE_MAGIC ||
        reply.version != ROBOT_CONFIG_MESSAGE_VERSION) return IvcReply::Reject;
    if (reply.session_id != request.session_id) return IvcReply::Ignore;
    if (reply.config_crc32 != request.config_crc32 ||
        reply.type == ROBOT_CONFIG_REJECTED || reply.status != ROBOT_CONFIG_STATUS_OK)
        return IvcReply::Reject;
    if (reply.type == ROBOT_CONFIG_ACK &&
        (reply.chunk_count < expected_next ||
         (expected_type == ROBOT_CONFIG_APPLIED && reply.chunk_count == expected_next)))
        return IvcReply::Ignore;
    if (reply.type != expected_type || reply.chunk_count != expected_next ||
        reply.chunk_index != request.chunk_index) return IvcReply::Reject;
    return IvcReply::Accept;
}
