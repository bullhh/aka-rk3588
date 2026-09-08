#include "protocol/robot_runtime_config_v1.h"
#include "robot/lekiwi_runtime_config.hpp"
#include "robot_config_receiver.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <string>

namespace {

robot_config_message_v1 make_message(uint8_t type, uint32_t session,
                                     uint16_t chunks, uint32_t crc) {
    robot_config_message_v1 message{};
    message.magic = ROBOT_CONFIG_MESSAGE_MAGIC;
    message.version = ROBOT_CONFIG_MESSAGE_VERSION;
    message.type = type;
    message.session_id = session;
    message.chunk_count = chunks;
    message.config_size = sizeof(robot_runtime_config_v1);
    message.config_crc32 = crc;
    return message;
}

void expect_response(const robot_config_message_v1& response, uint8_t type,
                     uint16_t next) {
    assert(response.type == type);
    assert(response.status == ROBOT_CONFIG_STATUS_OK);
    assert(response.chunk_count == next);
}

} // namespace

int main() {
    robot_runtime_config_v1 config{};
    std::string error;
    const std::string root = AKA_TEST_SOURCE_DIR;
    assert(build_lekiwi_runtime_config(
        root + "/config/lekiwi_calibration.json",
        root + "/config/lekiwi_pick_config.txt", config, error));
    assert(config.magic == ROBOT_RUNTIME_CONFIG_MAGIC);
    assert(config.version == ROBOT_RUNTIME_CONFIG_VERSION);
    assert(config.size == sizeof(config));
    assert(config.motion_speed_level == 4U);
    assert(config.ball_target_size == 155U);
    assert(config.bucket_target_size == 380U);
    const uint16_t expected_poses[ROBOT_POSE_COUNT][ROBOT_JOINT_COUNT] = {
        {2197U, 1699U, 2250U, 3042U, 2048U, 2289U},
        {2197U, 1699U, 2250U, 3042U, 2048U, 1404U},
        {1937U, 2112U, 1891U, 3042U, 2048U, 2289U},
        {1937U, 2628U, 2183U, 2537U, 2048U, 2289U},
        {1937U, 2628U, 2183U, 2537U, 2048U, 1404U},
        {1937U, 2112U, 1891U, 3042U, 2048U, 1404U},
        {2048U, 1871U, 1489U, 2686U, 2052U, 1404U},
        {2009U, 2054U, 1408U, 3042U, 2048U, 1404U},
        {2009U, 2369U, 1691U, 2916U, 2048U, 1404U},
        {2009U, 2369U, 1691U, 2916U, 2048U, 2289U},
    };
    assert(std::memcmp(config.poses, expected_poses,
                       sizeof(expected_poses)) == 0);

    const uint32_t crc = robot_config_crc32(&config, sizeof(config));
    const uint16_t chunks = static_cast<uint16_t>(
        (sizeof(config) + ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE - 1U) /
        ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE);
    const uint32_t session = 0x12345678U;
    robot_config_receiver receiver{};
    robot_config_receiver_init(&receiver);
    robot_runtime_config_v1 applied_config{};
    robot_config_message_v1 response{};
    bool applied = false;

    robot_config_message_v1 begin = make_message(
        ROBOT_CONFIG_BEGIN, session, chunks, crc);
    robot_config_receiver_handle(&receiver, &begin, &response,
                                 &applied_config, &applied);
    expect_response(response, ROBOT_CONFIG_ACK, 0U);
    assert(!applied);

    const auto* bytes = reinterpret_cast<const uint8_t*>(&config);
    for (uint16_t chunk = 0; chunk < chunks; ++chunk) {
        robot_config_message_v1 message = make_message(
            ROBOT_CONFIG_CHUNK, session, chunks, crc);
        message.chunk_index = chunk;
        const size_t offset = static_cast<size_t>(chunk) *
                              ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE;
        message.payload_size = static_cast<uint16_t>(std::min(
            sizeof(config) - offset,
            static_cast<size_t>(ROBOT_CONFIG_CHUNK_PAYLOAD_SIZE)));
        std::memcpy(message.payload, bytes + offset, message.payload_size);
        robot_config_receiver_handle(&receiver, &message, &response,
                                     &applied_config, &applied);
        expect_response(response, ROBOT_CONFIG_ACK,
                        static_cast<uint16_t>(chunk + 1U));
        assert(!applied);

        robot_config_receiver_handle(&receiver, &message, &response,
                                     &applied_config, &applied);
        expect_response(response, ROBOT_CONFIG_ACK,
                        static_cast<uint16_t>(chunk + 1U));
        assert(!applied);
    }

    robot_config_message_v1 commit = make_message(
        ROBOT_CONFIG_COMMIT, session, chunks, crc);
    commit.chunk_index = chunks;
    robot_config_receiver_handle(&receiver, &commit, &response,
                                 &applied_config, &applied);
    expect_response(response, ROBOT_CONFIG_APPLIED, chunks);
    assert(applied);
    assert(std::memcmp(&config, &applied_config, sizeof(config)) == 0);
    assert(receiver.receiving);
    assert(receiver.applied_session_id == 0U);

    // A controller-side rejection must leave COMMIT retryable rather than
    // caching a false APPLIED response.
    applied = false;
    robot_config_receiver_handle(&receiver, &commit, &response,
                                 &applied_config, &applied);
    expect_response(response, ROBOT_CONFIG_APPLIED, chunks);
    assert(applied);
    assert(receiver.receiving);

    robot_config_receiver_mark_applied(&receiver);
    assert(!receiver.receiving);
    assert(receiver.applied_session_id == session);
    assert(receiver.applied_crc32 == crc);

    applied = false;
    robot_config_receiver_handle(&receiver, &commit, &response,
                                 &applied_config, &applied);
    expect_response(response, ROBOT_CONFIG_APPLIED, chunks);
    assert(!applied);

    std::cout << "robot runtime config PASS chunks=" << chunks
              << " crc=0x" << std::hex << crc << std::dec << '\n';
    return 0;
}
