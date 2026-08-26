#ifndef PERCEPTION_RESULT_HPP
#define PERCEPTION_RESULT_HPP

#include <algorithm>
#include <cstdint>
#include <cstdio>

#include "../protocol/perception_result_v2.h"

namespace perception {

constexpr uint32_t kResultMagic = PERCEPTION_MAGIC; // "PCR1" in little endian.
constexpr uint16_t kResultVersion = PERCEPTION_VERSION;

enum ResultFlags : uint16_t {
    kTargetVisible = PERCEPTION_TARGET_VISIBLE,
    kBucketVisible = PERCEPTION_BUCKET_VISIBLE,
    // Set only by --robot-ci-once.  The controller still receives results from
    // the real camera/NPU process, but may substitute otherwise unavailable
    // fixture feedback (for example, an object held by the gripper).
    kRobotCi = PERCEPTION_ROBOT_CI,
};

using PerceptionResultV2 = ::perception_result_v2;

static_assert(sizeof(PerceptionResultV2) == 48,
              "PerceptionResultV2 must exactly fill one AxVisor IVC slot");

inline uint16_t clamp_u16(int value) {
    return static_cast<uint16_t>(std::max(0, std::min(65535, value)));
}

inline void print_status(const PerceptionResultV2& result,
                         uint64_t results,
                         double window_seconds,
                         double inference_fps,
                         double ivc_fps,
                         int sent,
                         int dropped,
                         FILE* stream = stdout) {
    std::fprintf(
        stream,
        "STARRY_PERCEPTION_STATUS results=%llu window_s=%.2f "
        "inference_fps=%.2f ivc_fps=%.2f sent=%d dropped=%d "
        "seq=%llu frame=%ux%u "
        "ball_visible=%u ball_confidence_milli=%u ball_center=%u,%u "
        "ball_box=%ux%u bucket_visible=%u bucket_center=%u,%u "
        "bucket_box=%ux%u\n",
        static_cast<unsigned long long>(results), window_seconds,
        inference_fps,
        ivc_fps,
        sent,
        dropped,
        static_cast<unsigned long long>(result.sequence),
        static_cast<unsigned>(result.frame_width),
        static_cast<unsigned>(result.frame_height),
        (result.flags & kTargetVisible) != 0 ? 1U : 0U,
        static_cast<unsigned>(result.confidence_milli),
        static_cast<unsigned>(result.center_x),
        static_cast<unsigned>(result.center_y),
        static_cast<unsigned>(result.box_width),
        static_cast<unsigned>(result.box_height),
        (result.flags & kBucketVisible) != 0 ? 1U : 0U,
        static_cast<unsigned>(result.bucket_center_x),
        static_cast<unsigned>(result.bucket_center_y),
        static_cast<unsigned>(result.bucket_box_width),
        static_cast<unsigned>(result.bucket_box_height));
    std::fflush(stream);
}

} // namespace perception

#endif // PERCEPTION_RESULT_HPP
