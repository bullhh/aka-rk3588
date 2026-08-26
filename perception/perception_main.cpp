#include "perception_result.hpp"
#include "bucket_detector.hpp"

#include "capture/uvc_capture.hpp"
#include "detect/detect.hpp"
#include "logger.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <thread>
#include <time.h>
#include <turbojpeg.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kFrameWidth = 640;
constexpr int kFrameHeight = 480;
constexpr size_t kMjpegCapacity = 1024 * 1024;
constexpr int kIvcRetryCount = 5;
constexpr useconds_t kIvcRetryDelayUs = 1000;
// Keep crash-reproduction progress visible in the host-side serial capture.
// At roughly 25-30 FPS this emits one sampled stage trace per second.
constexpr uint64_t kDebugTraceEveryResults = 30;
constexpr uint64_t kSlowIvcWriteMs = 20;
constexpr uint64_t kRobotCiPerceptionMs = 20000;
constexpr uint64_t kRobotCiTotalMs = 62000;

std::atomic<bool> g_stop{false};
std::atomic<bool> g_diag_stop{false};
std::atomic<uint64_t> g_diag_attempt{0};
std::atomic<uint64_t> g_diag_inference{0};
std::atomic<uint64_t> g_diag_progress{0};

enum class DiagStage : int {
    Idle,
    Capture,
    Decode,
    Rknn,
    Result,
    Ivc,
};

std::atomic<DiagStage> g_diag_stage{DiagStage::Idle};

const char* diag_stage_name(DiagStage stage) {
    switch (stage) {
    case DiagStage::Idle: return "idle";
    case DiagStage::Capture: return "capture";
    case DiagStage::Decode: return "decode";
    case DiagStage::Rknn: return "rknn";
    case DiagStage::Result: return "result";
    case DiagStage::Ivc: return "ivc";
    }
    return "unknown";
}

static_assert(sizeof(perception::PerceptionResultV2) == 48,
              "PerceptionResultV2 must exactly fill the AxVisor IVC slot");

class IvcPublisher {
public:
    IvcPublisher() = default;
    ~IvcPublisher() {
        if (fd_ >= 0) close(fd_);
    }

    IvcPublisher(const IvcPublisher&) = delete;
    IvcPublisher& operator=(const IvcPublisher&) = delete;

    bool open_device(const char* path) {
        fd_ = open(path, O_WRONLY | O_CLOEXEC);
        if (fd_ < 0) {
            LOGE("Failed to open AxVisor IVC publisher %s: %s", path, std::strerror(errno));
            return false;
        }
        return true;
    }

    bool send(const perception::PerceptionResultV2& result, bool& dropped) {
        dropped = false;
        for (int attempt = 0; attempt < kIvcRetryCount; ++attempt) {
            const ssize_t written = write(fd_, &result, sizeof(result));
            if (written == static_cast<ssize_t>(sizeof(result))) return true;
            // The StarryOS axivc device reports a full publisher ring as a
            // zero-length write.  This is expected while the subscriber is
            // still completing its cold-boot retry, so handle it exactly like
            // EAGAIN: retry briefly, then drop this perception frame without
            // terminating the long-running publisher.
            if (written == 0 ||
                (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
                usleep(kIvcRetryDelayUs);
                continue;
            }
            LOGE("AxVisor IVC send failed at seq=%llu: %s",
                 static_cast<unsigned long long>(result.sequence),
                 written < 0 ? std::strerror(errno) : "short write");
            return false;
        }

        dropped = true;
        return true;
    }

private:
    int fd_ = -1;
};

void signal_handler(int) {
    g_stop.store(true);
}

uint64_t monotonic_ms() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<uint64_t>(now.tv_sec) * 1000ULL +
           static_cast<uint64_t>(now.tv_nsec) / 1000000ULL;
}

bool parse_nonnegative(const char* value, int& output) {
    errno = 0;
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed < 0 || parsed > 100000000L) {
        return false;
    }
    output = static_cast<int>(parsed);
    return true;
}

bool parse_positive_double(const char* value, double& output) {
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (errno != 0 || end == value || *end != '\0' || parsed <= 0.0 || parsed > 1000.0) {
        return false;
    }
    output = parsed;
    return true;
}

void usage(const char* program) {
    std::fprintf(
        stderr,
        "Usage: %s <model.rknn> [uvc-index] [--max-results N] [--report-every N]\n"
        "       [--status-every N] [--pipeline-heartbeat 0|1]\n"
        "       [--transport stdout|ivc] [--ivc-device PATH]\n"
        "       [--robot-ci-once] [--min-fps N]\n"
        "  --max-results 0 keeps the perception loop running (default).\n"
        "  --report-every N emits one result per N inferences (default 1).\n"
        "  --status-every N prints rate and the latest detection in one status\n"
        "    line every N emitted results; 0 disables it (default 60).\n"
        "  --pipeline-heartbeat 1 prints a pipeline diagnostic every second;\n"
        "    0 disables the diagnostic thread (default 0).\n"
        "  --transport ivc publishes each result through /dev/axivc.\n"
        "  --robot-ci-once measures real perception for 20 seconds, then\n"
        "    publishes a deterministic raised-wheel pick/place validation.\n"
        "  --min-fps N sets the robot-CI perception threshold (default 15).\n",
        program);
}

int decode_mjpeg(const uint8_t* jpeg,
                 size_t jpeg_length,
                 uint8_t* output,
                 int output_width,
                 int output_height,
                 int& pad_x,
                 int& pad_y,
                 float& scale) {
    tjhandle decoder = tjInitDecompress();
    if (decoder == nullptr) return -1;

    int source_width = 0;
    int source_height = 0;
    int subsampling = 0;
    int colorspace = 0;
    if (tjDecompressHeader3(decoder,
                            jpeg,
                            jpeg_length,
                            &source_width,
                            &source_height,
                            &subsampling,
                            &colorspace) < 0) {
        tjDestroy(decoder);
        return -1;
    }

    scale = std::min(output_width / static_cast<float>(source_width),
                     output_height / static_cast<float>(source_height));
    const int scaled_width = std::max(1, static_cast<int>(source_width * scale));
    const int scaled_height = std::max(1, static_cast<int>(source_height * scale));
    pad_x = (output_width - scaled_width) / 2;
    pad_y = (output_height - scaled_height) / 2;

    std::vector<uint8_t> scaled(static_cast<size_t>(scaled_width) * scaled_height * 3);
    const int result = tjDecompress2(decoder,
                                     jpeg,
                                     jpeg_length,
                                     scaled.data(),
                                     scaled_width,
                                     0,
                                     scaled_height,
                                     TJPF_RGB,
                                     TJFLAG_FASTDCT);
    tjDestroy(decoder);
    if (result < 0) return -1;

    std::memset(output, 114, static_cast<size_t>(output_width) * output_height * 3);
    for (int row = 0; row < scaled_height; ++row) {
        std::memcpy(output + (static_cast<size_t>(row + pad_y) * output_width + pad_x) * 3,
                    scaled.data() + static_cast<size_t>(row) * scaled_width * 3,
                    static_cast<size_t>(scaled_width) * 3);
    }
    return 0;
}

perception::PerceptionResultV2 make_result(
    uint64_t sequence,
    const std::vector<detection>& detections,
    const perception::BucketDetection& bucket) {
    perception::PerceptionResultV2 result;
    result.sequence = sequence;
    result.monotonic_ms = monotonic_ms();
    result.frame_width = kFrameWidth;
    result.frame_height = kFrameHeight;
    if (bucket.visible) {
        result.flags |= perception::kBucketVisible;
        result.bucket_center_x = perception::clamp_u16(bucket.center_x);
        result.bucket_center_y = perception::clamp_u16(bucket.center_y);
        result.bucket_box_width = perception::clamp_u16(bucket.width);
        result.bucket_box_height = perception::clamp_u16(bucket.height);
    }
    if (detections.empty()) return result;

    const detection* best = &detections.front();
    for (const auto& candidate : detections) {
        if (candidate.score > best->score) best = &candidate;
    }

    result.flags |= perception::kTargetVisible;
    result.confidence_milli = perception::clamp_u16(
        static_cast<int>(std::max(0.0f, std::min(1.0f, best->score)) * 1000.0f));
    result.center_x = perception::clamp_u16(static_cast<int>(best->bbox.x));
    result.center_y = perception::clamp_u16(static_cast<int>(best->bbox.y));
    result.box_width = perception::clamp_u16(static_cast<int>(best->bbox.w));
    result.box_height = perception::clamp_u16(static_cast<int>(best->bbox.h));
    return result;
}

const char* apply_robot_ci_scene(perception::PerceptionResultV2& result,
                                 uint64_t script_ms) {
    result.flags = perception::kRobotCi;
    result.confidence_milli = 1000;
    result.center_x = kFrameWidth / 2;
    result.center_y = kFrameHeight / 2;
    result.box_width = 0;
    result.box_height = 0;
    result.bucket_center_x = kFrameWidth / 2;
    result.bucket_center_y = kFrameHeight / 2;
    result.bucket_box_width = 0;
    result.bucket_box_height = 0;

    if (script_ms < 1000) return "ball-missing";
    result.flags |= perception::kTargetVisible;
    result.box_width = 80;
    result.box_height = 80;
    if (script_ms < 2000) {
        result.center_x = 180;
        return "ball-left";
    }
    if (script_ms < 3000) {
        result.center_x = 460;
        return "ball-right";
    }
    if (script_ms < 4000) return "ball-forward";
    if (script_ms < 5000) {
        result.box_width = 180;
        result.box_height = 180;
        return "ball-reverse";
    }

    result.box_width = 155;
    result.box_height = 155;
    if (script_ms < 19000) return "ball-ready-arm-pick";

    result.flags &= static_cast<uint16_t>(~perception::kTargetVisible);
    if (script_ms < 20000) return "bucket-missing";
    result.flags |= perception::kBucketVisible;
    result.bucket_box_width = 200;
    result.bucket_box_height = 200;
    if (script_ms < 21000) {
        result.bucket_center_x = 180;
        return "bucket-left";
    }
    if (script_ms < 22000) {
        result.bucket_center_x = 460;
        return "bucket-right";
    }
    if (script_ms < 23000) return "bucket-forward";
    if (script_ms < 24000) {
        result.bucket_box_width = 420;
        result.bucket_box_height = 420;
        return "bucket-reverse";
    }
    result.bucket_box_width = 380;
    result.bucket_box_height = 380;
    return "bucket-ready-arm-place";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }

    const char* model_path = argv[1];
    int uvc_index = 0;
    int max_results = 0;
    int report_every = 1;
    int status_every = 60;
    int pipeline_heartbeat = 0;
    bool robot_ci_once = false;
    double robot_ci_min_fps = 15.0;
    std::string transport = "stdout";
    std::string ivc_device = "/dev/axivc";
    int argument = 2;
    if (argument < argc && argv[argument][0] != '-') {
        if (!parse_nonnegative(argv[argument], uvc_index)) {
            LOGE("Invalid UVC index: %s", argv[argument]);
            return 2;
        }
        ++argument;
    }
    while (argument < argc) {
        if (std::strcmp(argv[argument], "--max-results") == 0 && argument + 1 < argc) {
            if (!parse_nonnegative(argv[argument + 1], max_results)) {
                LOGE("Invalid --max-results value: %s", argv[argument + 1]);
                return 2;
            }
            argument += 2;
        } else if (std::strcmp(argv[argument], "--report-every") == 0 && argument + 1 < argc) {
            if (!parse_nonnegative(argv[argument + 1], report_every) || report_every == 0) {
                LOGE("Invalid --report-every value: %s", argv[argument + 1]);
                return 2;
            }
            argument += 2;
        } else if (std::strcmp(argv[argument], "--status-every") == 0 &&
                   argument + 1 < argc) {
            if (!parse_nonnegative(argv[argument + 1], status_every)) {
                LOGE("Invalid --status-every value: %s", argv[argument + 1]);
                return 2;
            }
            argument += 2;
        } else if (std::strcmp(argv[argument], "--pipeline-heartbeat") == 0 &&
                   argument + 1 < argc) {
            if (!parse_nonnegative(argv[argument + 1], pipeline_heartbeat) ||
                pipeline_heartbeat > 1) {
                LOGE("Invalid --pipeline-heartbeat value: %s", argv[argument + 1]);
                return 2;
            }
            argument += 2;
        } else if (std::strcmp(argv[argument], "--transport") == 0 && argument + 1 < argc) {
            transport = argv[argument + 1];
            if (transport != "stdout" && transport != "ivc") {
                LOGE("Invalid --transport value: %s", transport.c_str());
                return 2;
            }
            argument += 2;
        } else if (std::strcmp(argv[argument], "--ivc-device") == 0 && argument + 1 < argc) {
            ivc_device = argv[argument + 1];
            argument += 2;
        } else if (std::strcmp(argv[argument], "--robot-ci-once") == 0) {
            robot_ci_once = true;
            ++argument;
        } else if (std::strcmp(argv[argument], "--min-fps") == 0 && argument + 1 < argc) {
            if (!parse_positive_double(argv[argument + 1], robot_ci_min_fps)) {
                LOGE("Invalid --min-fps value: %s", argv[argument + 1]);
                return 2;
            }
            argument += 2;
        } else {
            LOGE("Unknown argument: %s", argv[argument]);
            usage(argv[0]);
            return 2;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    IvcPublisher ivc;
    if (transport == "ivc" && !ivc.open_device(ivc_device.c_str())) return 1;

    rknn_app_context_t rknn_context{};
    if (detect_init(model_path, &rknn_context) != 0) {
        LOGE("Failed to load model: %s", model_path);
        return 1;
    }

    UvcCapture capture;
    if (capture.open(uvc_index, kFrameWidth, kFrameHeight, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index);
        detect_deinit(&rknn_context);
        return 1;
    }

    std::unique_ptr<uint8_t[]> mjpeg(new uint8_t[kMjpegCapacity]);
    const size_t rgb_size = static_cast<size_t>(rknn_context.model_width) *
                            rknn_context.model_height * 3;
    std::unique_ptr<uint8_t[]> rgb(new uint8_t[rgb_size]);
    perception::BucketDetector bucket_detector;

    LOGI("Starry perception-only pipeline: camera=%d frame=%dx%d model=%dx%d",
         uvc_index,
         kFrameWidth,
         kFrameHeight,
         rknn_context.model_width,
         rknn_context.model_height);
    for (int valid = 0; valid < 10 && !g_stop.load();) {
        if (capture.getFrame(mjpeg.get(), kMjpegCapacity, 500) > 0) ++valid;
    }
    std::printf("STARRY_PERCEPTION_READY protocol=2 transport=%s robot_ci=%u\n",
                transport.c_str(), robot_ci_once ? 1U : 0U);
    std::fflush(stdout);

    std::thread diag_thread;
    if (pipeline_heartbeat != 0) {
        diag_thread = std::thread([] {
            uint64_t last_progress = g_diag_progress.load(std::memory_order_relaxed);
            unsigned stalled_seconds = 0;
            while (!g_diag_stop.load(std::memory_order_relaxed)) {
                sleep(1);
                const uint64_t progress = g_diag_progress.load(std::memory_order_relaxed);
                stalled_seconds = progress == last_progress ? stalled_seconds + 1 : 0;
                last_progress = progress;
                std::printf("STARRY_PIPELINE_HEARTBEAT stage=%s attempt=%llu "
                            "inference=%llu progress=%llu stalled_s=%u\n",
                            diag_stage_name(g_diag_stage.load(std::memory_order_relaxed)),
                            static_cast<unsigned long long>(
                                g_diag_attempt.load(std::memory_order_relaxed)),
                            static_cast<unsigned long long>(
                                g_diag_inference.load(std::memory_order_relaxed)),
                            static_cast<unsigned long long>(progress), stalled_seconds);
                std::fflush(stdout);
            }
        });
    }

    const uint64_t started_ms = monotonic_ms();
    uint64_t perf_window_started_ms = started_ms;
    uint64_t perf_window_started_inferences = 0;
    unsigned perf_windows = 0;
    bool robot_ci_perf_passed = !robot_ci_once;
    std::string robot_ci_phase;
    uint64_t inference_sequence = 0;
    int emitted = 0;
    int ivc_sent = 0;
    int ivc_dropped = 0;
    int consecutive_failures = 0;
    bool transport_failed = false;
    uint64_t rate_window_started_ms = started_ms;
    uint64_t rate_window_started_inferences = 0;
    int rate_window_started_ivc_sent = 0;
    int rate_window_started_ivc_dropped = 0;
    int rate_window_started_results = 0;
    perception::PerceptionResultV2 latest_result{};
    bool has_latest_result = false;
    const auto maybe_report_status = [&](uint64_t now_ms) {
        if (status_every == 0 || !has_latest_result) return;
        const int window_results = emitted - rate_window_started_results;
        if (window_results < status_every) return;
        const uint64_t window_ms = now_ms - rate_window_started_ms;

        const uint64_t window_inferences =
            inference_sequence - rate_window_started_inferences;
        const int window_ivc_sent = ivc_sent - rate_window_started_ivc_sent;
        const int window_ivc_dropped =
            ivc_dropped - rate_window_started_ivc_dropped;
        const double inference_fps =
            window_inferences * 1000.0 / static_cast<double>(window_ms);
        const double ivc_fps =
            window_ivc_sent * 1000.0 / static_cast<double>(window_ms);

        perception::print_status(latest_result,
                                 static_cast<uint64_t>(window_results),
                                 window_ms / 1000.0,
                                 inference_fps,
                                 ivc_fps,
                                 window_ivc_sent,
                                 window_ivc_dropped);

        rate_window_started_ms = now_ms;
        rate_window_started_inferences = inference_sequence;
        rate_window_started_ivc_sent = ivc_sent;
        rate_window_started_ivc_dropped = ivc_dropped;
        rate_window_started_results = emitted;
    };
    while (!g_stop.load() && (max_results == 0 || emitted < max_results)) {
        g_diag_attempt.fetch_add(1, std::memory_order_relaxed);
        g_diag_stage.store(DiagStage::Capture, std::memory_order_relaxed);
        const int jpeg_length = capture.getFrame(mjpeg.get(), kMjpegCapacity, 1000);
        g_diag_progress.fetch_add(1, std::memory_order_relaxed);
        if (jpeg_length <= 0) {
            if (++consecutive_failures >= 10) {
                LOGE("Camera produced no frame for 10 consecutive attempts");
                break;
            }
            continue;
        }

        int pad_x = 0;
        int pad_y = 0;
        float scale = 1.0f;
        g_diag_stage.store(DiagStage::Decode, std::memory_order_relaxed);
        if (decode_mjpeg(mjpeg.get(),
                         static_cast<size_t>(jpeg_length),
                         rgb.get(),
                         rknn_context.model_width,
                         rknn_context.model_height,
                         pad_x,
                         pad_y,
                         scale) != 0) {
            if (++consecutive_failures >= 10) {
                LOGE("MJPEG decode failed for 10 consecutive frames");
                break;
            }
            continue;
        }
        g_diag_progress.fetch_add(1, std::memory_order_relaxed);

        std::vector<detection> detections;
        g_diag_stage.store(DiagStage::Rknn, std::memory_order_relaxed);
        if (detect_run(&rknn_context,
                       rgb.get(),
                       rknn_context.model_width,
                       rknn_context.model_height,
                       kFrameWidth,
                       kFrameHeight,
                       pad_x,
                       pad_y,
                       scale,
                       0.5f,
                       0.45f,
                       detections) < 0) {
            if (++consecutive_failures >= 10) {
                LOGE("RKNN inference failed for 10 consecutive frames");
                break;
            }
            continue;
        }
        g_diag_progress.fetch_add(1, std::memory_order_relaxed);
        consecutive_failures = 0;
        ++inference_sequence;
        g_diag_inference.store(inference_sequence, std::memory_order_relaxed);
        g_diag_stage.store(DiagStage::Result, std::memory_order_relaxed);
        const uint64_t elapsed_ms = monotonic_ms() - started_ms;
        if (robot_ci_once && perf_windows < 2 &&
            monotonic_ms() - perf_window_started_ms >= 10000) {
            const uint64_t window_elapsed_ms = monotonic_ms() - perf_window_started_ms;
            const uint64_t window_inferences =
                inference_sequence - perf_window_started_inferences;
            const double fps = window_elapsed_ms > 0
                ? window_inferences * 1000.0 / window_elapsed_ms : 0.0;
            ++perf_windows;
            std::printf("STARRY_ROBOT_CI_PERF_WINDOW index=%u/2 elapsed_ms=%llu "
                        "inferences=%llu effective_fps=%.2f threshold=%.2f\n",
                        perf_windows,
                        static_cast<unsigned long long>(window_elapsed_ms),
                        static_cast<unsigned long long>(window_inferences),
                        fps, robot_ci_min_fps);
            std::fflush(stdout);
            if (fps < robot_ci_min_fps) {
                LOGE("Robot CI perception performance failed: %.2f < %.2f FPS",
                     fps, robot_ci_min_fps);
                transport_failed = true;
                break;
            }
            perf_window_started_ms = monotonic_ms();
            perf_window_started_inferences = inference_sequence;
            robot_ci_perf_passed = perf_windows == 2;
        }
        if (inference_sequence % static_cast<uint64_t>(report_every) != 0) {
            continue;
        }

        const perception::BucketDetection bucket = bucket_detector.detect(
            rgb.get(), rknn_context.model_width, rknn_context.model_height);
        perception::PerceptionResultV2 result =
            make_result(inference_sequence, detections, bucket);
        latest_result = result;
        has_latest_result = true;
        if (robot_ci_once && elapsed_ms >= kRobotCiPerceptionMs) {
            const char* phase = apply_robot_ci_scene(
                result, elapsed_ms - kRobotCiPerceptionMs);
            if (robot_ci_phase != phase) {
                robot_ci_phase = phase;
                std::printf("STARRY_ROBOT_CI_PHASE name=%s elapsed_ms=%llu\n",
                            phase, static_cast<unsigned long long>(elapsed_ms));
                std::fflush(stdout);
            }
        }
        const uint64_t emitted_index = static_cast<uint64_t>(emitted) + 1U;
        const bool debug_trace =
            emitted_index % kDebugTraceEveryResults == 0U;
        if (transport == "ivc") {
            g_diag_stage.store(DiagStage::Ivc, std::memory_order_relaxed);
            bool dropped = false;
            const uint64_t ivc_started_ms = monotonic_ms();
            if (debug_trace) {
                LOGD("STARRY_DIAG seq=%llu stage=ivc-send-enter sent=%d dropped=%d",
                     static_cast<unsigned long long>(result.sequence), ivc_sent,
                     ivc_dropped);
                std::fflush(stdout);
            }
            if (!ivc.send(result, dropped)) {
                transport_failed = true;
                break;
            }
            const uint64_t ivc_elapsed_ms = monotonic_ms() - ivc_started_ms;
            if (dropped) {
                ++ivc_dropped;
                LOGW("AxVisor IVC ring stayed full; dropped seq=%llu total=%d",
                     static_cast<unsigned long long>(result.sequence),
                     ivc_dropped);
            } else {
                ++ivc_sent;
            }
            if (debug_trace || ivc_elapsed_ms >= kSlowIvcWriteMs) {
                LOGD("STARRY_DIAG seq=%llu stage=ivc-send-exit elapsed_ms=%llu "
                     "result=%s sent=%d dropped=%d slow=%u",
                     static_cast<unsigned long long>(result.sequence),
                     static_cast<unsigned long long>(ivc_elapsed_ms),
                     dropped ? "dropped" : "sent", ivc_sent, ivc_dropped,
                     ivc_elapsed_ms >= kSlowIvcWriteMs ? 1U : 0U);
                std::fflush(stdout);
            }
        }
        g_diag_progress.fetch_add(1, std::memory_order_relaxed);
        g_diag_stage.store(DiagStage::Idle, std::memory_order_relaxed);
        ++emitted;
        maybe_report_status(monotonic_ms());
        if (robot_ci_once && elapsed_ms >= kRobotCiTotalMs) {
            std::printf("STARRY_ROBOT_CI_DONE perf=%s duration_ms=%llu\n",
                        robot_ci_perf_passed ? "pass" : "fail",
                        static_cast<unsigned long long>(elapsed_ms));
            std::fflush(stdout);
            break;
        }
    }

    g_diag_stop.store(true, std::memory_order_relaxed);
    if (diag_thread.joinable()) diag_thread.join();
    capture.close();
    detect_deinit(&rknn_context);
    std::printf("STARRY_PERCEPTION_OK results=%d inferences=%llu ivc_sent=%d ivc_dropped=%d\n",
                emitted,
                static_cast<unsigned long long>(inference_sequence),
                ivc_sent,
                ivc_dropped);
    return consecutive_failures >= 10 || transport_failed ||
                   (robot_ci_once && !robot_ci_perf_passed)
               ? 1
               : 0;
}
