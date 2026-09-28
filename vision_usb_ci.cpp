// vision_usb_ci.cpp
//
// Two 10s windows of real UVC capture -> JPEG decode -> RKNN YOLO inference run
// concurrently with a bounded FT232R (0403:6001) binary loopback workload.
//
// This app never opens /dev/ttyS6 or any arm/wheel device and never invokes the
// robot single-shot flow. The FT232R is reached through one of two transports,
// chosen explicitly or automatically:
//   - "usb": raw libusb vendor control + bulk (StarryOS and root Linux), which
//            exercises USB host / usbfs without needing Starry FTDI TTY support;
//   - "tty": Linux /dev/ttyUSBx termios when the raw node is denied, which does
//            exercise the Linux ftdi_sio TTY path.
// The DEVICE log always names the transport actually used, so the two coverage
// areas are never confused.

#include "vision_usb_ci.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <sys/time.h>
#include <time.h>

#include <turbojpeg.h>

#include "logger.hpp"
#include "capture/uvc_capture.hpp"
#include "detect/detect.hpp"
#include "usb/ftdi_protocol.hpp"
#include "usb/ft232_loopback.hpp"
#include "usb/ft232_tty.hpp"

namespace {

const char* const TAG = "[VISION_USB_CI]";

const int    FRAME_WIDTH  = 640;
const int    FRAME_HEIGHT = 480;
const size_t MJPEG_CAP    = 1024 * 1024;

const uint64_t WINDOW_US          = 10ULL * 1000000ULL;
const int      WARMUP_FRAMES      = 15;
const uint64_t WARMUP_DEADLINE_US = 6ULL * 1000000ULL;
const uint64_t FRAME_STALL_US     = 3ULL * 1000000ULL;
// A single corrupted MJPEG frame from the UVC stream is tolerated: a window may
// skip up to this many decode failures, and the next one fails the window with
// reason=jpeg_decode_unstable. Sustained corruption therefore still fails while
// one-off hardware noise does not flake an otherwise healthy run.
const uint64_t MAX_JPEG_ERRORS_PER_WINDOW = 3;
const uint64_t MAX_JPEG_ERRORS_TOTAL      = 2 * MAX_JPEG_ERRORS_PER_WINDOW;

const int      FT232_BAUD          = 1000000;
// The worker's frame id is the fixed 0x01 used by all loopback frames. The
// instruction is Feetech PING (see ftdi_proto::LOOPBACK_INSTRUCTION), never
// WRITE, so a miswired real bus cannot change any servo register or motion.
const uint8_t  LOOPBACK_FRAME_ID   = 0x01;
static_assert(LOOPBACK_FRAME_ID != ftdi_proto::FEETECH_BROADCAST_ID,
              "loopback frames must not use the Feetech broadcast id");
const int      DEFAULT_CADENCE_HZ  = 20;
const double   DEFAULT_MIN_FPS     = 28.0;
// Fixed CI floor shared with run_vision_usb_ci_once.sh: at 20 Hz a full 10s
// window must produce at least 160 successful exchanges. An env override may
// only raise it, never weaken it.
const uint64_t FIXED_MIN_TX_PER_WINDOW = ftdi_proto::MIN_TX_PER_WINDOW_FLOOR;
const uint64_t FIXED_MIN_TX_TOTAL = 2 * FIXED_MIN_TX_PER_WINDOW;

volatile sig_atomic_t g_stop = 0;

void on_signal(int /*sig*/) { g_stop = 1; }

uint64_t monotonic_us()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

int env_int(const char* name, int fallback)
{
    const char* value = getenv(name);
    if (!value || !value[0]) return fallback;
    char* end = nullptr;
    const long parsed = strtol(value, &end, 10);
    if (!end || *end != '\0' || parsed <= 0 || parsed > 100000) return fallback;
    return static_cast<int>(parsed);
}

// Local JPEG -> RGB letterbox copy (same transform as tennis.cpp).
int decode_mjpeg(const uint8_t* jpeg_data, size_t jpeg_len, uint8_t* rgb_out,
                 int out_w, int out_h, int* pad_x, int* pad_y, float* scale_out)
{
    tjhandle tj = tjInitDecompress();
    if (!tj) return -1;

    int w = 0, h = 0, subsamp = 0, colorspace = 0;
    if (tjDecompressHeader3(tj, jpeg_data, jpeg_len, &w, &h, &subsamp, &colorspace) < 0) {
        tjDestroy(tj);
        return -1;
    }
    const float scale = std::min(static_cast<float>(out_w) / w,
                                 static_cast<float>(out_h) / h);
    const int new_w = static_cast<int>(w * scale + 0.5f);
    const int new_h = static_cast<int>(h * scale + 0.5f);
    const int off_x = (out_w - new_w) / 2;
    const int off_y = (out_h - new_h) / 2;
    if (pad_x) *pad_x = off_x;
    if (pad_y) *pad_y = off_y;
    if (scale_out) *scale_out = scale;

    memset(rgb_out, 114, static_cast<size_t>(out_w) * out_h * 3);
    uint8_t* tmp = static_cast<uint8_t*>(malloc(static_cast<size_t>(new_w) * new_h * 3));
    if (!tmp) { tjDestroy(tj); return -1; }

    const int ret = tjDecompress2(tj, jpeg_data, jpeg_len, tmp, new_w, 0, new_h,
                                  TJPF_RGB, TJFLAG_FASTDCT);
    tjDestroy(tj);
    if (ret < 0) { free(tmp); return -1; }

    for (int y = 0; y < new_h; y++) {
        memcpy(rgb_out + static_cast<size_t>(y + off_y) * out_w * 3 + off_x * 3,
               tmp + static_cast<size_t>(y) * new_w * 3,
               static_cast<size_t>(new_w) * 3);
    }
    free(tmp);
    return 0;
}

void print_fail(const char* reason, const std::string& detail)
{
    if (!detail.empty())
        printf("%s ATTEMPT_FAIL reason=%s detail=%s\n", TAG, reason, detail.c_str());
    else
        printf("%s ATTEMPT_FAIL reason=%s\n", TAG, reason);
    fflush(stdout);
}

struct WindowPerf {
    double   elapsed_s   = 0.0;
    uint64_t processed   = 0;
    // MJPEG frames that arrived on the UVC stream but failed JPEG decode.
    uint64_t jpeg_errors = 0;
};

// Run one >=10s window of capture + decode + inference. Returns the window
// performance and sets ok/reason; on failure the caller must abort the run.
WindowPerf run_window(UvcCapture& capture, rknn_app_context_t& ctx,
                      uint8_t* mjpeg, uint8_t* rgb, int model_w, int model_h,
                      const ft232::LoopbackWorker* worker,
                      bool& ok, std::string& reason)
{
    WindowPerf perf;
    const uint64_t start = monotonic_us();
    uint64_t last_frame_us = start;
    std::vector<detection> dets;

    while (monotonic_us() - start < WINDOW_US) {
        if (g_stop) { ok = false; reason = "interrupted"; break; }
        // A loopback failure must abort the window promptly instead of running
        // to 10s and burying the first error.
        if (worker && worker->fatal()) {
            ok = false;
            reason = "loopback_error";
            break;
        }

        const int jpeg_len = capture.getFrame(mjpeg, MJPEG_CAP, 200);
        if (jpeg_len <= 0) {
            if (monotonic_us() - last_frame_us > FRAME_STALL_US) {
                ok = false;
                reason = "frame_acquisition_stalled";
                break;
            }
            continue;
        }
        // A frame arrived (so the USB stream is alive), even if it later fails to
        // decode. Update the stall clock here so tolerated decode errors keep the
        // window going, and a stream of bad frames is caught by the error budget
        // below instead of being misread as acquisition stall.
        last_frame_us = monotonic_us();

        int pad_x = 0, pad_y = 0;
        float scale = 1.0f;
        if (decode_mjpeg(mjpeg, static_cast<size_t>(jpeg_len), rgb, model_w, model_h,
                         &pad_x, &pad_y, &scale) != 0) {
            // Count the bad frame and skip this iteration: processed is not
            // incremented because no valid frame was handled. The window only
            // fails once the tolerance is exceeded, so a single hardware-noise
            // frame cannot flake an otherwise healthy run while sustained
            // corruption still fails.
            perf.jpeg_errors++;
            if (perf.jpeg_errors > MAX_JPEG_ERRORS_PER_WINDOW) {
                ok = false;
                reason = "jpeg_decode_unstable";
                break;
            }
            continue;
        }

        dets.clear();
        const int det_count = detect_run(&ctx, rgb, model_w, model_h,
                                         FRAME_WIDTH, FRAME_HEIGHT, pad_x, pad_y, scale,
                                         0.5f, 0.45f, dets);
        if (det_count < 0) {
            ok = false;
            reason = "inference_failed";
            break;
        }
        perf.processed++;
    }

    perf.elapsed_s = static_cast<double>(monotonic_us() - start) / 1000000.0;
    return perf;
}

} // namespace

int cmd_vision_usb_ci(int argc, char** argv)
{
    // ── Parse arguments ───────────────────────────────────────────────────────
    const char* model_path = (argc >= 3) ? argv[2] : "models/tennis.rknn";
    double min_fps = DEFAULT_MIN_FPS;
    if (argc >= 4) {
        char* end = nullptr;
        const double parsed = strtod(argv[3], &end);
        if (!end || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0 ||
            parsed > 1000.0) {
            printf("%s Usage: %s vision-usb-ci [model.rknn] [min_fps] [uvc_index] "
                   "[--ftdi-serial S] [--ftdi-transport usb|tty|auto]\n", TAG, argv[0]);
            return 2;
        }
        min_fps = parsed;
    }
    int uvc_index = (argc >= 5) ? atoi(argv[4]) : 0;
    if (uvc_index < 0) uvc_index = 0;

    std::string serial_selector;
    const char* serial_env = getenv("FT232_SERIAL");
    if (serial_env && serial_env[0]) serial_selector = serial_env;
    std::string transport_mode = "auto";
    const char* transport_env = getenv("FTDI_TRANSPORT");
    if (transport_env && transport_env[0]) transport_mode = transport_env;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--ftdi-serial") == 0) {
            if (i + 1 >= argc) {
                printf("%s Usage: --ftdi-serial requires a value\n", TAG);
                return 2;
            }
            serial_selector = argv[i + 1];
            i++;
        } else if (strcmp(argv[i], "--ftdi-transport") == 0) {
            if (i + 1 >= argc) {
                printf("%s Usage: --ftdi-transport requires usb|tty|auto\n", TAG);
                return 2;
            }
            transport_mode = argv[i + 1];
            i++;
        }
    }
    if (transport_mode != "auto" && transport_mode != "usb" &&
        transport_mode != "tty") {
        printf("%s Usage: --ftdi-transport/-FTDI_TRANSPORT must be usb|tty|auto\n",
               TAG);
        return 2;
    }

    const int cadence_hz = env_int("VISION_USB_CI_CADENCE_HZ", DEFAULT_CADENCE_HZ);
    // An env override may raise the per-window requirement but never weaken it
    // below the fixed CI floor that the launcher also enforces.
    const uint64_t min_tx_per_window = std::max<uint64_t>(
        FIXED_MIN_TX_PER_WINDOW,
        static_cast<uint64_t>(env_int("VISION_USB_CI_MIN_TX",
                                      static_cast<int>(FIXED_MIN_TX_PER_WINDOW))));

    FILE* probe = fopen(model_path, "rb");
    if (!probe) {
        print_fail("missing_model", model_path);
        return 1;
    }
    fclose(probe);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    // ── FT232R transport: raw libusb first, then the Linux TTY fallback ───────
    // The raw libusb path is what StarryOS and root Linux runs use. When it is
    // denied (root-only /dev/bus/usb) the same 0403:6001 device is reopened via
    // its own /dev/ttyUSBx node. The TTY backend re-reads sysfs serial, so an
    // unreadable libusb serial is not treated as an ambiguous bus. Explicit
    // FTDI_TRANSPORT=usb never falls back. Nothing else is ever opened.
    std::string error;
    std::unique_ptr<ft232::LoopbackTransport> transport;
    if (transport_mode == "auto" || transport_mode == "usb") {
        std::unique_ptr<ft232::Ft232Loopback> usb(new ft232::Ft232Loopback());
        if (usb->open(serial_selector, FT232_BAUD, error)) {
            transport = std::move(usb);
        } else if (transport_mode == "usb") {
            print_fail("ft232_open", error);
            return 1;
        } else {
            const ft232::OpenFailure failure = usb->open_failure();
            if (!ft232::may_fall_back_to_tty(failure)) {
                print_fail("ft232_open", error);
                return 1;
            }
            printf("%s FT232_USB_FALLBACK failure=%s detail=%s\n", TAG,
                   ft232::open_failure_string(failure), error.c_str());
            fflush(stdout);
        }
    }
    if (!transport) {
        std::unique_ptr<ft232::Ft232Tty> tty(new ft232::Ft232Tty());
        if (tty->open(serial_selector, FT232_BAUD, error)) {
            transport = std::move(tty);
        } else {
            print_fail("ft232_open", error);
            return 1;
        }
    }
    const ft232::DeviceIdentity& id = transport->identity();
    // Capture identity now; close() clears the device identity.
    const std::string ft232_serial = id.serial;
    const int ft232_bus = id.bus;
    const int ft232_addr = id.address;
    // The camera is selected only by its zero-based UVC index. UvcCapture does
    // not read or report VID/PID, so no camera hardware identity is claimed.
    printf("%s DEVICE transport=%s ft232=0403:6001 serial=%s bus=%d addr=%d "
           "iface=%d ep_in=0x%02x ep_out=0x%02x tty=%s camera_uvc_index=%d\n",
           TAG, transport->transport_name(),
           id.serial.empty() ? "-" : id.serial.c_str(), id.bus, id.address,
           id.interface_number, id.endpoint_in, id.endpoint_out,
           id.tty_path.empty() ? "-" : id.tty_path.c_str(), uvc_index);
    fflush(stdout);

    // ── RKNN model before the camera (matches the stable robot startup order,
    //    avoiding Starry USB/NPU startup contention) ───────────────────────────
    rknn_app_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    if (detect_init(model_path, &ctx) != 0) {
        // A failed init is not guaranteed safe to release; mirror the stable
        // robot workload and only close what was actually opened.
        transport->close();
        print_fail("model_init_failed", model_path);
        return 1;
    }
    const int model_w = ctx.model_width;
    const int model_h = ctx.model_height;

    UvcCapture capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        detect_deinit(&ctx);
        transport->close();
        print_fail("uvc_open_failed", "");
        return 1;
    }

    uint8_t* mjpeg = static_cast<uint8_t*>(malloc(MJPEG_CAP));
    uint8_t* rgb = static_cast<uint8_t*>(
        malloc(static_cast<size_t>(model_w) * model_h * 3));
    if (!mjpeg || !rgb) {
        free(mjpeg);
        free(rgb);
        capture.close();
        detect_deinit(&ctx);
        transport->close();
        print_fail("oom", "");
        return 1;
    }

    std::string reason;
    bool ok = true;

    // ── Bounded camera warmup ─────────────────────────────────────────────────
    {
        const uint64_t deadline = monotonic_us() + WARMUP_DEADLINE_US;
        int valid = 0;
        while (valid < WARMUP_FRAMES) {
            if (g_stop) { ok = false; reason = "interrupted"; break; }
            if (monotonic_us() > deadline) {
                ok = false;
                reason = "camera_warmup_timeout";
                break;
            }
            if (capture.getFrame(mjpeg, MJPEG_CAP, 500) > 0) valid++;
        }
    }

    // ── Two visual windows with the concurrent loopback worker ────────────────
    ft232::LoopbackWorker worker(*transport, cadence_hz, LOOPBACK_FRAME_ID);
    WindowPerf perf[2];
    bool perf_valid[2] = {false, false};
    ftdi_proto::WindowStats loopback[2];
    bool worker_started = false;

    if (ok) {
        printf("%s PERF_BEGIN windows=2 duration_s=10 min_fps=%.2f cadence_hz=%d "
               "min_tx=%" PRIu64 "\n", TAG, min_fps, cadence_hz, min_tx_per_window);
        fflush(stdout);
        worker.start();
        worker_started = true;

        for (int w = 0; w < 2; w++) {
            worker.set_window(w);
            perf[w] = run_window(capture, ctx, mjpeg, rgb, model_w, model_h, &worker,
                                 ok, reason);
            perf_valid[w] = true;
            if (!ok) break;
        }
    }

    // Stop the worker first, then read the authoritative per-window stats so no
    // exchange still in flight is lost or attributed twice.
    if (worker_started) {
        worker.stop();
        loopback[0] = worker.snapshot(0);
        loopback[1] = worker.snapshot(1);
    }

    for (int w = 0; w < 2; w++) {
        if (!perf_valid[w]) continue;
        printf("%s PERF_WINDOW index=%d/2 elapsed_s=%.2f processed=%" PRIu64
               " effective_fps=%.2f jpeg_errors=%" PRIu64 "\n", TAG, w + 1,
               perf[w].elapsed_s, perf[w].processed,
               perf[w].elapsed_s > 0.0
                   ? perf[w].processed / perf[w].elapsed_s : 0.0,
               perf[w].jpeg_errors);
        printf("%s LOOPBACK_WINDOW index=%d/2 attempts=%" PRIu64 " tx=%" PRIu64
               " errors=%" PRIu64 " avg_ms=%.3f max_ms=%.3f\n", TAG, w + 1,
               loopback[w].attempts, loopback[w].matches, loopback[w].errors,
               loopback[w].avg_latency_ms(), loopback[w].max_latency_us / 1000.0);
    }
    fflush(stdout);

    if (ok && worker.fatal()) {
        ok = false;
        reason = "loopback_error";
    }

    // ── UVC pause / resume + fresh frame ──────────────────────────────────────
    bool pause_resume_ok = false;
    if (ok) {
        if (capture.pause() != 0 || capture.resume() != 0) {
            ok = false;
            reason = "uvc_pause_resume_failed";
        } else {
            const uint64_t deadline = monotonic_us() + WARMUP_DEADLINE_US;
            while (monotonic_us() < deadline) {
                if (capture.getFrame(mjpeg, MJPEG_CAP, 500) > 0) {
                    pause_resume_ok = true;
                    break;
                }
            }
            if (!pause_resume_ok) {
                ok = false;
                reason = "uvc_resume_no_fresh_frame";
            }
        }
    }

    // ── Cleanup (all of it must succeed before any success marker) ────────────
    capture.close();
    const int release_ret = detect_deinit(&ctx);
    free(mjpeg);
    free(rgb);
    const bool device_cleanup_ok = transport->close();
    const std::string device_cleanup_error = transport->cleanup_error();

    if (!ok) {
        print_fail(reason.empty() ? "run_failed" : reason.c_str(), "");
        return 1;
    }
    if (!pause_resume_ok) {
        print_fail("uvc_pause_resume_failed", "");
        return 1;
    }
    if (release_ret != 0) {
        print_fail("model_cleanup_failed", "");
        return 1;
    }
    if (!device_cleanup_ok) {
        print_fail("ft232_cleanup_failed", device_cleanup_error);
        return 1;
    }
    if (worker.fatal()) {
        print_fail("loopback_error", worker.fatal_error());
        return 1;
    }

    // ── Verdict ───────────────────────────────────────────────────────────────
    ftdi_proto::VerdictInput verdict_input;
    verdict_input.min_fps = min_fps;
    verdict_input.min_tx_per_window = min_tx_per_window;
    uint64_t total_processed = 0;
    double total_elapsed = 0.0;
    uint64_t total_tx = 0;
    uint64_t total_errors = 0;
    uint64_t total_attempts = 0;
    uint64_t total_jpeg_errors = 0;
    for (int w = 0; w < 2; w++) {
        verdict_input.perf[w].elapsed_s = perf[w].elapsed_s;
        verdict_input.perf[w].processed = perf[w].processed;
        verdict_input.loopback[w] = loopback[w];
        total_processed += perf[w].processed;
        total_elapsed += perf[w].elapsed_s;
        total_tx += loopback[w].matches;
        total_errors += loopback[w].errors;
        total_attempts += loopback[w].attempts;
        total_jpeg_errors += perf[w].jpeg_errors;
    }
    const double effective_fps = total_elapsed > 0.0 ? total_processed / total_elapsed : 0.0;

    const ftdi_proto::VerdictResult verdict = ftdi_proto::evaluate_verdict(verdict_input);
    if (!verdict.pass) {
        print_fail(verdict.reason.c_str(), "");
        return 1;
    }
    if (total_tx < FIXED_MIN_TX_TOTAL) {
        print_fail("loopback_below_total_floor", "");
        return 1;
    }

    printf("%s PERF_SUMMARY windows=2 elapsed_s=%.2f processed=%" PRIu64
           " effective_fps=%.2f min_fps=%.2f jpeg_errors=%" PRIu64 "\n", TAG,
           total_elapsed, total_processed, effective_fps, min_fps,
           total_jpeg_errors);
    printf("%s LOOPBACK_SUMMARY attempts=%" PRIu64 " tx=%" PRIu64 " errors=%" PRIu64
           " min_tx=%" PRIu64 "\n", TAG, total_attempts, total_tx, total_errors,
           2 * min_tx_per_window);
    printf("%s UVC_PAUSE_RESUME=PASS fresh_frame=1\n", TAG);
    printf("%s APPLICATION_PASS windows=2 min_fps=%.2f processed=%" PRIu64
           " elapsed_s=%.2f effective_fps=%.2f loopback_tx=%" PRIu64
           " loopback_errors=%" PRIu64 " loopback_min_tx=%" PRIu64
           " jpeg_errors=%" PRIu64
           " ft232_serial=%s ft232_bus=%d ft232_addr=%d camera_uvc_index=%d"
           " pause_resume=1\n",
           TAG, min_fps, total_processed, total_elapsed, effective_fps, total_tx,
           total_errors, 2 * min_tx_per_window, total_jpeg_errors,
           ft232_serial.empty() ? "-" : ft232_serial.c_str(), ft232_bus, ft232_addr,
           uvc_index);
    fflush(stdout);
    return 0;
}
