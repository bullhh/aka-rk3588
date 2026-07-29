// UVC capture wrapper for rk3588.
// Logic follows uvc-fps-c/uvc-fps.c - only libuvc functions used there are used here.
#include "uvc_capture.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <time.h>

static uint64_t monotonic_us()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

// ── helpers from uvc-fps.c ────────────────────────────────────────────────────
// UVC_FRAME_FORMAT_MJPEG is defined in libuvc.h - do not redefine

UvcCapture::UvcCapture()
{}

UvcCapture::~UvcCapture()
{
    close();
}

// ── open ──────────────────────────────────────────────────────────────────────
int UvcCapture::open(int device_index, int width, int height, int fps)
{
    width_  = width;
    height_ = height;

    // uvc_init
    int res = uvc_init(&ctx_, nullptr);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_init failed: %s\n", uvc_strerror((uvc_error_t)res));
        return -1;
    }

    // enumerate devices
    uvc_device_t** dev_list = nullptr;
    res = uvc_get_device_list(ctx_, &dev_list);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_get_device_list failed: %s\n", uvc_strerror((uvc_error_t)res));
        uvc_exit(ctx_); ctx_ = nullptr;
        return -1;
    }

    uvc_device_t* dev = nullptr;
    for (int i = 0; dev_list[i] != nullptr; i++) {
        if (i == device_index) {
            dev = dev_list[i];
            uvc_ref_device(dev);
            break;
        }
    }
    uvc_free_device_list(dev_list, 1);

    if (!dev) {
        fprintf(stderr, "[UvcCapture] device index %d not found\n", device_index);
        uvc_exit(ctx_); ctx_ = nullptr;
        return -1;
    }

    res = uvc_open(dev, &devh_);
    uvc_unref_device(dev);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_open failed: %s\n", uvc_strerror((uvc_error_t)res));
        uvc_exit(ctx_); ctx_ = nullptr;
        return -1;
    }

    // Keep the requested frame rate stable. The camera otherwise enables
    // exposure-priority mode and may reduce a 30 FPS stream to about 10 FPS.
    res = uvc_set_ae_priority(devh_, 0);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] disable dynamic frame rate failed: %s\n",
                uvc_strerror((uvc_error_t)res));
    }

    // negotiate stream control (MJPEG)
    res = uvc_get_stream_ctrl_format_size(
        devh_, &ctrl_, UVC_FRAME_FORMAT_MJPEG, width, height, fps);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_get_stream_ctrl_format_size failed: %s\n", uvc_strerror((uvc_error_t)res));
        uvc_close(devh_); devh_ = nullptr;
        uvc_exit(ctx_);   ctx_  = nullptr;
        return -1;
    }
    // Use libuvc's synchronous stream API. On Starry, libuvc's additional
    // callback-dispatch thread can fall behind and overwrite completed frames.
    res = uvc_stream_open_ctrl(devh_, &strmh_, &ctrl_);
    if (res >= 0) res = uvc_stream_start(strmh_, nullptr, nullptr, 0);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] start stream failed: %s\n", uvc_strerror((uvc_error_t)res));
        if (strmh_) { uvc_stream_close(strmh_); strmh_ = nullptr; }
        uvc_close(devh_); devh_ = nullptr;
        uvc_exit(ctx_);   ctx_  = nullptr;
        return -1;
    }
    streaming_ = true;

    printf("[UvcCapture] streaming %dx%d @ %d fps\n", width, height, fps);
    return 0;
}

void UvcCapture::close()
{
    if (strmh_) {
        if (streaming_) uvc_stream_stop(strmh_);
        uvc_stream_close(strmh_);
        strmh_ = nullptr;
        streaming_ = false;
    }
    if (devh_) {
        uvc_close(devh_);
        devh_ = nullptr;
    }
    if (ctx_) {
        uvc_exit(ctx_);
        ctx_ = nullptr;
    }
}

int UvcCapture::pause()
{
    if (!devh_ || !streaming_) return 0;
    uvc_stream_stop(strmh_);
    uvc_stream_close(strmh_);
    strmh_ = nullptr;
    streaming_ = false;
    return 0;
}

int UvcCapture::resume()
{
    if (!devh_) return -1;
    if (streaming_) return 0;
    int res = uvc_stream_open_ctrl(devh_, &strmh_, &ctrl_);
    if (res >= 0) res = uvc_stream_start(strmh_, nullptr, nullptr, 0);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] resume failed: %s\n",
                uvc_strerror((uvc_error_t)res));
        if (strmh_) {
            uvc_stream_close(strmh_);
            strmh_ = nullptr;
        }
        return -1;
    }
    streaming_ = true;
    return 0;
}

// ── getFrame ──────────────────────────────────────────────────────────────────
int UvcCapture::getFrame(uint8_t* buf, size_t cap, int timeout_ms,
                         long* wait_us, long* copy_us)
{
    const uint64_t wait_start_us = monotonic_us();
    uvc_frame_t* frame = nullptr;
    const int res = uvc_stream_get_frame(strmh_, &frame, timeout_ms * 1000);
    const uint64_t copy_start_us = monotonic_us();
    if (wait_us) *wait_us = static_cast<long>(copy_start_us - wait_start_us);
    if (res < 0 || !frame || !frame->data || frame->data_bytes == 0) {
        if (copy_us) *copy_us = 0;
        return -1;
    }
    const size_t copy_len = frame->data_bytes < cap ? frame->data_bytes : cap;
    memcpy(buf, frame->data, copy_len);
    if (copy_us) *copy_us = static_cast<long>(monotonic_us() - copy_start_us);
    captured_frames_.fetch_add(1, std::memory_order_relaxed);
    return static_cast<int>(copy_len);
}

UvcCapture::Stats UvcCapture::stats() const
{
    Stats result;
    result.captured_frames = captured_frames_.load(std::memory_order_relaxed);
    return result;
}
