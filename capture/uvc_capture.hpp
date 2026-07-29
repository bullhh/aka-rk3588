#ifndef UVC_CAPTURE_HPP
#define UVC_CAPTURE_HPP

#include <cstdint>
#include <cstddef>
#include <atomic>

#include <libuvc/libuvc.h>

// UVC camera capture wrapper.
// Uses only libuvc functions already used in uvc-fps-c/uvc-fps.c.
// After open(), call getFrame() in a loop; it fills a caller-supplied buffer
// with raw MJPEG bytes.  Call close() when done.
class UvcCapture {
public:
    struct Stats {
        uint64_t captured_frames = 0;
    };

    UvcCapture();
    ~UvcCapture();

    // Open device at zero-based index with requested format.
    // Returns 0 on success, -1 on failure.
    int open(int device_index = 0,
             int width = 640,
             int height = 480,
             int fps = 30);

    void close();
    int pause();
    int resume();
    bool streaming() const { return streaming_; }

    // Block until the next frame arrives (or timeout_ms elapses).
    // Copies MJPEG data into buf (capacity cap).
    // Returns number of bytes written, or -1 on error/timeout.
    int getFrame(uint8_t* buf, size_t cap, int timeout_ms = 200,
                 long* wait_us = nullptr, long* copy_us = nullptr);

    Stats stats() const;

    int width()  const { return width_;  }
    int height() const { return height_; }

private:
    uvc_context_t*       ctx_  = nullptr;
    uvc_device_handle_t* devh_ = nullptr;
    uvc_stream_handle_t* strmh_ = nullptr;
    uvc_stream_ctrl_t    ctrl_ = {};

    int width_  = 640;
    int height_ = 480;

    bool     streaming_ = false;

    std::atomic<uint64_t> captured_frames_{0};
};

#endif // UVC_CAPTURE_HPP
