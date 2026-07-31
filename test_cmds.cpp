// test_cmds.cpp — debug sub-commands (test-uvc / test-yolo / test-motor / test-arm)
#include "test_cmds.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include <vector>

#include <math.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <thread>
#include <atomic>

#include <turbojpeg.h>

#include "logger.hpp"
#include "capture/uvc_capture.hpp"
#include "detect/detect.hpp"
#include "motor/motor.hpp"
#include "arm/arm.hpp"
#include "feetech/feetech_bus.hpp"
#include "robot/omni_base.hpp"
#include "robot/feetech_arm.hpp"
#include "robot/lekiwi_calibration.hpp"
#include "robot/lekiwi_arm_poses.hpp"
#include "robot/lekiwi_task_controller.hpp"

static const int FRAME_WIDTH  = 640;
static const int FRAME_HEIGHT = 480;

// ── helpers (local copies) ────────────────────────────────────────────────────
static long _elapsed_us(const struct timeval& s) {
    struct timeval now; gettimeofday(&now, nullptr);
    return (now.tv_sec - s.tv_sec) * 1000000L + (now.tv_usec - s.tv_usec);
}

static int _decode_mjpeg(const uint8_t* jpeg, size_t len,
                         uint8_t* out, int ow, int oh,
                         int* pad_x, int* pad_y, float* scale_out)
{
    tjhandle tj = tjInitDecompress();
    if (!tj) return -1;
    int w, h, subsamp, cs;
    if (tjDecompressHeader3(tj, jpeg, len, &w, &h, &subsamp, &cs) < 0)
        { tjDestroy(tj); return -1; }

    float sc = std::min((float)ow / w, (float)oh / h);
    int nw = (int)(w * sc + 0.5f), nh = (int)(h * sc + 0.5f);
    int ox = (ow - nw) / 2, oy = (oh - nh) / 2;
    if (pad_x)    *pad_x    = ox;
    if (pad_y)    *pad_y    = oy;
    if (scale_out)*scale_out = sc;

    memset(out, 114, ow * oh * 3);
    uint8_t* tmp = (uint8_t*)malloc(nw * nh * 3);
    if (!tmp) { tjDestroy(tj); return -1; }
    int ret = tjDecompress2(tj, jpeg, len, tmp, nw, 0, nh, TJPF_RGB, TJFLAG_FASTDCT);
    tjDestroy(tj);
    if (ret < 0) { free(tmp); return -1; }
    for (int y = 0; y < nh; y++)
        memcpy(out + ((y + oy) * ow + ox) * 3, tmp + y * nw * 3, nw * 3);
    free(tmp);
    return 0;
}

static int _save_jpeg(const char* path, const uint8_t* data, size_t len) {
    FILE* f = fopen(path, "wb");
    if (!f) { LOGE("Cannot open %s", path); return -1; }
    fwrite(data, 1, len, f);
    fclose(f);
    LOGI("Saved %zu bytes → %s", len, path);
    return 0;
}

static void _draw_box(uint8_t* rgb, int iw, int ih,
                      int x, int y, int w, int h,
                      uint8_t r, uint8_t g, uint8_t b)
{
    auto cl = [](int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); };
    int x0=cl(x,0,iw-1), y0=cl(y,0,ih-1), x1=cl(x+w-1,0,iw-1), y1=cl(y+h-1,0,ih-1);
    for (int px=x0;px<=x1;px++) {
        uint8_t* t=rgb+(y0*iw+px)*3; uint8_t* bt=rgb+(y1*iw+px)*3;
        t[0]=bt[0]=r; t[1]=bt[1]=g; t[2]=bt[2]=b;
    }
    for (int py=y0;py<=y1;py++) {
        uint8_t* l=rgb+(py*iw+x0)*3; uint8_t* ri=rgb+(py*iw+x1)*3;
        l[0]=ri[0]=r; l[1]=ri[1]=g; l[2]=ri[2]=b;
    }
}

// ── 5×7 点阵字体（ASCII 32-126）────────────────────────────────────────────────
// 每个字符 5 列，每列 7 位（bit0=top），存储为 5 个 uint8_t
static const uint8_t _font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x08,0x2A,0x1C,0x2A,0x08}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
};

// 在 RGB 图上绘制字符（scale: 放大倍数）
static void _draw_char(uint8_t* rgb, int iw, int ih, int x0, int y0, char c,
                       uint8_t r, uint8_t g, uint8_t b, int scale = 2)
{
    if (c < 32 || c > 'z') c = '?';
    const uint8_t* glyph = _font5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (!(glyph[col] & (1 << row))) continue;
            for (int sy = 0; sy < scale; sy++) {
                for (int sx = 0; sx < scale; sx++) {
                    int px = x0 + col * scale + sx;
                    int py = y0 + row * scale + sy;
                    if (px < 0 || px >= iw || py < 0 || py >= ih) continue;
                    uint8_t* p = rgb + (py * iw + px) * 3;
                    p[0] = r; p[1] = g; p[2] = b;
                }
            }
        }
    }
}

static void _draw_text(uint8_t* rgb, int iw, int ih, int x, int y, const char* text,
                       uint8_t r, uint8_t g, uint8_t b, int scale = 2)
{
    int cx = x;
    for (int i = 0; text[i]; i++) {
        _draw_char(rgb, iw, ih, cx, y, text[i], r, g, b, scale);
        cx += (5 + 1) * scale;
    }
}

// 清空目录内所有文件（不递归）
static void _clear_dir(const char* dir) {
    DIR* d = opendir(dir);
    if (!d) return;
    struct dirent* ent;
    char path[512];
    while ((ent = readdir(d)) != nullptr) {
        if (ent->d_name[0] == '.') continue;
        snprintf(path, sizeof(path), "%s/%s", dir, ent->d_name);
        remove(path);
    }
    closedir(d);
}

static int _save_rgb_as_jpeg(const char* path, const uint8_t* rgb, int w, int h) {
    tjhandle tj = tjInitCompress();
    if (!tj) return -1;
    unsigned char* buf = nullptr; unsigned long bl = 0;
    int ret = tjCompress2(tj, rgb, w, 0, h, TJPF_RGB,
                          &buf, &bl, TJSAMP_420, 90, TJFLAG_FASTDCT);
    tjDestroy(tj);
    if (ret < 0) return -1;
    FILE* f = fopen(path, "wb"); if (!f) { tjFree(buf); return -1; }
    fwrite(buf, 1, bl, f); fclose(f); tjFree(buf);
    LOGI("Saved %lu bytes → %s", bl, path);
    return 0;
}

// ── test-uvc ──────────────────────────────────────────────────────────────────
int cmd_test_uvc(int uvc_index)
{
    UvcCapture capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index); return 1;
    }

    const char* bench_env = getenv("AKA_UVC_BENCH_SECONDS");
    const int bench_seconds = bench_env ? atoi(bench_env) : 0;
    if (bench_seconds > 0) {
        const size_t BUF = 1024 * 1024;
        uint8_t* buf = (uint8_t*)malloc(BUF);
        if (!buf) { capture.close(); return 1; }

        for (int i = 0; i < 10; i++) capture.getFrame(buf, BUF, 500);
        if (getenv("AKA_UVC_BENCH_RESUME")) {
            printf("[UVC-BENCH] pause/resume test\n");
            capture.pause();
            const char* pause_env = getenv("AKA_UVC_BENCH_PAUSE_SECONDS");
            const int pause_seconds = pause_env ? atoi(pause_env) : 1;
            sleep(pause_seconds > 0 ? pause_seconds : 1);
            if (capture.resume() != 0) {
                free(buf);
                capture.close();
                return 1;
            }
        }
        UvcCapture::Stats start_stats = capture.stats();
        UvcCapture::Stats window_stats = start_stats;
        struct timeval start{}, window_start{};
        gettimeofday(&start, nullptr);
        window_start = start;
        uint64_t consumed = 0;
        uint64_t window_consumed = 0;
        uint64_t consumed_bytes = 0;
        uint64_t window_consumed_bytes = 0;

        while (_elapsed_us(start) < bench_seconds * 1000000L) {
            const int frame_len = capture.getFrame(buf, BUF, 500);
            if (frame_len > 0) {
                consumed++;
                window_consumed++;
                consumed_bytes += static_cast<uint64_t>(frame_len);
                window_consumed_bytes += static_cast<uint64_t>(frame_len);
            }
            const long window_us = _elapsed_us(window_start);
            if (window_us >= 10000000L) {
                UvcCapture::Stats now = capture.stats();
                const uint64_t captured = now.captured_frames - window_stats.captured_frames;
                const double seconds = window_us / 1000000.0;
                printf("[UVC-BENCH] window=%.2fs captured=%llu consumed=%llu camera=%.2ffps consumed=%.2ffps avg_frame=%.1fKiB\n",
                       seconds,
                       static_cast<unsigned long long>(captured),
                       static_cast<unsigned long long>(window_consumed),
                       captured / seconds, window_consumed / seconds,
                       window_consumed ? window_consumed_bytes /
                                             static_cast<double>(window_consumed) / 1024.0 : 0.0);
                fflush(stdout);
                window_stats = now;
                window_consumed = 0;
                window_consumed_bytes = 0;
                gettimeofday(&window_start, nullptr);
            }
        }

        const long total_us = _elapsed_us(start);
        UvcCapture::Stats end_stats = capture.stats();
        const uint64_t captured = end_stats.captured_frames - start_stats.captured_frames;
        printf("[UVC-BENCH] total=%.2fs captured=%llu consumed=%llu camera=%.2ffps consumed=%.2ffps avg_frame=%.1fKiB\n",
               total_us / 1000000.0,
               static_cast<unsigned long long>(captured),
               static_cast<unsigned long long>(consumed),
               captured * 1000000.0 / total_us,
               consumed * 1000000.0 / total_us,
               consumed ? consumed_bytes / static_cast<double>(consumed) / 1024.0 : 0.0);
        free(buf);
        capture.close();
        return 0;
    }

    LOGI("Camera opened (%dx%d), warming up 20 frames...", FRAME_WIDTH, FRAME_HEIGHT);
    const size_t BUF = 1024 * 1024;
    uint8_t* buf = (uint8_t*)malloc(BUF);
    for (int i = 0; i < 20; i++) capture.getFrame(buf, BUF, 500);
    int len = capture.getFrame(buf, BUF, 2000);
    capture.close();
    if (len <= 0) { LOGE("No frame received"); free(buf); return 1; }
    LOGI("Got MJPEG frame: %d bytes", len);
    _save_jpeg("capture.jpg", buf, len);
    free(buf);
    return 0;
}

// ── test-yolo ─────────────────────────────────────────────────────────────────
int cmd_test_yolo(const char* model_path, int uvc_index)
{
    rknn_app_context_t ctx;
    if (detect_init(model_path, &ctx) != 0) {
        LOGE("Failed to load model: %s", model_path); return 1;
    }
    int mw = ctx.model_width, mh = ctx.model_height;
    LOGI("Model input %dx%d", mw, mh);

    UvcCapture capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index);
        detect_deinit(&ctx);
        return 1;
    }
    const size_t MBUF = 1024 * 1024;
    uint8_t* mjpeg_buf = (uint8_t*)malloc(MBUF);
    LOGI("Warming up camera (20 frames)...");
    for (int i = 0; i < 20; i++) capture.getFrame(mjpeg_buf, MBUF, 500);
    int jpeg_len = capture.getFrame(mjpeg_buf, MBUF, 2000);
    capture.close();
    if (jpeg_len <= 0) {
        LOGE("No frame"); free(mjpeg_buf); detect_deinit(&ctx); return 1;
    }
    _save_jpeg("capture.jpg", mjpeg_buf, jpeg_len);

    uint8_t* rgb = (uint8_t*)malloc(mw * mh * 3);
    int px=0, py=0; float sc=1.0f;
    if (_decode_mjpeg(mjpeg_buf, jpeg_len, rgb, mw, mh, &px, &py, &sc) != 0) {
        LOGE("JPEG decode failed"); free(mjpeg_buf); free(rgb); detect_deinit(&ctx); return 1;
    }
    free(mjpeg_buf);
    LOGI("Letterbox: scale=%.4f pad=(%d,%d)", sc, px, py);

    // Display buffer at native resolution (reverse letterbox)
    uint8_t* disp = (uint8_t*)malloc(FRAME_WIDTH * FRAME_HEIGHT * 3);
    for (int y = 0; y < FRAME_HEIGHT; y++) {
        int sy = std::max(0, std::min((int)(y * sc) + py, mh - 1));
        for (int x = 0; x < FRAME_WIDTH; x++) {
            int sx = std::max(0, std::min((int)(x * sc) + px, mw - 1));
            const uint8_t* s = rgb + (sy * mw + sx) * 3;
            uint8_t*       d = disp + (y * FRAME_WIDTH + x) * 3;
            d[0]=s[0]; d[1]=s[1]; d[2]=s[2];
        }
    }

    std::vector<detection> dets;
    {
        int sv = dup(STDOUT_FILENO), dn = open("/dev/null", O_WRONLY);
        dup2(dn, STDOUT_FILENO); close(dn);
        detect_run(&ctx, rgb, mw, mh, FRAME_WIDTH, FRAME_HEIGHT, px, py, sc,
                   0.5f, 0.45f, dets, nullptr, nullptr, nullptr, nullptr);
        fflush(stdout); dup2(sv, STDOUT_FILENO); close(sv);
    }
    detect_deinit(&ctx); free(rgb);

    LOGI("Detections: %d", (int)dets.size());
    for (int i = 0; i < (int)dets.size(); i++) {
        const detection& d = dets[i];
        LOGI("  [%d] cls=%d score=%.3f bbox=(%.0f,%.0f,%.0f,%.0f)",
             i, d.cls, d.score, d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h);
        _draw_box(disp, FRAME_WIDTH, FRAME_HEIGHT,
                  (int)(d.bbox.x - d.bbox.w*0.5f),
                  (int)(d.bbox.y - d.bbox.h*0.5f),
                  (int)d.bbox.w, (int)d.bbox.h, 0, 255, 0);
    }
    _save_rgb_as_jpeg("result.jpg", disp, FRAME_WIDTH, FRAME_HEIGHT);
    free(disp);
    return 0;
}

// ── test-motor ────────────────────────────────────────────────────────────────
int cmd_test_motor(const char* uart_dev, int argc, char** argv)
{
    int speed = 0;
    for (int i = 3; i < argc; i++)
        if (strncmp(argv[i], "speed=", 6) == 0) speed = atoi(argv[i]+6);

    printf("=== test-motor: %s ===\n", uart_dev);
    Motor motor(MotorDriverType::UART, uart_dev);

    if (speed > 0) {
        printf("Forward %d%% for 5s...\n", speed);
        motor.forward(speed);
        sleep(5);
        motor.standby();
    } else {
        printf("\n[1] Forward 50%% for 2s\n");  motor.forward(50);  sleep(2);
        printf("\n[2] Stop\n");                  motor.standby();    sleep(1);
        printf("\n[3] Backward 50%% for 2s\n"); motor.backward(50); sleep(2);
        printf("\n[4] Stop\n");                  motor.standby();    sleep(1);
        printf("\n[5] Turn left 50%% for 2s\n"); motor.left(50);    sleep(2);
        printf("\n[6] Stop\n");                  motor.standby();    sleep(1);
        printf("\n[7] Turn right 50%% for 2s\n");motor.right(50);   sleep(2);
        printf("\n[8] Stop\n");                  motor.standby();
    }
    printf("\n=== test-motor DONE ===\n");
    return 0;
}

// ── cmd_test_arm ──────────────────────────────────────────────────────────────
// Usage:
//   tennis test-arm [dev]                     -- show help
//   tennis test-arm [dev] grab                -- grab sequence
//   tennis test-arm [dev] release             -- release gripper
//   tennis test-arm [dev] show                -- lift and show
//   tennis test-arm [dev] pos                 -- move to home/ready
//   tennis test-arm [dev] demo                -- pos->grab->show->release
//   tennis test-arm [dev] <a0> <a1> <a2>      -- set 3 servo angles (0~270)
//   tennis test-arm [dev] set <id> <angle>    -- set single servo angle
int cmd_test_arm(const char* uart_dev, int argc, char** argv)
{
    // argv layout: argv[0]=tennis argv[1]="test-arm" argv[2]=dev_or_cmd ...
    // uart_dev is already resolved by main; remaining args start at argv[3]
    printf("=== test-arm  dev=%s ===\n", uart_dev);

    Arm arm(uart_dev, 115200);

    // Collect sub-args: everything after argv[2] (the dev)
    int sub_argc = argc - 3;   // args after <dev>
    char** sub = argv + 3;

    if (sub_argc == 0) {
        printf(
            "Usage:\n"
            "  tennis test-arm [dev] grab\n"
            "  tennis test-arm [dev] release\n"
            "  tennis test-arm [dev] show\n"
            "  tennis test-arm [dev] pos\n"
            "  tennis test-arm [dev] demo\n"
            "  tennis test-arm [dev] <a0> <a1> <a2>   (set 3 servo angles 0~270)\n"
            "  tennis test-arm [dev] set <id> <angle>  (set single servo)\n"
        );
        return 0;
    }

    const char* cmd = sub[0];

    if (strcmp(cmd, "grab") == 0) {
        printf("Executing grab sequence...\n"); fflush(stdout);
        arm.grab();
        printf("Done.\n");
        return 0;
    }
    if (strcmp(cmd, "release") == 0) {
        printf("Releasing gripper...\n"); fflush(stdout);
        arm.release();
        printf("Done.\n");
        return 0;
    }
    if (strcmp(cmd, "show") == 0) {
        printf("Showing ball...\n"); fflush(stdout);
        arm.show();
        printf("Done.\n");
        return 0;
    }
    if (strcmp(cmd, "pos") == 0) {
        printf("Moving to home/ready position...\n"); fflush(stdout);
        arm.grab_pos();
        printf("Done.\n");
        return 0;
    }
    if (strcmp(cmd, "demo") == 0) {
        printf("Demo: pos -> grab -> show -> release\n"); fflush(stdout);
        arm.grab_pos();  usleep(1500000);
        arm.grab();      usleep(2000000);
        arm.show();      usleep(2000000);
        arm.release();
        printf("Demo done.\n");
        return 0;
    }
    if (strcmp(cmd, "set") == 0 && sub_argc == 3) {
        int   id    = atoi(sub[1]);
        float angle = atof(sub[2]);
        printf("servo %d -> %.1f deg\n", id, angle); fflush(stdout);
        arm.set_angle(id, angle);
        usleep(1500000);  // 等待舵机到位
        return 0;
    }
    if (sub_argc == 3) {
        float a0 = atof(sub[0]);
        float a1 = atof(sub[1]);
        float a2 = atof(sub[2]);
        printf("servo 0=%.0f  1=%.0f  2=%.0f\n", a0, a1, a2); fflush(stdout);
        arm.set_angle(0, a0);
        arm.set_angle(1, a1);
        arm.set_angle(2, a2);
        usleep(1500000);  // 等待舵机到位
        return 0;
    }

    printf("Unknown arm command: %s\n", cmd);
    return 1;
}

// ── test-bucket ───────────────────────────────────────────────────────────────
// 识别红色桶：RGB→HSV，双区间红色掩膜，连通域过滤，输出 bbox 并保存标注图
//
// HSV 范围（与 Python 版一致）：
//   区间1: H∈[0,10]   S∈[80,255] V∈[50,255]
//   区间2: H∈[170,180] S∈[80,255] V∈[50,255]
// OpenCV H 范围 [0,180]，这里直接用 [0,360] 归一化后等比换算：
//   实际存储 H = H_opencv * 2，即区间1 H∈[0,20], 区间2 H∈[340,360]

struct BucketBox { int x, y, w, h; int area; };

// RGB(0-255) → HSV: H∈[0,360) S∈[0,255] V∈[0,255]
static void _rgb2hsv(uint8_t r, uint8_t g, uint8_t b,
                     int& H, int& S, int& V)
{
    int R = r, G = g, B = b;
    int cmax = std::max({R, G, B});
    int cmin = std::min({R, G, B});
    int delta = cmax - cmin;

    V = cmax;
    S = (cmax == 0) ? 0 : (delta * 255 / cmax);

    if (delta == 0) { H = 0; return; }
    float h;
    if      (cmax == R) h = 60.0f * (((G - B) % (delta * 6)) / (float)delta);
    else if (cmax == G) h = 60.0f * ((float)(B - R) / delta + 2.0f);
    else                h = 60.0f * ((float)(R - G) / delta + 4.0f);
    if (h < 0) h += 360.0f;
    H = (int)h;
}

static bool _is_red(int H, int S, int V)
{
    if (S < 80 || V < 50) return false;
    return (H <= 20) || (H >= 340);   // 对应 OpenCV H∈[0,10]∪[170,180]
}

// 简单 4-连通 Union-Find 连通域标记（不依赖任何外部库）
static std::vector<BucketBox> _find_red_boxes(const uint8_t* rgb, int iw, int ih,
                                               int min_area = 1000)
{
    // 1. 生成掩膜
    std::vector<uint8_t> mask(iw * ih, 0);
    for (int i = 0; i < iw * ih; i++) {
        int H, S, V;
        _rgb2hsv(rgb[i*3], rgb[i*3+1], rgb[i*3+2], H, S, V);
        mask[i] = _is_red(H, S, V) ? 1 : 0;
    }

    // 2. 两遍扫描连通域标记（two-pass）
    std::vector<int> label(iw * ih, 0);
    std::vector<int> parent;
    parent.push_back(0); // label 0 = background
    int next_label = 1;

    // 辅助：路径压缩 find（用 while 循环避免递归，不依赖 std::function）
    auto uf_find = [&](int x) -> int {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](int a, int b) {
        a = uf_find(a); b = uf_find(b);
        if (a != b) parent[a] = b;
    };

    // 第一遍
    for (int y = 0; y < ih; y++) {
        for (int x = 0; x < iw; x++) {
            int idx = y * iw + x;
            if (!mask[idx]) { label[idx] = 0; continue; }
            int up   = (y > 0) ? label[(y-1)*iw + x] : 0;
            int left = (x > 0) ? label[y*iw + x - 1]  : 0;
            if (up == 0 && left == 0) {
                label[idx] = next_label;
                parent.push_back(next_label);
                next_label++;
            } else if (up != 0 && left == 0) {
                label[idx] = up;
            } else if (up == 0 && left != 0) {
                label[idx] = left;
            } else {
                label[idx] = left;
                unite(up, left);
            }
        }
    }

    // 第二遍：统计每个根标签的 bbox 和 area
    std::vector<int> x0_v(next_label, iw), y0_v(next_label, ih);
    std::vector<int> x1_v(next_label, -1), y1_v(next_label, -1);
    std::vector<int> cnt(next_label, 0);
    for (int y = 0; y < ih; y++) {
        for (int x = 0; x < iw; x++) {
            int idx = y * iw + x;
            if (!label[idx]) continue;
            int root = uf_find(label[idx]);
            label[idx] = root;
            cnt[root]++;
            if (x < x0_v[root]) x0_v[root] = x;
            if (y < y0_v[root]) y0_v[root] = y;
            if (x > x1_v[root]) x1_v[root] = x;
            if (y > y1_v[root]) y1_v[root] = y;
        }
    }

    std::vector<BucketBox> boxes;
    for (int l = 1; l < next_label; l++) {
        if (uf_find(l) != l) continue;    // 非根节点跳过
        if (cnt[l] < min_area) continue;
        BucketBox b;
        b.x = x0_v[l]; b.y = y0_v[l];
        b.w = x1_v[l] - x0_v[l] + 1;
        b.h = y1_v[l] - y0_v[l] + 1;
        b.area = cnt[l];
        boxes.push_back(b);
    }
    return boxes;
}

int cmd_test_bucket(int uvc_index)
{
    UvcCapture capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index); return 1;
    }
    LOGI("Camera opened (%dx%d), warming up 20 frames...", FRAME_WIDTH, FRAME_HEIGHT);

    // 创建 imgs/ 目录，若已存在则清空
    mkdir("imgs", 0755);
    _clear_dir("imgs");
    LOGI("imgs/ directory ready");

    const size_t MJPEG_BUF = 1024 * 1024;
    uint8_t* mjpeg_buf = (uint8_t*)malloc(MJPEG_BUF);
    uint8_t* rgb_buf   = (uint8_t*)malloc(FRAME_WIDTH * FRAME_HEIGHT * 3);
    const float FRAME_AREA = (float)(FRAME_WIDTH * FRAME_HEIGHT);

    for (int i = 0; i < 20; i++) capture.getFrame(mjpeg_buf, MJPEG_BUF, 500);

    LOGI("Starting red bucket detection loop (Ctrl-C to stop)...");

    // 采样目标：area 从 0.05 到 0.95，共 20 档，每档只保存一次
    const int   NUM_SLOTS   = 20;
    const float SLOT_MIN    = 0.05f;
    const float SLOT_MAX    = 0.95f;
    const float SLOT_STEP   = (SLOT_MAX - SLOT_MIN) / (NUM_SLOTS - 1); // 间隔
    const float SLOT_HALF   = SLOT_STEP * 0.5f;                         // 捕获半径
    bool saved[NUM_SLOTS]   = {};
    int  saved_cnt          = 0;
    int  skip_frames        = 10;   // 摄像头稳定前跳过的帧数

    while (true) {
        int len = capture.getFrame(mjpeg_buf, MJPEG_BUF, 500);
        if (len <= 0) { printf("[bucket] no frame\n"); fflush(stdout); continue; }

        // 跳过前几帧，等摄像头曝光稳定
        if (skip_frames > 0) { skip_frames--; continue; }

        if (_decode_mjpeg(mjpeg_buf, len, rgb_buf, FRAME_WIDTH, FRAME_HEIGHT,
                          nullptr, nullptr, nullptr) != 0) continue;

        auto boxes = _find_red_boxes(rgb_buf, FRAME_WIDTH, FRAME_HEIGHT, 1000);

        if (boxes.empty()) {
            printf("[bucket] not found\n");
        } else {
            // 取面积最大的一个
            const BucketBox* best = &boxes[0];
            for (const auto& b : boxes)
                if (b.area > best->area) best = &b;

            float area_ratio = best->area / FRAME_AREA;
            int cx = best->x + best->w / 2;
            int cy = best->y + best->h / 2;
            printf("[bucket] found  area=%.3f  box=(%d,%d,%d,%d)  center=(%d,%d)\n",
                   area_ratio, best->x, best->y, best->w, best->h, cx, cy);

            // 检查是否命中某个未保存的档位
            for (int s = 0; s < NUM_SLOTS && saved_cnt < NUM_SLOTS; s++) {
                if (saved[s]) continue;
                float target = SLOT_MIN + s * SLOT_STEP;
                if (fabsf(area_ratio - target) <= SLOT_HALF) {
                    // 复制 rgb，画框 + 文字后保存
                    uint8_t* out = (uint8_t*)malloc(FRAME_WIDTH * FRAME_HEIGHT * 3);
                    memcpy(out, rgb_buf, FRAME_WIDTH * FRAME_HEIGHT * 3);

                    _draw_box(out, FRAME_WIDTH, FRAME_HEIGHT,
                              best->x, best->y, best->w, best->h, 0, 255, 0);

                    char label[32];
                    snprintf(label, sizeof(label), "area=%.3f", area_ratio);
                    // 文字背景（黑色矩形）
                    int tx = best->x + 2, ty = best->y + 2;
                    int tw = (int)(strlen(label)) * 12 + 4, th = 18;
                    for (int py = ty; py < ty + th && py < FRAME_HEIGHT; py++)
                        for (int px = tx; px < tx + tw && px < FRAME_WIDTH; px++) {
                            uint8_t* p = out + (py * FRAME_WIDTH + px) * 3;
                            p[0] = p[1] = p[2] = 0;
                        }
                    _draw_text(out, FRAME_WIDTH, FRAME_HEIGHT, tx + 2, ty + 2,
                               label, 255, 255, 0, 2);

                    char fname[64];
                    snprintf(fname, sizeof(fname), "imgs/bucket_%02d_%.2f.jpg", s, target);
                    _save_rgb_as_jpeg(fname, out, FRAME_WIDTH, FRAME_HEIGHT);
                    free(out);

                    saved[s] = true;
                    saved_cnt++;
                    LOGI("Saved slot %d/%d → %s  (area=%.3f)",
                         saved_cnt, NUM_SLOTS, fname, area_ratio);
                    break;
                }
            }
        }
        fflush(stdout);
    }

    free(mjpeg_buf);
    free(rgb_buf);
    capture.close();
    return 0;
}

// ── detect_bucket_frame: reusable helper for main loop ────────────────────────
bool detect_bucket_frame(const uint8_t* rgb, int width, int height,
                         BucketResult& out, int min_area)
{
    out.found = false;
    float frame_area = (float)(width * height);
    auto boxes = _find_red_boxes(rgb, width, height, min_area);
    if (boxes.empty()) return false;

    // pick largest
    const BucketBox* best = &boxes[0];
    for (const auto& b : boxes)
        if (b.area > best->area) best = &b;

    out.found      = true;
    out.area_ratio = best->area / frame_area;
    out.x = best->x; out.y = best->y;
    out.w = best->w; out.h = best->h;
    out.cx = best->x + best->w / 2;
    out.cy = best->y + best->h / 2;
    return true;
}

// ── test-feetech ─────────────────────────────────────────────────────────────
int cmd_test_feetech(const char* uart_dev, int argc, char** argv)
{
    const char* cmd = (argc >= 4) ? argv[3] : "scan";
    printf("=== test-feetech dev=%s cmd=%s ===\n", uart_dev, cmd);

    feetech::FeetechBus bus(uart_dev, 1000000);
    if (!bus.open()) {
        printf("open failed: %s\n", bus.last_error().c_str());
        return 1;
    }

    if (strcmp(cmd, "scan") == 0) {
        auto ids = bus.scan(1, 9);
        printf("found %zu motor(s):", ids.size());
        for (int id : ids) printf(" %d", id);
        printf("\n");
        return ids.empty() ? 1 : 0;
    }

    if (strcmp(cmd, "read") == 0) {
        bool ok = true;
        for (int id = 1; id <= 9; id++) {
            feetech::MotorStatus st;
            if (bus.read_status(id, st)) {
                printf("id=%d pos=%d vel=%d voltage=%d temp=%d\n",
                       id, st.position, st.velocity, st.voltage, st.temperature);
            } else {
                printf("id=%d read failed: %s\n", id, bus.last_error().c_str());
                ok = false;
            }
        }
        return ok ? 0 : 1;
    }

    if (strcmp(cmd, "torque-off") == 0) {
        bool ok = true;
        for (int id = 1; id <= 9; id++) ok = bus.enable_torque(id, false) && ok;
        printf("torque-off %s\n", ok ? "ok" : "failed");
        return ok ? 0 : 1;
    }

    printf("Usage: tennis test-feetech [dev] <scan|read|torque-off>\n");
    return 2;
}

// ── test-base ────────────────────────────────────────────────────────────────
int cmd_test_base(const char* uart_dev, int argc, char** argv)
{
    const char* cmd = (argc >= 4) ? argv[3] : "stop";
    int level = (argc >= 5) ? atoi(argv[4]) : 0;
    int duration_ms = 350;

    printf("=== test-base dev=%s cmd=%s level=%d duration=%dms ===\n",
           uart_dev, cmd, level, duration_ms);

    feetech::FeetechBus bus(uart_dev, 1000000);
    if (!bus.open()) {
        printf("open failed: %s\n", bus.last_error().c_str());
        return 1;
    }
    OmniBase base(bus);
    if (!base.configure()) {
        printf("base configure failed: %s\n", bus.last_error().c_str());
        base.stop();
        return 1;
    }

    bool ok = true;
    if      (strcmp(cmd, "forward") == 0)      ok = base.forward(level);
    else if (strcmp(cmd, "backward") == 0)     ok = base.backward(level);
    else if (strcmp(cmd, "left") == 0)         ok = base.left(level);
    else if (strcmp(cmd, "right") == 0)        ok = base.right(level);
    else if (strcmp(cmd, "rotate-left") == 0)  ok = base.rotate_left(level);
    else if (strcmp(cmd, "rotate-right") == 0) ok = base.rotate_right(level);
    else if (strcmp(cmd, "stop") == 0)         ok = base.stop();
    else {
        printf("Usage: tennis test-base [dev] <forward|backward|left|right|rotate-left|rotate-right|stop> [level]\n");
        base.stop();
        return 2;
    }

    if (strcmp(cmd, "stop") != 0) {
        usleep(duration_ms * 1000);
        base.stop();
    }
    printf("test-base %s\n", ok ? "ok" : bus.last_error().c_str());
    return ok ? 0 : 1;
}

static bool _read_pos(feetech::FeetechBus& bus, int id, int& pos) {
    return bus.read_u16(id, feetech::reg::PRESENT_POSITION, pos, true);
}

struct JointDef { const char* name; int id; bool full_turn; };

struct RangeRecord {
    int min = 4095;
    int max = 0;
    int raw = 0;
    bool got = false;
};

static bool _record_all_ranges_until_enter(feetech::FeetechBus& bus,
                                           const JointDef* joints,
                                           int joint_count,
                                           RangeRecord* ranges)
{
    printf("\n一次性校准 1..6 号机械臂电机。\n");
    printf("请把所有机械臂关节都在安全范围内来回转到两端。\n");
    printf("程序会同时记录每个电机的 raw/min/max。\n");
    printf("全部关节都转完后按 ENTER。不要硬顶机械极限。\n\n");
    fflush(stdout);

    std::atomic<bool> done(false);
    std::thread waiter([&done]() {
        int c;
        while ((c = getchar()) != '\n' && c != EOF) {}
        done = true;
    });

    while (!done) {
        for (int i = 0; i < joint_count; i++) {
            if (joints[i].full_turn) continue;
            int pos = 0;
            if (_read_pos(bus, joints[i].id, pos)) {
                ranges[i].raw = pos;
                if (!ranges[i].got) {
                    ranges[i].min = ranges[i].max = pos;
                    ranges[i].got = true;
                }
                if (pos < ranges[i].min) ranges[i].min = pos;
                if (pos > ranges[i].max) ranges[i].max = pos;
            }
        }
        printf("\033[6A");
        for (int i = 0; i < joint_count; i++) {
            if (joints[i].full_turn) {
                printf("  id=%d %-18s full-turn range=[0,4095]\n", joints[i].id, joints[i].name);
            } else {
                printf("  id=%d %-18s raw=%4d min=%4d max=%4d span=%4d\n",
                       joints[i].id, joints[i].name, ranges[i].raw,
                       ranges[i].min, ranges[i].max, ranges[i].max - ranges[i].min);
            }
        }
        fflush(stdout);
        usleep(70000);
    }
    waiter.join();
    printf("\n");

    bool ok = true;
    for (int i = 0; i < joint_count; i++) {
        if (joints[i].full_turn) continue;
        if (!ranges[i].got || ranges[i].max - ranges[i].min < 20) {
            printf("  range invalid: id=%d %s min=%d max=%d\n",
                   joints[i].id, joints[i].name, ranges[i].min, ranges[i].max);
            ok = false;
        }
    }
    return ok;
}

static int _center_to_homing(int range_min, int range_max) {
    int center = (range_min + range_max) / 2;
    return center - 2047;
}

static const char* kPosePath = "config/lekiwi_arm_poses.txt";

static ArmRawPose _read_current_arm_pose(feetech::FeetechBus& bus) {
    const char* names[] = {
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper",
    };
    ArmRawPose pose;
    for (int i = 0; i < 6; i++) {
        int raw = 0;
        if (_read_pos(bus, i + 1, raw)) pose.joints[names[i]] = raw;
    }
    return pose;
}

static void _print_pose(const char* name, const ArmRawPose& pose) {
    printf("[%s]\n", name);
    for (const auto& kv : pose.joints) {
        printf("  %-18s %4d\n", kv.first.c_str(), kv.second);
    }
}

static int cmd_calibrate_lekiwi_arm(const char* uart_dev) {
    printf("=== LeKiwi arm calibration dev=%s ===\n", uart_dev);
    printf("校准方式：一次性转动所有机械臂关节到安全两端，程序自动计算中点作为零位。\n");
    printf("输出文件：config/lekiwi_calibration.json\n\n");

    feetech::FeetechBus bus(uart_dev, 1000000);
    if (!bus.open()) {
        printf("open failed: %s\n", bus.last_error().c_str());
        return 1;
    }

    auto ids = bus.scan(1, 9);
    printf("found %zu motor(s):", ids.size());
    for (int id : ids) printf(" %d", id);
    printf("\n");
    for (int id = 1; id <= 6; id++) {
        bool found = false;
        for (int got : ids) if (got == id) found = true;
        if (!found) {
            printf("missing required arm motor id=%d\n", id);
            return 1;
        }
    }

    printf("关闭 1..6 号机械臂扭矩...\n");
    for (int id = 1; id <= 6; id++) bus.enable_torque(id, false);

    JointDef joints[] = {
        {"arm_shoulder_pan", 1, false},
        {"arm_shoulder_lift", 2, false},
        {"arm_elbow_flex", 3, false},
        {"arm_wrist_flex", 4, false},
        {"arm_wrist_roll", 5, true},
        {"arm_gripper", 6, false},
    };
    const int joint_count = (int)(sizeof(joints) / sizeof(joints[0]));

    printf("\n当前要记录的机械臂电机：\n");
    for (int i = 0; i < joint_count; i++) {
        printf("  id=%d  %s%s\n", joints[i].id, joints[i].name,
               joints[i].full_turn ? " (continuous/full-turn)" : "");
    }

    RangeRecord ranges[joint_count];
    for (int i = 0; i < joint_count; i++) {
        if (joints[i].full_turn) {
            ranges[i].got = true;
            ranges[i].min = 0;
            ranges[i].max = 4095;
        }
    }
    // Reserve six display lines before the ANSI cursor-up refresh loop.
    for (int i = 0; i < joint_count; i++) printf("\n");
    if (!_record_all_ranges_until_enter(bus, joints, joint_count, ranges)) {
        printf("calibration aborted: invalid range detected\n");
        return 1;
    }

    LekiwiCalibration cal;
    for (int i = 0; i < joint_count; i++) {
        const auto& j = joints[i];
        JointCalibration jc;
        jc.id = j.id;
        jc.drive_mode = 0;
        if (j.full_turn) {
            jc.homing_offset = 0;
            jc.range_min = 0;
            jc.range_max = 4095;
        } else {
            jc.range_min = ranges[i].min;
            jc.range_max = ranges[i].max;
            jc.homing_offset = _center_to_homing(jc.range_min, jc.range_max);
            int center = (jc.range_min + jc.range_max) / 2;
            printf("  id=%d %-18s center=%d homing_offset=%d range=[%d,%d]\n",
                   j.id, j.name, center, jc.homing_offset, jc.range_min, jc.range_max);
        }
        cal.set(j.name, jc);
    }

    cal.set("base_left_wheel",  {7, 0, 0, 0, 4095});
    cal.set("base_back_wheel",  {8, 0, 0, 0, 4095});
    cal.set("base_right_wheel", {9, 0, 0, 0, 4095});

    mkdir("config", 0755);
    const char* out = "config/lekiwi_calibration.json";
    if (!cal.save(out)) {
        printf("failed to save %s\n", out);
        return 1;
    }

    LekiwiCalibration verify;
    if (!verify.load(out)) {
        printf("saved file failed to reload: %s\n", verify.last_error().c_str());
        return 1;
    }

    printf("\n校准文件已保存：%s\n", out);
    printf("校准结束后保持机械臂扭矩关闭。需要动作测试时再运行 test-new-arm pos/set/grab。\n");
    return 0;
}

// ── test-new-arm ─────────────────────────────────────────────────────────────
int cmd_test_new_arm(const char* uart_dev, int argc, char** argv)
{
    const char* cmd = (argc >= 4) ? argv[3] : "help";
    printf("=== test-new-arm dev=%s cmd=%s ===\n", uart_dev, cmd);

    auto print_usage = []() {
        printf("Usage:\n");
        printf("  tennis test-new-arm [dev] calibrate|calib-check|config-check|motion-check|pos|grab|ik-pick|ik-put|release|release-pos|show|torque-off\n");
        printf("  tennis test-new-arm [dev] task home|carry|place-approach|place-release|place-cycle\n");
        printf("  tennis test-new-arm [dev] set <joint_name> <deg>\n");
        printf("  tennis test-new-arm [dev] raw <joint_name> <raw_0_4095>\n");
        printf("  tennis test-new-arm [dev] pose-save <name>\n");
        printf("  tennis test-new-arm [dev] pose-run <name>\n");
        printf("  tennis test-new-arm [dev] pose-list\n");
    };

    if (strcmp(cmd, "help") == 0) {
        print_usage();
        return 0;
    }

    if (strcmp(cmd, "calibrate") == 0) {
        return cmd_calibrate_lekiwi_arm(uart_dev);
    }

    if (strcmp(cmd, "config-check") == 0) {
        LeKiwiPickConfig config;
        if (!config.load()) {
            printf("failed to load config/lekiwi_pick_config.txt\n");
            return 1;
        }
        std::string validation_error;
        if (!config.validate(validation_error)) {
            printf("config invalid: %s\n", validation_error.c_str());
            return 1;
        }
        printf("config ok: pick/place trajectories are inside configured safety limits\n");
        printf("motion: level=%d arm=%.1f deg/s home=%.1f deg/s gripper=%.1f deg/s "
               "pick_settle=%d ms carry_settle=%d ms place_settle=%d ms\n",
               config.motion_speed_level, config.arm_motion_speed_deg_s(),
               config.arm_home_speed_deg_s, config.gripper_speed_deg_s,
               config.pick_settle_ms, config.carry_settle_ms,
               config.place_settle_ms);
        printf("ball: far=%d near=%d backward=%d fine_turn=%d slowdown=%d%% "
               "stop=%d+-%d px center=+-%d px stable=%d frames\n",
               config.ball_far_speed, config.ball_near_speed,
               config.ball_backward_speed, config.ball_fine_turn_speed,
               config.ball_slowdown_start_percent, config.ball_stop_size_px,
               config.ball_stop_tolerance_px,
               config.ball_center_tolerance_px, config.ball_stable_frames);
        printf("bucket: metric=sqrt(w*h) far=%d near=%d backward=%d turn=%d "
               "fine_turn=%d slowdown=%d%% stop=%d px center=+-%d px "
               "stable=%d frames lookahead=%d ms\n",
               config.bucket_far_speed, config.bucket_near_speed,
               config.bucket_backward_speed, config.bucket_turn_speed,
               config.bucket_fine_turn_speed,
               config.bucket_slowdown_start_percent,
               config.bucket_stop_size_px,
               config.bucket_center_tolerance_px,
               config.bucket_stable_frames, config.braking_lookahead_ms);
        return 0;
    }

    if (strcmp(cmd, "motion-check") == 0) {
        auto run_case = [](bool bucket) {
            LeKiwiMoveController controller(FRAME_WIDTH, FRAME_HEIGHT);
            float size = bucket ? 100.0f : 55.0f;
            const float target = bucket ? controller.bucket_target_position()
                                        : controller.target_position();
            float velocity = 0.0f;
            bool saw_brake = false;
            bool saw_slowdown = false;
            bool reached = false;
            const char* previous_label = "";
            for (int frame = 0; frame < 240; frame++) {
                LeKiwiMoveController::Command motion;
                if (bucket) {
                    const int w = (int)std::lround(size * 1.25f);
                    const int h = (int)std::lround(size * 0.80f);
                    motion = controller.update_bucket(true, FRAME_WIDTH / 2, w, h);
                } else {
                    detection det{};
                    det.bbox.x = FRAME_WIDTH / 2;
                    det.bbox.y = FRAME_HEIGHT / 2;
                    det.bbox.w = size;
                    det.bbox.h = size;
                    motion = controller.update_ball({det});
                }
                if (strcmp(previous_label, motion.label) != 0) {
                    printf("motion-check %s frame=%d label=%s metric=%d pred=%d "
                           "L=%d R=%d\n",
                           bucket ? "bucket" : "ball", frame, motion.label,
                           motion.distance_size, motion.predicted_size,
                           motion.left_speed, motion.right_speed);
                    previous_label = motion.label;
                }
                saw_brake = saw_brake || strstr(motion.label, "BRAKE") != nullptr;
                const int forward = (motion.left_speed + motion.right_speed) / 2;
                saw_slowdown = saw_slowdown || (forward > 0 && forward < 50);
                velocity += 0.35f * (forward - velocity);
                size += velocity * 0.08f;
                if (motion.reached) {
                    reached = true;
                    break;
                }
                usleep(50000);
            }
            const float allowed_error = bucket ? 20.0f : 10.0f;
            const bool ok = reached && saw_slowdown &&
                            std::abs(size - target) <= allowed_error;
            printf("motion-check %s result=%s brake=%d slowdown=%d final=%.1f\n",
                   bucket ? "bucket" : "ball",
                   ok ? "ok" : "failed",
                   saw_brake ? 1 : 0, saw_slowdown ? 1 : 0, size);
            return ok;
        };
        return run_case(false) && run_case(true) ? 0 : 1;
    }

    feetech::FeetechBus bus(uart_dev, 1000000);
    if (!bus.open()) {
        printf("open failed: %s\n", bus.last_error().c_str());
        return 1;
    }

    if (strcmp(cmd, "pose-save") == 0 && argc >= 5) {
        LekiwiArmPoses poses;
        poses.load(kPosePath);
        ArmRawPose pose = _read_current_arm_pose(bus);
        if (pose.joints.size() != 6) {
            printf("failed to read all arm joints\n");
            return 1;
        }
        poses.set(argv[4], pose);
        mkdir("config", 0755);
        if (!poses.save(kPosePath)) {
            printf("failed to save %s\n", kPosePath);
            return 1;
        }
        _print_pose(argv[4], pose);
        printf("saved to %s\n", kPosePath);
        return 0;
    }

    if (strcmp(cmd, "pose-list") == 0) {
        LekiwiArmPoses poses;
        if (!poses.load(kPosePath)) {
            printf("no pose file: %s\n", poses.last_error().c_str());
            return 1;
        }
        auto names = poses.names();
        printf("poses:");
        for (const auto& name : names) printf(" %s", name.c_str());
        printf("\n");
        return 0;
    }

    FeetechArm arm(bus);
    if (strcmp(cmd, "calib-check") == 0) {
        if (arm.has_calibration()) {
            printf("calibration ok\n");
            return 0;
        }
        printf("missing/invalid calibration: %s\n", arm.calibration_error().c_str());
        return 1;
    }
    if (!arm.has_calibration()) {
        printf("missing/invalid calibration: %s\n", arm.calibration_error().c_str());
        return 1;
    }
    if (!arm.configure()) {
        printf("arm configure failed: %s\n", bus.last_error().c_str());
        return 1;
    }

    bool ok = true;
    std::string command_error;
    if      (strcmp(cmd, "pos") == 0)        ok = arm.grab_pos();
    else if (strcmp(cmd, "grab") == 0)       ok = arm.grab();
    else if (strcmp(cmd, "ik-pick") == 0) {
        LeKiwiArmController ctrl(arm);
        ok = ctrl.begin_pick();
        int tick = 0;
        while (ok && !ctrl.done() && !ctrl.failed() && tick < 600) {
            ok = ctrl.tick();
            if ((tick % 10) == 0) {
                printf("ik-pick step=%zu/%zu %s\n",
                       ctrl.step_index() + 1,
                       ctrl.step_count(),
                       ctrl.current_step_label());
                fflush(stdout);
            }
            tick++;
        }
        float gripper = 0.0f;
        bool holding = false;
        if (ok && ctrl.done() && !ctrl.failed()) holding = ctrl.verify_grab(&gripper);
        if (ctrl.failed()) ok = false;
        printf("ik-pick done=%d failed=%d gripper=%.1f holding=%s\n",
               ctrl.done() ? 1 : 0, ctrl.failed() ? 1 : 0,
               gripper, holding ? "yes" : "no");
        ok = ok && ctrl.done() && !ctrl.failed();
        if (!ok) command_error = ctrl.last_error();
    }
    else if (strcmp(cmd, "ik-put") == 0 ||
             (strcmp(cmd, "task") == 0 && argc >= 5)) {
        LeKiwiArmController ctrl(arm);
        ok = strcmp(cmd, "ik-put") == 0
            ? ctrl.begin_put() : ctrl.begin_stage(argv[4]);
        int tick = 0;
        while (ok && !ctrl.done() && !ctrl.failed() && tick < 1200) {
            ok = ctrl.tick();
            if ((tick % 10) == 0) {
                printf("%s step=%zu/%zu %s\n", cmd,
                       ctrl.step_index() + 1,
                       ctrl.step_count(),
                       ctrl.current_step_label());
                fflush(stdout);
            }
            tick++;
        }
        if (ctrl.failed()) ok = false;
        printf("%s done=%d failed=%d\n", cmd,
               ctrl.done() ? 1 : 0, ctrl.failed() ? 1 : 0);
        ok = ok && ctrl.done() && !ctrl.failed();
        if (!ok) command_error = ctrl.last_error();
    }
    else if (strcmp(cmd, "release") == 0)    ok = arm.release();
    else if (strcmp(cmd, "release-pos") == 0)ok = arm.release_pos();
    else if (strcmp(cmd, "show") == 0)       ok = arm.show();
    else if (strcmp(cmd, "torque-off") == 0) ok = arm.stop_torque();
    else if (strcmp(cmd, "pose-run") == 0 && argc >= 5) ok = arm.run_pose(argv[4], 1000);
    else if (strcmp(cmd, "set") == 0 && argc >= 6) {
        ok = arm.set_joint_deg(argv[4], atof(argv[5]));
        usleep(1000 * 1000);
    } else if (strcmp(cmd, "raw") == 0 && argc >= 6) {
        ok = arm.set_joint_raw(argv[4], atoi(argv[5]));
        usleep(1000 * 1000);
    } else {
        print_usage();
        return 2;
    }

    const std::string& error = command_error.empty() ? arm.last_error() : command_error;
    printf("test-new-arm %s\n", ok ? "ok" : error.c_str());
    return ok ? 0 : 1;
}
