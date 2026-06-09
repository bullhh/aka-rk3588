// Source: aka-sg2002/tennis.cpp - ported to rk3588 / RKNN YOLOv8
// Description: chase-only state machine
//   - UVC camera (MJPEG) -> libjpeg-turbo decode -> RKNN YOLOv8 inference
//   - Motor driver abstraction: UART (ESP32-C3) or PWM
//   - Smooth continuous differential steering (no stop-and-turn)
//   - Ctrl-C safe exit

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <algorithm>
#include <vector>
#include <utility>

#include <turbojpeg.h>

#include "logger.hpp"
#include "motor/motor.hpp"
#include "capture/uvc_capture.hpp"
#include "detect/detect.hpp"
#include "test_cmds.hpp"
#include "arm/arm.hpp"
#include "feetech/feetech_bus.hpp"
#include "robot/omni_base.hpp"
#include "robot/feetech_arm.hpp"
#include "robot/lekiwi_task_controller.hpp"

// ── Build-time tunables ───────────────────────────────────────────────────────
#define ENABLE_SAVE_IMAGE 0
#define SKIP_FRAMES       0  // Process every Nth frame (0=process all)

// ── Camera / model parameters ─────────────────────────────────────────────────
static const int FRAME_WIDTH  = 640;
static const int FRAME_HEIGHT = 480;
static const int MODEL_W      = 640;
static const int MODEL_H      = 640;

// ── Control parameters ────────────────────────────────────────────────────────
static const int   CHASE_SPEED_FAR  = 40;
static const int   CHASE_SPEED_NEAR = 3;

static const float AREA_FAR         = 0.02f;
static const float AREA_NEAR        = 0.35f;
static const float AREA_BRAKE       = 0.20f;

static const int   BRAKE_SPEED      = 3;
static const float AREA_STOP        = 0.28f;
static const float AREA_REVERSE     = 0.50f;
static const int   REVERSE_SPEED    = 15;

static const float AREA_STOP_EXIT   = 0.20f;
static const int   STOP_CONFIRM_CNT = 4;
static const int   BRAKE_PULSE_US   = 350000;
static const int   STOP_CENTER_OFFSET = 90;

static const float K_TURN              = 25.0f;
static const int   MAX_TURN_BIAS_FAR   = 5;
static const int   MAX_TURN_BIAS_NEAR  = 10;
static const int   CENTER_DEAD_ZONE    = 15;
static const int   STOP_CENTER_ZONE    = 5;
static const int   ALIGN_PIVOT_SPD     = 15;
static const int   ALIGN_PIVOT_MIN     = 3;

static const int   SEARCH_FRAMES    = 25;
static const int   SEARCH_PIVOT_SPD = 10;

static const int   ALIGN_STALL_FRAMES  = 20;
static const int   ALIGN_STALL_MOVE_PX = 10;
static const int   ALIGN_KICK_SPD      = 35;
static const int   ALIGN_KICK_US       = 180000;

// ── Bucket approach parameters ────────────────────────────────────────────────
// 桶面积 > 此值时视为"足够近"可放球
static const float BUCKET_AREA_DEPOSIT  = 0.90f;
// 桶面积 > 此值时制动（避免撞桶）
static const float BUCKET_AREA_BRAKE    = 0.70f;
// 趋近桶时的前进速度
static const int   BUCKET_APPROACH_SPD  = 25;
// 趋近桶时的制动速度
static const int   BUCKET_BRAKE_SPD     = 5;
// 转向增益（与追球相同逻辑）
static const float BUCKET_K_TURN        = 20.0f;
static const int   BUCKET_MAX_BIAS      = 8;
// 搜索桶时的旋转速度
static const int   BUCKET_SEARCH_SPD    = 12;
// 连续多少帧找不到桶 → 旋转搜索
static const int   BUCKET_LOST_FRAMES   = 10;
// 找到桶后确认帧数（防抖）
static const int   BUCKET_CONFIRM_CNT   = 3;

// ── Game state machine ────────────────────────────────────────────────────────
enum class GameState {
    CHASE_BALL,      // 追球，直到抓到
    GRAB,            // 已到位，执行抓球动作（blocking，单次）
    FIND_BUCKET,     // 旋转搜索红色桶
    APPROACH_BUCKET, // 趋近桶
    DEPOSIT,         // 放球（blocking，单次），完成后回到 CHASE_BALL
    PICK_BALL,       // LeKiwi: IK/P 控制夹球
    PUT_BALL,        // LeKiwi: IK/P 控制放球
};

class DriveAdapter {
public:
    virtual ~DriveAdapter() = default;
    virtual void drive(int left_speed, int right_speed) = 0;
    virtual void brake() = 0;
    virtual void standby() = 0;
};

// ── Globals for signal handler ────────────────────────────────────────────────
static Motor*              g_motor      = nullptr;
static DriveAdapter*       g_drive      = nullptr;
static UvcCapture*         g_capture    = nullptr;
static rknn_app_context_t* g_rknn_ctx   = nullptr;
static Arm*                g_arm        = nullptr;
static int                 g_saved_stderr = -1;
static int                 g_devnull    = -1;

static void cleanup_and_exit() {
    if (g_drive)    g_drive->standby();
    else if (g_motor) g_motor->standby();
    if (g_capture)  g_capture->close();
    if (g_rknn_ctx) detect_deinit(g_rknn_ctx);
    if (g_saved_stderr >= 0 && g_devnull >= 0)
        dup2(g_saved_stderr, STDERR_FILENO);
}

static void signal_handler(int /*sig*/) {
    cleanup_and_exit();
    exit(0);
}

// ── Timing helper ─────────────────────────────────────────────────────────────
static long elapsed_us(const struct timeval& start) {
    struct timeval now; gettimeofday(&now, nullptr);
    return (now.tv_sec - start.tv_sec) * 1000000L + (now.tv_usec - start.tv_usec);
}

// ── JPEG decode -> RGB letterbox ─────────────────────────────────────────────
static int decode_mjpeg(const uint8_t* jpeg_data, size_t jpeg_len,
                        uint8_t* rgb_out, int out_w, int out_h,
                        int* pad_x = nullptr, int* pad_y = nullptr,
                        float* scale_out = nullptr,
                        long* t_header = nullptr, long* t_decomp = nullptr,
                        long* t_copy = nullptr)
{
    struct timeval t0;
    tjhandle tj = tjInitDecompress();
    if (!tj) return -1;

    gettimeofday(&t0, nullptr);
    int w, h, subsamp, colorspace;
    if (tjDecompressHeader3(tj, jpeg_data, jpeg_len, &w, &h, &subsamp, &colorspace) < 0)
        { tjDestroy(tj); return -1; }
    if (t_header) *t_header = elapsed_us(t0);

    float scale = std::min((float)out_w / w, (float)out_h / h);
    int new_w = (int)(w * scale + 0.5f);
    int new_h = (int)(h * scale + 0.5f);
    int off_x = (out_w - new_w) / 2;
    int off_y = (out_h - new_h) / 2;
    if (pad_x)     *pad_x     = off_x;
    if (pad_y)     *pad_y     = off_y;
    if (scale_out) *scale_out = scale;

    memset(rgb_out, 114, out_w * out_h * 3);
    uint8_t* tmp = (uint8_t*)malloc(new_w * new_h * 3);
    if (!tmp) { tjDestroy(tj); return -1; }

    gettimeofday(&t0, nullptr);
    int ret = tjDecompress2(tj, jpeg_data, jpeg_len,
                            tmp, new_w, 0, new_h, TJPF_RGB, TJFLAG_FASTDCT);
    tjDestroy(tj);
    if (t_decomp) *t_decomp = elapsed_us(t0);

    if (ret < 0) { free(tmp); return -1; }

    gettimeofday(&t0, nullptr);
    for (int y = 0; y < new_h; y++)
        memcpy(rgb_out + ((y + off_y) * out_w + off_x) * 3,
               tmp     + y * new_w * 3, new_w * 3);
    if (t_copy) *t_copy = elapsed_us(t0);

    free(tmp);
    return 0;
}

// ── Dynamic base speed ────────────────────────────────────────────────────────
static int base_speed(float area_ratio) {
    if (area_ratio >= AREA_BRAKE) return BRAKE_SPEED;  // 制动区: 锁最低速
    float t = (area_ratio - AREA_FAR) / (AREA_BRAKE - AREA_FAR);
    t = std::max(0.0f, std::min(1.0f, t));
    return (int)(CHASE_SPEED_FAR + t * (BRAKE_SPEED - CHASE_SPEED_FAR));
}

// ── Usage ─────────────────────────────────────────────────────────────────────
static void usage(const char* prog) {
    LOGI("Usage:");
    LOGI("  %s <model.rknn> [uart_dev] [uvc_device_index] [arm_dev] [platform]", prog);
    LOGI("  Example legacy: %s tennis.rknn /dev/ttyS3 0 /dev/ttyUSB1", prog);
    LOGI("  Example lekiwi: %s tennis.rknn /dev/ttyACM0 0 /dev/ttyACM0 lekiwi", prog);
    LOGI("  %s test-uvc   [uvc_index]               -- capture one frame -> capture.jpg", prog);
    LOGI("  %s test-yolo  <model.rknn> [uvc_index]  -- detect one frame  -> result.jpg", prog);
    LOGI("  %s test-motor [uart_dev] [speed=N]       -- motor test", prog);
    LOGI("  %s test-arm   [uart_dev] <cmd|a0 a1 a2>  -- arm servo test (default /dev/ttyUSB1)", prog);
    LOGI("  %s test-bucket [uvc_index]               -- red bucket detect -> bucket.jpg", prog);
    LOGI("  %s test-feetech [uart_dev] <scan|read|torque-off> -- STS3215 bus test", prog);
    LOGI("  %s test-base [uart_dev] <forward|backward|left|right|rotate-left|rotate-right|stop> [level]", prog);
    LOGI("  %s test-new-arm [uart_dev] <calibrate|calib-check|pos|grab|release|show|torque-off|set name deg|raw name value>", prog);
}

class LegacyDriveAdapter : public DriveAdapter {
public:
    explicit LegacyDriveAdapter(Motor& motor) : motor_(motor) {}
    void drive(int left_speed, int right_speed) override { motor_.drive(left_speed, right_speed); }
    void brake() override { motor_.brake(); }
    void standby() override { motor_.standby(); }
private:
    Motor& motor_;
};

class OmniDriveAdapter : public DriveAdapter {
public:
    explicit OmniDriveAdapter(OmniBase& base) : base_(base) {}
    void drive(int left_speed, int right_speed) override {
        float x = ((left_speed + right_speed) * 0.5f) / 100.0f * 0.12f;
        float turn = (right_speed - left_speed) / 100.0f * 35.0f;
        base_.drive_body(x, 0.0f, turn);
    }
    void brake() override { base_.stop(); }
    void standby() override { base_.stop(); }
private:
    OmniBase& base_;
};

class ArmAdapter {
public:
    virtual ~ArmAdapter() = default;
    virtual void grab_pos() = 0;
    virtual void grab() = 0;
    virtual void release() = 0;
};

class LegacyArmAdapter : public ArmAdapter {
public:
    explicit LegacyArmAdapter(Arm& arm) : arm_(arm) {}
    void grab_pos() override { arm_.grab_pos(); }
    void grab() override { arm_.grab(); }
    void release() override { arm_.release(); }
private:
    Arm& arm_;
};

class FeetechArmAdapter : public ArmAdapter {
public:
    explicit FeetechArmAdapter(FeetechArm& arm) : arm_(arm) {}
    void grab_pos() override { arm_.grab_pos(); }
    void grab() override { arm_.grab(); }
    void release() override { arm_.release(); }
private:
    FeetechArm& arm_;
};

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    if (argc < 2) { usage(argv[0]); return 1; }

    if (strcmp(argv[1], "test-uvc") == 0) {
        int idx = (argc >= 3) ? atoi(argv[2]) : 0;
        return cmd_test_uvc(idx);
    }
    if (strcmp(argv[1], "test-yolo") == 0) {
        if (argc < 3) { LOGE("test-yolo requires <model.rknn>"); usage(argv[0]); return 1; }
        int idx = (argc >= 4) ? atoi(argv[3]) : 0;
        return cmd_test_yolo(argv[2], idx);
    }
    if (strcmp(argv[1], "test-motor") == 0) {
        const char* dev = (argc >= 3) ? argv[2] : "/dev/ttyUSB0";
        return cmd_test_motor(dev, argc, argv);
    }
    if (strcmp(argv[1], "test-arm") == 0) {
        const char* dev = (argc >= 3) ? argv[2] : "/dev/ttyUSB1";
        return cmd_test_arm(dev, argc, argv);
    }
    if (strcmp(argv[1], "test-bucket") == 0) {
        int idx = (argc >= 3) ? atoi(argv[2]) : 0;
        return cmd_test_bucket(idx);
    }
    if (strcmp(argv[1], "test-feetech") == 0) {
        const char* dev = (argc >= 3) ? argv[2] : "/dev/ttyACM0";
        return cmd_test_feetech(dev, argc, argv);
    }
    if (strcmp(argv[1], "test-base") == 0) {
        const char* dev = (argc >= 3) ? argv[2] : "/dev/ttyACM0";
        return cmd_test_base(dev, argc, argv);
    }
    if (strcmp(argv[1], "test-new-arm") == 0) {
        const char* dev = (argc >= 3) ? argv[2] : "/dev/ttyACM0";
        return cmd_test_new_arm(dev, argc, argv);
    }

    const char* model_path = argv[1];
    const char* uart_dev   = (argc >= 3) ? argv[2] : "/dev/ttyS3";
    int         uvc_index  = (argc >= 4) ? atoi(argv[3]) : 0;
    const char* arm_dev    = (argc >= 5) ? argv[4] : "/dev/ttyUSB1";
    const char* platform   = (argc >= 6) ? argv[5] : "legacy";
    bool use_lekiwi = (strcmp(platform, "lekiwi") == 0 || strcmp(platform, "omni") == 0);
    const int stop_center_offset = use_lekiwi ? 0 : STOP_CENTER_OFFSET;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    Motor* motor_ptr = nullptr;
    Arm* legacy_arm_ptr = nullptr;
    feetech::FeetechBus* ft_bus_ptr = nullptr;
    OmniBase* omni_base_ptr = nullptr;
    FeetechArm* ft_arm_ptr = nullptr;
    DriveAdapter* drive_ptr = nullptr;
    ArmAdapter* arm_ptr = nullptr;

    if (use_lekiwi) {
        ft_bus_ptr = new feetech::FeetechBus(uart_dev, 1000000);
        if (!ft_bus_ptr->open()) {
            LOGE("Failed to open Feetech bus %s: %s", uart_dev, ft_bus_ptr->last_error().c_str());
            return 1;
        }
        omni_base_ptr = new OmniBase(*ft_bus_ptr);
        ft_arm_ptr = new FeetechArm(*ft_bus_ptr);
        if (!ft_arm_ptr->has_calibration()) {
            LOGE("Missing/invalid LeKiwi calibration: %s", ft_arm_ptr->calibration_error().c_str());
            return 1;
        }
        if (!omni_base_ptr->configure()) {
            LOGE("Failed to configure omni base: %s", ft_bus_ptr->last_error().c_str());
            return 1;
        }
        if (!ft_arm_ptr->configure()) {
            LOGE("Failed to configure Feetech arm: %s", ft_bus_ptr->last_error().c_str());
            return 1;
        }
        drive_ptr = new OmniDriveAdapter(*omni_base_ptr);
        arm_ptr = new FeetechArmAdapter(*ft_arm_ptr);
        g_drive = drive_ptr;
        arm_ptr->grab_pos();
        LOGI("LeKiwi platform initialized (Feetech %s)", uart_dev);
    } else {
        motor_ptr = new Motor(MotorDriverType::UART, uart_dev);
        g_motor = motor_ptr;
        motor_ptr->set_min_speed(15);
        drive_ptr = new LegacyDriveAdapter(*motor_ptr);
        g_drive = drive_ptr;
        LOGI("Motor initialized (UART %s)  min_speed=%d", uart_dev, motor_ptr->get_min_speed());

        legacy_arm_ptr = new Arm(arm_dev);
        g_arm = legacy_arm_ptr;
        arm_ptr = new LegacyArmAdapter(*legacy_arm_ptr);
        arm_ptr->grab_pos();
        LOGI("Arm initialized (%s)", arm_dev);
    }

    UvcCapture capture;
    g_capture = &capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index); return 1;
    }
    LOGI("Camera opened (%dx%d)", FRAME_WIDTH, FRAME_HEIGHT);

    rknn_app_context_t rknn_ctx;
    g_rknn_ctx = &rknn_ctx;
    if (detect_init(model_path, &rknn_ctx) != 0) {
        LOGE("Failed to load model: %s", model_path); return 1;
    }
    int model_w = rknn_ctx.model_width;
    int model_h = rknn_ctx.model_height;
    LOGI("Model input %dx%d", model_w, model_h);

    // Suppress rknn runtime stderr spam
    g_devnull      = open("/dev/null", O_WRONLY);
    g_saved_stderr = dup(STDERR_FILENO);
    dup2(g_devnull, STDERR_FILENO);

    const size_t MJPEG_BUF = 1024 * 1024;
    uint8_t* mjpeg_buf = (uint8_t*)malloc(MJPEG_BUF);
    uint8_t* rgb_buf   = (uint8_t*)malloc(model_w * model_h * 3);
    // separate full-res buffer for bucket detection (HSV on 640×480)
    uint8_t* bucket_rgb = (uint8_t*)malloc(FRAME_WIDTH * FRAME_HEIGHT * 3);
    if (!mjpeg_buf || !rgb_buf || !bucket_rgb) { LOGE("OOM"); return 1; }

    dup2(g_saved_stderr, STDERR_FILENO);
    LOGI("Warming up camera (skip 20 frames)...");
    dup2(g_devnull, STDERR_FILENO);
    for (int i = 0; i < 20; i++) capture.getFrame(mjpeg_buf, MJPEG_BUF, 500);

    int  frame_idx    = 0;
    int  proc_cnt     = 0;
    long t_decode_acc = 0, t_infer_acc = 0, t_ctrl_acc = 0;

    // ── Last-seen tracking for search-after-loss ──────────────────────────────
    int  last_offset      = 0;
    int  last_seen_frame  = -999;

    // ── Stop state ────────────────────────────────────────────────────────────
    bool stopped          = false;
    int  stop_confirm_cnt = 0;
    int  align_cnt        = 0;

    // ── ALIGN stall detection ─────────────────────────────────────────────────
    static const int STALL_BUF = 30;
    int  align_off_buf[STALL_BUF] = {};
    int  align_off_head = 0;
    bool align_kicking  = false;

    // ── Game state ────────────────────────────────────────────────────────────
    GameState game_state = GameState::CHASE_BALL;
    int  bucket_lost_cnt  = 0;   // 连续找不到桶的帧数
    int  bucket_confirm   = 0;   // 连续看到桶的帧数（防抖）
    LeKiwiMoveController lekiwi_move(FRAME_WIDTH, FRAME_HEIGHT);
    LeKiwiArmController* lekiwi_arm_ctrl = use_lekiwi ? new LeKiwiArmController(*ft_arm_ptr) : nullptr;
    int lekiwi_arm_log_tick = 0;
    LeKiwiPickConfig lekiwi_pick_base_config;
    lekiwi_pick_base_config.load();
    std::vector<std::pair<float, float>> lekiwi_pick_retry_offsets = {
        {0.0f, 0.0f},
        {-0.005f, 0.0f},
        {-0.010f, 0.0f},
        {-0.015f, 0.0f},
        {-0.020f, 0.0f},
        {0.010f, 0.0f},
        {0.0f, -0.010f},
        {0.0f, 0.010f},
        {-0.015f, -0.010f},
        {-0.015f, 0.010f},
    };
    size_t lekiwi_pick_retry_index = 0;
    LeKiwiPickConfig lekiwi_pick_attempt_config = lekiwi_pick_base_config;

    // ── Chase loop ────────────────────────────────────────────────────────────
    while (true) {
        struct timeval t_start, t_stage;
        gettimeofday(&t_start, nullptr);
        frame_idx++;

        int jpeg_len = capture.getFrame(mjpeg_buf, MJPEG_BUF, 200);
        if (jpeg_len <= 0) {
            dup2(g_saved_stderr, STDERR_FILENO);
            LOGW("[Frame %d] No frame (timeout)", frame_idx);
            dup2(g_devnull, STDERR_FILENO);
            usleep(10000);
            continue;
        }

#if SKIP_FRAMES > 0
        if ((frame_idx % (SKIP_FRAMES + 1)) != 0) continue;
#endif
        proc_cnt++;

        long th=0, td=0, tc=0;
        int  lb_x=0, lb_y=0;
        float lb_sc=1.0f;
        if (decode_mjpeg(mjpeg_buf, jpeg_len, rgb_buf, model_w, model_h,
                         &lb_x, &lb_y, &lb_sc, &th, &td, &tc) != 0) continue;
        t_decode_acc += th + td + tc;

        if (use_lekiwi && game_state == GameState::PICK_BALL) {
            drive_ptr->standby();
            if (lekiwi_arm_ctrl && !lekiwi_arm_ctrl->active() &&
                !lekiwi_arm_ctrl->done() && !lekiwi_arm_ctrl->failed()) {
                lekiwi_pick_attempt_config = lekiwi_pick_base_config;
                if (lekiwi_pick_retry_index < lekiwi_pick_retry_offsets.size()) {
                    float dx = lekiwi_pick_retry_offsets[lekiwi_pick_retry_index].first;
                    float dy = lekiwi_pick_retry_offsets[lekiwi_pick_retry_index].second;
                    lekiwi_pick_attempt_config.pre_grab_x += dx;
                    lekiwi_pick_attempt_config.grab_x += dx;
                    lekiwi_pick_attempt_config.pre_grab_y += dy;
                    lekiwi_pick_attempt_config.grab_y += dy;
                }
                lekiwi_arm_ctrl->begin_pick(lekiwi_pick_attempt_config);
                dup2(g_saved_stderr, STDERR_FILENO);
                float log_dx = 0.0f, log_dy = 0.0f;
                if (lekiwi_pick_retry_index < lekiwi_pick_retry_offsets.size()) {
                    log_dx = lekiwi_pick_retry_offsets[lekiwi_pick_retry_index].first;
                    log_dy = lekiwi_pick_retry_offsets[lekiwi_pick_retry_index].second;
                }
                printf("[GAME] PICK_BALL start IK catch sequence attempt=%zu/%zu offset=(%.4f, %.4f) grab=(%.4f, %.4f)\n",
                       lekiwi_pick_retry_index + 1,
                       lekiwi_pick_retry_offsets.size(),
                       log_dx,
                       log_dy,
                       lekiwi_pick_attempt_config.grab_x,
                       lekiwi_pick_attempt_config.grab_y);
                dup2(g_devnull, STDERR_FILENO);
            }

            if (!lekiwi_arm_ctrl || !lekiwi_arm_ctrl->tick() || lekiwi_arm_ctrl->failed()) {
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] PICK_BALL controller failed -> CHASE_BALL\n");
                dup2(g_devnull, STDERR_FILENO);
                if (lekiwi_arm_ctrl) lekiwi_arm_ctrl->reset();
                lekiwi_move.reset();
                game_state = GameState::CHASE_BALL;
                continue;
            }

            if (lekiwi_arm_ctrl && (++lekiwi_arm_log_tick % 10) == 0) {
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] PICK_BALL step=%zu/%zu %s\n",
                       lekiwi_arm_ctrl->step_index() + 1,
                       lekiwi_arm_ctrl->step_count(),
                       lekiwi_arm_ctrl->current_step_label());
                dup2(g_devnull, STDERR_FILENO);
            }

            if (lekiwi_arm_ctrl->done()) {
                float gripper_pos = 0.0f;
                bool holding = lekiwi_arm_ctrl->verify_grab(&gripper_pos);
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] PICK_BALL done gripper=%.1f holding=%s\n",
                       gripper_pos, holding ? "yes" : "no");
                dup2(g_devnull, STDERR_FILENO);
                lekiwi_arm_ctrl->reset();
                lekiwi_arm_log_tick = 0;
                lekiwi_move.reset();
                if (holding) {
                    if (lekiwi_pick_retry_index > 0) {
                        lekiwi_pick_attempt_config.save();
                        lekiwi_pick_base_config = lekiwi_pick_attempt_config;
                        dup2(g_saved_stderr, STDERR_FILENO);
                        printf("[GAME] saved successful pick config grab=(%.4f, %.4f)\n",
                               lekiwi_pick_base_config.grab_x,
                               lekiwi_pick_base_config.grab_y);
                        dup2(g_devnull, STDERR_FILENO);
                    }
                    lekiwi_pick_retry_index = 0;
                    game_state = GameState::FIND_BUCKET;
                    bucket_lost_cnt = 0;
                    bucket_confirm = 0;
                    printf("[GAME] -> FIND_BUCKET\n");
                } else {
                    lekiwi_pick_retry_index++;
                    if (lekiwi_pick_retry_index < lekiwi_pick_retry_offsets.size()) {
                        game_state = GameState::CHASE_BALL;
                        lekiwi_move.reset();
                        last_seen_frame = -999;
                        dup2(g_saved_stderr, STDERR_FILENO);
                        printf("[GAME] grab failed -> CHASE_BALL for visual realign, next pick attempt=%zu/%zu\n",
                               lekiwi_pick_retry_index + 1,
                               lekiwi_pick_retry_offsets.size());
                        dup2(g_devnull, STDERR_FILENO);
                    } else {
                        lekiwi_pick_retry_index = 0;
                        game_state = GameState::CHASE_BALL;
                        printf("[GAME] grab failed all attempts -> CHASE_BALL\n");
                    }
                }
            }
            continue;
        }

        if (use_lekiwi && game_state == GameState::PUT_BALL) {
            drive_ptr->standby();
            if (lekiwi_arm_ctrl && !lekiwi_arm_ctrl->active() &&
                !lekiwi_arm_ctrl->done() && !lekiwi_arm_ctrl->failed()) {
                lekiwi_arm_ctrl->begin_put();
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] PUT_BALL start IK put sequence\n");
                dup2(g_devnull, STDERR_FILENO);
            }

            if (!lekiwi_arm_ctrl || !lekiwi_arm_ctrl->tick() || lekiwi_arm_ctrl->failed()) {
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] PUT_BALL controller failed -> CHASE_BALL\n");
                dup2(g_devnull, STDERR_FILENO);
                if (lekiwi_arm_ctrl) lekiwi_arm_ctrl->reset();
                lekiwi_move.reset();
                game_state = GameState::CHASE_BALL;
                continue;
            }

            if (lekiwi_arm_ctrl && (++lekiwi_arm_log_tick % 10) == 0) {
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] PUT_BALL step=%zu/%zu %s\n",
                       lekiwi_arm_ctrl->step_index() + 1,
                       lekiwi_arm_ctrl->step_count(),
                       lekiwi_arm_ctrl->current_step_label());
                dup2(g_devnull, STDERR_FILENO);
            }

            if (lekiwi_arm_ctrl->done()) {
                if (lekiwi_arm_ctrl) lekiwi_arm_ctrl->reset();
                lekiwi_arm_log_tick = 0;
                lekiwi_move.reset();
                stopped = false;
                stop_confirm_cnt = 0;
                align_cnt = 0;
                align_off_head = 0;
                last_seen_frame = -999;
                bucket_lost_cnt = 0;
                bucket_confirm = 0;
                game_state = GameState::CHASE_BALL;
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] PUT_BALL done -> CHASE_BALL\n");
                dup2(g_devnull, STDERR_FILENO);
            }
            continue;
        }

        // ── 桶的状态机（FIND_BUCKET / APPROACH_BUCKET / DEPOSIT / DONE）────────
        // 这几个状态不需要 YOLO，直接用 HSV 识别桶后跳过后续逻辑
        if (game_state == GameState::FIND_BUCKET ||
            game_state == GameState::APPROACH_BUCKET ||
            game_state == GameState::DEPOSIT)
        {
            // 用全分辨率 bucket_rgb 做 HSV（decode_mjpeg 已解码到 model 尺寸的 rgb_buf，
            // 这里重新以 FRAME 尺寸解码一次供桶检测用）
            decode_mjpeg(mjpeg_buf, jpeg_len, bucket_rgb,
                         FRAME_WIDTH, FRAME_HEIGHT, nullptr, nullptr, nullptr);

            if (game_state == GameState::DEPOSIT) {
                drive_ptr->standby();
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] DEPOSIT – releasing ball...\n");
                dup2(g_devnull, STDERR_FILENO);
                if (arm_ptr) arm_ptr->release();
                usleep(500000);
                if (arm_ptr) arm_ptr->grab_pos();
                // ── 重置，继续找下一个球 ──────────────────────────────────
                game_state       = GameState::CHASE_BALL;
                stopped          = false;
                stop_confirm_cnt = 0;
                align_cnt        = 0;
                align_off_head   = 0;
                last_seen_frame  = -999;
                bucket_lost_cnt  = 0;
                bucket_confirm   = 0;
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] -> CHASE_BALL (next round)\n");
                dup2(g_devnull, STDERR_FILENO);
                continue;
            }

            // FIND_BUCKET / APPROACH_BUCKET: run HSV detection
            BucketResult br{};
            bool bucket_visible = detect_bucket_frame(bucket_rgb,
                                                      FRAME_WIDTH, FRAME_HEIGHT, br);

            if (use_lekiwi) {
                auto cmd = lekiwi_move.update_bucket(bucket_visible, br.cx, br.w, br.h);
                if (cmd.idle) drive_ptr->standby();
                else drive_ptr->drive(cmd.left_speed, cmd.right_speed);

                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] LEKIWI_BUCKET visible=%d label=%s cx=%d size=%d/%d L=%d R=%d stable=%d\n",
                       bucket_visible ? 1 : 0, cmd.label, br.cx, br.w, br.h,
                       cmd.left_speed, cmd.right_speed, cmd.reached ? 1 : 0);
                dup2(g_devnull, STDERR_FILENO);

                if (cmd.reached) {
                    drive_ptr->standby();
                    game_state = GameState::PUT_BALL;
                    if (lekiwi_arm_ctrl) lekiwi_arm_ctrl->reset();
                    lekiwi_move.reset();
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[GAME] -> PUT_BALL\n");
                    dup2(g_devnull, STDERR_FILENO);
                }
                continue;
            }

            if (game_state == GameState::FIND_BUCKET) {
                if (bucket_visible) {
                    bucket_confirm++;
                    bucket_lost_cnt = 0;
                    if (bucket_confirm >= BUCKET_CONFIRM_CNT) {
                        game_state = GameState::APPROACH_BUCKET;
                        bucket_confirm = 0;
                        dup2(g_saved_stderr, STDERR_FILENO);
                        printf("[GAME] -> APPROACH_BUCKET  area=%.3f\n", br.area_ratio);
                        dup2(g_devnull, STDERR_FILENO);
                    } else {
                        drive_ptr->standby(); // 稳住等确认
                    }
                } else {
                    bucket_confirm = 0;
                    bucket_lost_cnt++;
                    // 旋转搜索（始终向右转，可根据场地调整）
                    drive_ptr->drive(BUCKET_SEARCH_SPD, -BUCKET_SEARCH_SPD);
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[GAME] FIND_BUCKET searching... lost=%d\n", bucket_lost_cnt);
                    dup2(g_devnull, STDERR_FILENO);
                }
                continue;
            }

            // APPROACH_BUCKET
            if (!bucket_visible) {
                bucket_lost_cnt++;
                if (bucket_lost_cnt > BUCKET_LOST_FRAMES) {
                    game_state = GameState::FIND_BUCKET;
                    bucket_confirm = 0;
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[GAME] APPROACH_BUCKET lost bucket -> FIND_BUCKET\n");
                    dup2(g_devnull, STDERR_FILENO);
                } else {
                    drive_ptr->standby(); // 短暂丢失时停车等
                }
                continue;
            }
            bucket_lost_cnt = 0;

            if (br.area_ratio >= BUCKET_AREA_DEPOSIT) {
                // 足够近 → 制动停车，进入放球
                struct timeval tb2; gettimeofday(&tb2, nullptr);
                while (elapsed_us(tb2) < BRAKE_PULSE_US) {
                    drive_ptr->brake(); usleep(20000);
                }
                drive_ptr->standby();
                game_state = GameState::DEPOSIT;
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[GAME] -> DEPOSIT  bucket area=%.3f\n", br.area_ratio);
                dup2(g_devnull, STDERR_FILENO);
                continue;
            }

            // 趋近：差速对准桶中心
            int half_w2  = FRAME_WIDTH / 2;
            int bk_off   = br.cx - half_w2;
            int bk_bias  = (abs(bk_off) <= CENTER_DEAD_ZONE) ? 0
                         : (int)(BUCKET_K_TURN * bk_off / (float)half_w2);
            bk_bias = std::max(-BUCKET_MAX_BIAS, std::min(BUCKET_MAX_BIAS, bk_bias));

            int bk_spd = (br.area_ratio >= BUCKET_AREA_BRAKE)
                         ? BUCKET_BRAKE_SPD : BUCKET_APPROACH_SPD;
            int bk_l = std::max(-100, std::min(100, bk_spd + bk_bias));
            int bk_r = std::max(-100, std::min(100, bk_spd - bk_bias));
            drive_ptr->drive(bk_l, bk_r);

            dup2(g_saved_stderr, STDERR_FILENO);
            printf("[GAME] APPROACH_BUCKET  area=%.3f off=%d  L=%d R=%d\n",
                   br.area_ratio, bk_off, bk_l, bk_r);
            dup2(g_devnull, STDERR_FILENO);
            continue;
        }

        // ── 以下为 CHASE_BALL 逻辑（YOLO based）─────────────────────────────
        long ti=0, tr=0, to=0, tp=0;
        std::vector<detection> dets;
        detect_run(&rknn_ctx, rgb_buf, model_w, model_h,
                   FRAME_WIDTH, FRAME_HEIGHT, lb_x, lb_y, lb_sc,
                   0.5f, 0.45f, dets, &ti, &tr, &to, &tp);
        t_infer_acc += ti + tr + to + tp;

        // ── Smooth differential steering ──────────────────────────────────────
        gettimeofday(&t_stage, nullptr);
        const int half_w = FRAME_WIDTH / 2;

        if (!dets.empty()) {
            int best = 0;
            for (int i = 1; i < (int)dets.size(); i++)
                if (dets[i].bbox.w * dets[i].bbox.h >
                    dets[best].bbox.w * dets[best].bbox.h)
                    best = i;

            const box& b     = dets[best].bbox;
            float area_ratio = (b.w * b.h) / (float)(FRAME_WIDTH * FRAME_HEIGHT);
            int   ball_cx    = (int)b.x;
            int   offset     = ball_cx - half_w;   // <0 = ball on left

            // Update last-seen tracking
            last_offset     = offset;
            last_seen_frame = frame_idx;

            int spd  = base_speed(area_ratio);

            if (use_lekiwi) {
                auto cmd = lekiwi_move.update_ball(dets);
                if (cmd.idle) drive_ptr->standby();
                else drive_ptr->drive(cmd.left_speed, cmd.right_speed);

                dup2(g_saved_stderr, STDERR_FILENO);
                long frame_us = elapsed_us(t_start);
                printf("[STATE] LEKIWI_CHASE label=%s area=%.3f off=%3d size=%3d L=%3d R=%3d ready=%d fps=%.1f\n",
                       cmd.label, area_ratio, offset, (int)std::max(b.w, b.h),
                       cmd.left_speed, cmd.right_speed, cmd.reached ? 1 : 0,
                       1e6f / frame_us);
                dup2(g_devnull, STDERR_FILENO);

                if (cmd.reached) {
                    drive_ptr->standby();
                    game_state = GameState::PICK_BALL;
                    if (lekiwi_arm_ctrl) lekiwi_arm_ctrl->reset();
                    lekiwi_move.reset();
                    lekiwi_pick_base_config.load();
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[GAME] -> PICK_BALL attempt=%zu/%zu\n",
                           lekiwi_pick_retry_index + 1,
                           lekiwi_pick_retry_offsets.size());
                    dup2(g_devnull, STDERR_FILENO);
                }
                t_ctrl_acc += elapsed_us(t_stage);
                continue;
            }

            // ── 区域标签（用于日志）───────────────────────────────────────────
            const char* zone = (area_ratio >= AREA_REVERSE) ? "REVERSE" :
                               (area_ratio >= AREA_STOP)    ? "STOP"    :
                               (area_ratio >= AREA_BRAKE)   ? "BRAKE"   :
                               (area_ratio >= AREA_NEAR)    ? "NEAR"    :
                               (area_ratio >= AREA_FAR)     ? "FAR"     : "LOST";

            // ── Stop state: exit only when ball moves far away ────────────────
            if (stopped) {
                if (area_ratio >= AREA_REVERSE) {
                    // 即使已停止，球太近也要后退
                    stopped = false;
                    stop_confirm_cnt = 0;
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[STATE] STOPPED->REVERSE  area=%.3f\n", area_ratio);
                    dup2(g_devnull, STDERR_FILENO);
                    // fall through to REVERSE logic below
                } else if (area_ratio < AREA_STOP_EXIT) {
                    stopped = false;
                    stop_confirm_cnt = 0;
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[STATE] RESUME  area=%.3f zone=%s\n", area_ratio, zone);
                    dup2(g_devnull, STDERR_FILENO);
                } else {
                    drive_ptr->standby();
                    t_ctrl_acc += elapsed_us(t_stage);
                    continue;
                }
            }

            // ── 后退: 球占画面过大 ────────────────────────────────────────────
            if (area_ratio >= AREA_REVERSE) {
                int rev_left  = (offset > CENTER_DEAD_ZONE)  ? -REVERSE_SPEED + 5 :
                                (offset < -CENTER_DEAD_ZONE) ? -REVERSE_SPEED - 5 :
                                -REVERSE_SPEED;
                int rev_right = (offset > CENTER_DEAD_ZONE)  ? -REVERSE_SPEED - 5 :
                                (offset < -CENTER_DEAD_ZONE) ? -REVERSE_SPEED + 5 :
                                -REVERSE_SPEED;
                drive_ptr->drive(rev_left, rev_right);
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[STATE] REVERSE  area=%.3f off=%d  L=%d R=%d\n",
                       area_ratio, offset, rev_left, rev_right);
                dup2(g_devnull, STDERR_FILENO);
                t_ctrl_acc += elapsed_us(t_stage);
                continue;
            }

            // ── Stop condition: close enough AND at target offset, confirm N frames ───
            int stop_off = offset - stop_center_offset;
            if (area_ratio >= AREA_STOP && abs(stop_off) <= STOP_CENTER_ZONE) {
                stop_confirm_cnt++;
                align_cnt = 0;
                align_off_head = 0;
                if (stop_confirm_cnt >= STOP_CONFIRM_CNT) {
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[STATE] BRAKING  area=%.3f off=%d  %dms...\n",
                           area_ratio, offset, BRAKE_PULSE_US/1000);
                    dup2(g_devnull, STDERR_FILENO);
                    struct timeval tb; gettimeofday(&tb, nullptr);
                    while (elapsed_us(tb) < BRAKE_PULSE_US) {
                        drive_ptr->brake();
                        usleep(20000);
                    }
                    drive_ptr->standby();
                    stopped = true;
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[STATE] STOPPED  area=%.3f  -> GRAB\n", area_ratio);
                    dup2(g_devnull, STDERR_FILENO);
                    // ── 执行抓球 ──────────────────────────────────────────────
                    if (arm_ptr) arm_ptr->grab();
                    // ── 切换到找桶状态 ────────────────────────────────────────
                    game_state    = GameState::FIND_BUCKET;
                    bucket_lost_cnt = 0;
                    bucket_confirm  = 0;
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[GAME] -> FIND_BUCKET\n");
                    dup2(g_devnull, STDERR_FILENO);
                } else {
                    drive_ptr->brake();
                }
                t_ctrl_acc += elapsed_us(t_stage);
                continue;
            } else if (area_ratio >= AREA_STOP && abs(stop_off) > STOP_CENTER_ZONE) {
                // Close but not at target offset → proportional pivot to align
                stop_confirm_cnt = 0;
                align_cnt++;

                // ── 卡死检测：记录 offset 历史，若近N帧无移动则踢出 ──────────
                align_off_buf[align_off_head % STALL_BUF] = offset;
                align_off_head++;
                bool stalled = false;
                if (align_cnt >= ALIGN_STALL_FRAMES) {
                    int mn = align_off_buf[0], mx = align_off_buf[0];
                    int check = std::min(align_cnt, STALL_BUF);
                    for (int i = 1; i < check; i++) {
                        int v = align_off_buf[i];
                        if (v < mn) mn = v;
                        if (v > mx) mx = v;
                    }
                    stalled = (mx - mn) < ALIGN_STALL_MOVE_PX;
                }

                if (stalled) {
                    int kick = (stop_off > 0) ? ALIGN_KICK_SPD : -ALIGN_KICK_SPD;
                    dup2(g_saved_stderr, STDERR_FILENO);
                    printf("[STATE] ALIGN_KICK  area=%.3f off=%3d stop_off=%3d  kick=%d  %dms\n",
                           area_ratio, offset, stop_off, kick, ALIGN_KICK_US/1000);
                    dup2(g_devnull, STDERR_FILENO);
                    struct timeval tk; gettimeofday(&tk, nullptr);
                    while (elapsed_us(tk) < ALIGN_KICK_US) {
                        drive_ptr->drive(kick, -kick);
                        usleep(20000);
                    }
                    drive_ptr->brake();
                    // 重置历史，避免连续踢
                    align_off_head = 0;
                    align_cnt = 0;
                    t_ctrl_acc += elapsed_us(t_stage);
                    continue;
                }

                float t = std::min(1.0f, (float)abs(stop_off) / (float)half_w);
                int pivot_spd = (int)(ALIGN_PIVOT_MIN + t * (ALIGN_PIVOT_SPD - ALIGN_PIVOT_MIN));
                int pivot = (stop_off > 0) ? pivot_spd : -pivot_spd;
                drive_ptr->drive(pivot, -pivot);
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[STATE] ALIGN  area=%.3f off=%3d stop_off=%3d  pivot=%d  [%d]\n",
                       area_ratio, offset, stop_off, pivot, align_cnt);
                dup2(g_devnull, STDERR_FILENO);
                t_ctrl_acc += elapsed_us(t_stage);
                continue;
            } else {
                stop_confirm_cnt = 0;
                align_cnt = 0;
                align_off_head = 0;
            }

            // ── Chase ─────────────────────────────────────────────────────────
            int bias = (abs(offset) <= CENTER_DEAD_ZONE)
                       ? 0
                       : (int)(K_TURN * offset / (float)half_w);

            int left_spd, right_spd;
            if (area_ratio >= 0.35) {
                // 近距离：纯轴转，双轮反向，避免差速导致单轮反转丢球
                int max_bias = MAX_TURN_BIAS_NEAR;
                int pivot = std::max(-max_bias, std::min(max_bias, bias));
                left_spd  = pivot;
                right_spd = -pivot;
            } else {
                // 远距离：差速，双轮同向，保持前进同时修正方向
                int max_bias = MAX_TURN_BIAS_FAR;
                bias = std::max(-max_bias, std::min(max_bias, bias));
                left_spd  = std::max(-100, std::min(100, spd + bias));
                right_spd = std::max(-100, std::min(100, spd - bias));
            }

            drive_ptr->drive(left_spd, right_spd);

            dup2(g_saved_stderr, STDERR_FILENO);
            long frame_us = elapsed_us(t_start);
            const char* steer = (area_ratio >= AREA_BRAKE) ? "pivot" : "diff";
            printf("[STATE] CHASE zone=%-6s area=%.3f off=%3d steer=%-5s  L=%3d R=%3d  fps=%.1f\n",
                   zone, area_ratio, offset, steer, left_spd, right_spd, 1e6f / frame_us);
            dup2(g_devnull, STDERR_FILENO);

        } else {
            if (use_lekiwi) {
                auto cmd = lekiwi_move.update_ball(dets);
                if (cmd.idle) drive_ptr->standby();
                else drive_ptr->drive(cmd.left_speed, cmd.right_speed);
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[STATE] LEKIWI_SEARCH label=%s L=%d R=%d\n",
                       cmd.label, cmd.left_speed, cmd.right_speed);
                dup2(g_devnull, STDERR_FILENO);
                t_ctrl_acc += elapsed_us(t_stage);
                continue;
            }
            int frames_lost = frame_idx - last_seen_frame;
            if (last_seen_frame >= 0 && frames_lost <= SEARCH_FRAMES) {
                // 刚丢失：沿最后看到球的方向快速转
                int pivot = (last_offset >= 0) ? SEARCH_PIVOT_SPD : -SEARCH_PIVOT_SPD;
                drive_ptr->drive(pivot, -pivot);
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[STATE] SEARCH lost=%d/%d  pivot=%s\n",
                       frames_lost, SEARCH_FRAMES, last_offset >= 0 ? "R" : "L");
                dup2(g_devnull, STDERR_FILENO);
            } else {
                // 长时间丢失：原地慢速旋转扫描，方向每 60 帧反转一次
                int scan_dir = ((frame_idx / 60) % 2 == 0) ? 1 : -1;
                drive_ptr->drive(scan_dir * SEARCH_PIVOT_SPD, -scan_dir * SEARCH_PIVOT_SPD);
                dup2(g_saved_stderr, STDERR_FILENO);
                printf("[STATE] SCAN  frame=%d  dir=%s\n",
                       frame_idx, scan_dir > 0 ? "R" : "L");
                dup2(g_devnull, STDERR_FILENO);
            }
        }

        t_ctrl_acc += elapsed_us(t_stage);
    }

    free(mjpeg_buf);
    free(rgb_buf);
    free(bucket_rgb);
    cleanup_and_exit();
    return 0;
}
