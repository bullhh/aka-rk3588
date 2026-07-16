#ifndef FEETECH_BUS_HPP
#define FEETECH_BUS_HPP

#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

#include <libusb-1.0/libusb.h>

namespace feetech {

enum class OperatingMode : uint8_t {
    POSITION = 0,
    VELOCITY = 1,
    PWM      = 2,
    STEP     = 3,
};

struct MotorStatus {
    int id = 0;
    int position = 0;
    int velocity = 0;
    int voltage = 0;
    int temperature = 0;
};

class FeetechBus {
public:
    explicit FeetechBus(const std::string& port = "/dev/ttyACM0", int baudrate = 1000000);
    ~FeetechBus();

    bool is_open() const;
    bool open();
    void close();

    bool ping(int id);
    std::vector<int> scan(int first_id = 1, int last_id = 9);

    bool write_u8(int id, uint8_t addr, uint8_t value);
    bool write_u16(int id, uint8_t addr, int value, bool sign_magnitude = false);
    bool read_u8(int id, uint8_t addr, uint8_t& value);
    bool read_u16(int id, uint8_t addr, int& value, bool sign_magnitude = false);

    bool sync_write_u16(uint8_t addr, const std::vector<std::pair<int, int>>& id_values,
                        bool sign_magnitude = false);

    bool set_operating_mode(int id, OperatingMode mode);
    bool enable_torque(int id, bool enable);
    bool set_acceleration(int id, uint8_t acceleration);
    bool set_goal_position(int id, int position);
    bool set_goal_velocity(int id, int velocity);
    bool read_status(int id, MotorStatus& status);

    void set_debug(bool debug) { debug_ = debug; }
    const std::string& last_error() const { return last_error_; }

private:
    enum : uint8_t {
        INST_PING       = 0x01,
        INST_READ       = 0x02,
        INST_WRITE      = 0x03,
        INST_SYNC_WRITE = 0x83,
    };

    static constexpr uint8_t BROADCAST_ID = 0xFE;

    bool open_port();
    bool configure_port();
    bool open_usb_cdc();
    void close_usb();
    bool configure_usb_cdc();
    bool write_bytes(const uint8_t* data, size_t len);
    bool read_byte(uint8_t& byte, int timeout_ms);
    bool read_usb_packet(int timeout_ms);
    void flush_input();
    bool tx_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params);
    bool rx_status(uint8_t expected_id, std::vector<uint8_t>& params, uint8_t* error_out,
                   int timeout_ms = 100);
    bool tx_rx(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params,
               std::vector<uint8_t>& reply, uint8_t* error_out = nullptr);
    bool write_reg(int id, uint8_t addr, const std::vector<uint8_t>& data);
    bool read_reg(int id, uint8_t addr, uint8_t len, std::vector<uint8_t>& data);

    static uint8_t checksum(uint8_t id, uint8_t length, uint8_t instruction,
                            const std::vector<uint8_t>& params);
    static uint16_t encode_sign_magnitude(int value, int sign_bit = 15);
    static int decode_sign_magnitude(uint16_t value, int sign_bit = 15);
    static uint8_t lo(uint16_t value) { return (uint8_t)(value & 0xff); }
    static uint8_t hi(uint16_t value) { return (uint8_t)((value >> 8) & 0xff); }

    void set_error(const std::string& msg);
    void log_debug(const char* fmt, ...) const;

    enum class Backend {
        NONE,
        TTY,
        USB_CDC,
    };

    std::string port_;
    int baudrate_;
    int fd_ = -1;
    Backend backend_ = Backend::NONE;
    libusb_context* usb_ctx_ = nullptr;
    libusb_device_handle* usb_handle_ = nullptr;
    int usb_control_iface_ = -1;
    int usb_data_iface_ = -1;
    bool usb_control_claimed_ = false;
    bool usb_data_claimed_ = false;
    uint8_t usb_ep_in_ = 0;
    uint8_t usb_ep_out_ = 0;
    std::vector<uint8_t> usb_rx_buffer_;
    size_t usb_rx_offset_ = 0;
    bool debug_ = false;
    std::string last_error_;
};

namespace reg {
static constexpr uint8_t ID                  = 5;
static constexpr uint8_t RETURN_DELAY_TIME   = 7;
static constexpr uint8_t RESPONSE_LEVEL      = 8;
static constexpr uint8_t PHASE               = 18;
static constexpr uint8_t P_COEFFICIENT       = 21;
static constexpr uint8_t D_COEFFICIENT       = 22;
static constexpr uint8_t I_COEFFICIENT       = 23;
static constexpr uint8_t OPERATING_MODE      = 33;
static constexpr uint8_t MAX_ACCELERATION    = 85;
static constexpr uint8_t TORQUE_ENABLE       = 40;
static constexpr uint8_t ACCELERATION        = 41;
static constexpr uint8_t GOAL_POSITION       = 42;
static constexpr uint8_t GOAL_TIME           = 44;
static constexpr uint8_t GOAL_VELOCITY       = 46;
static constexpr uint8_t LOCK                = 55;
static constexpr uint8_t PRESENT_POSITION    = 56;
static constexpr uint8_t PRESENT_VELOCITY    = 58;
static constexpr uint8_t PRESENT_VOLTAGE     = 62;
static constexpr uint8_t PRESENT_TEMPERATURE = 63;
} // namespace reg

} // namespace feetech

#endif // FEETECH_BUS_HPP
