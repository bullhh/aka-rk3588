#include "feetech_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace feetech {

FeetechBus::FeetechBus(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {}

FeetechBus::~FeetechBus() {
    close();
}

bool FeetechBus::open() {
    if (fd_ >= 0) return true;
    if (!open_port()) return false;
    return configure_port();
}

void FeetechBus::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool FeetechBus::open_port() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        set_error("open " + port_ + " failed: " + std::strerror(errno));
        return false;
    }
    return true;
}

static speed_t baud_to_termios(int baudrate) {
    switch (baudrate) {
        case 115200:  return B115200;
        case 1000000: return B1000000;
        default:      return B1000000;
    }
}

bool FeetechBus::configure_port() {
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
        set_error("tcgetattr failed: " + std::string(std::strerror(errno)));
        close();
        return false;
    }

    speed_t baud = baud_to_termios(baudrate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        set_error("tcsetattr failed: " + std::string(std::strerror(errno)));
        close();
        return false;
    }
    return true;
}

uint8_t FeetechBus::checksum(uint8_t id, uint8_t length, uint8_t instruction,
                             const std::vector<uint8_t>& params) {
    uint16_t sum = id + length + instruction;
    for (uint8_t p : params) sum += p;
    return (uint8_t)(~sum & 0xff);
}

void FeetechBus::set_error(const std::string& msg) {
    last_error_ = msg;
    if (debug_) fprintf(stderr, "[FeetechBus] %s\n", msg.c_str());
}

bool FeetechBus::tx_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params) {
    if (fd_ < 0 && !open()) return false;

    uint8_t length = (uint8_t)(params.size() + 2);
    std::vector<uint8_t> pkt;
    pkt.reserve(params.size() + 6);
    pkt.push_back(0xff);
    pkt.push_back(0xff);
    pkt.push_back(id);
    pkt.push_back(length);
    pkt.push_back(instruction);
    pkt.insert(pkt.end(), params.begin(), params.end());
    pkt.push_back(checksum(id, length, instruction, params));

    ssize_t n = ::write(fd_, pkt.data(), pkt.size());
    if (n != (ssize_t)pkt.size()) {
        set_error("serial write failed: " + std::string(std::strerror(errno)));
        return false;
    }
    tcdrain(fd_);
    return true;
}

bool FeetechBus::rx_status(uint8_t expected_id, std::vector<uint8_t>& params, uint8_t* error_out,
                           int timeout_ms) {
    params.clear();
    if (error_out) *error_out = 0xff;

    std::vector<uint8_t> buf;
    buf.reserve(64);

    auto wait_byte = [&](uint8_t& b, int ms) -> bool {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);
        struct timeval tv;
        tv.tv_sec = ms / 1000;
        tv.tv_usec = (ms % 1000) * 1000;
        int rc = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (rc <= 0) return false;
        return ::read(fd_, &b, 1) == 1;
    };

    uint8_t b = 0;
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        if (!wait_byte(b, 5)) { elapsed += 5; continue; }
        buf.push_back(b);
        size_t n = buf.size();
        if (n >= 2 && buf[n - 2] == 0xff && buf[n - 1] == 0xff) {
            buf.clear();
            buf.push_back(0xff);
            buf.push_back(0xff);
            break;
        }
    }
    if (buf.size() < 2) {
        set_error("rx timeout waiting for header");
        return false;
    }

    uint8_t id = 0, length = 0, error = 0;
    if (!wait_byte(id, 20) || !wait_byte(length, 20) || !wait_byte(error, 20)) {
        set_error("rx timeout reading status header");
        return false;
    }
    if (expected_id != BROADCAST_ID && id != expected_id) {
        set_error("unexpected status id");
        return false;
    }
    if (length < 2) {
        set_error("invalid status length");
        return false;
    }

    int param_len = length - 2;
    params.resize(param_len);
    for (int i = 0; i < param_len; i++) {
        if (!wait_byte(params[i], 20)) {
            set_error("rx timeout reading params");
            return false;
        }
    }
    uint8_t chk = 0;
    if (!wait_byte(chk, 20)) {
        set_error("rx timeout reading checksum");
        return false;
    }

    std::vector<uint8_t> chk_params;
    chk_params.push_back(error);
    chk_params.insert(chk_params.end(), params.begin(), params.end());
    uint8_t expected_chk = checksum(id, length, 0, chk_params);
    // checksum() includes an instruction byte; status packets have error at that position.
    uint16_t sum = id + length + error;
    for (uint8_t p : params) sum += p;
    expected_chk = (uint8_t)(~sum & 0xff);
    if (chk != expected_chk) {
        set_error("status checksum mismatch");
        return false;
    }
    if (error_out) *error_out = error;
    return true;
}

bool FeetechBus::tx_rx(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params,
                       std::vector<uint8_t>& reply, uint8_t* error_out) {
    tcflush(fd_, TCIFLUSH);
    if (!tx_packet(id, instruction, params)) return false;
    if (id == BROADCAST_ID) return true;
    uint8_t err = 0;
    bool ok = rx_status(id, reply, &err, 120);
    if (error_out) *error_out = err;
    if (!ok) return false;
    if (err != 0) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "motor id=%u returned error status 0x%02x for instruction 0x%02x",
                 id, err, instruction);
        set_error(msg);
        return false;
    }
    return true;
}

bool FeetechBus::ping(int id) {
    std::vector<uint8_t> reply;
    uint8_t err = 0;
    return tx_rx((uint8_t)id, INST_PING, {}, reply, &err);
}

std::vector<int> FeetechBus::scan(int first_id, int last_id) {
    std::vector<int> found;
    for (int id = first_id; id <= last_id; id++) {
        if (ping(id)) found.push_back(id);
        usleep(10000);
    }
    return found;
}

bool FeetechBus::write_reg(int id, uint8_t addr, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> params;
    params.reserve(data.size() + 1);
    params.push_back(addr);
    params.insert(params.end(), data.begin(), data.end());
    std::vector<uint8_t> reply;
    return tx_rx((uint8_t)id, INST_WRITE, params, reply);
}

bool FeetechBus::read_reg(int id, uint8_t addr, uint8_t len, std::vector<uint8_t>& data) {
    std::vector<uint8_t> reply;
    std::vector<uint8_t> params = {addr, len};
    if (!tx_rx((uint8_t)id, INST_READ, params, reply)) return false;
    if (reply.size() < len) {
        set_error("short read response");
        return false;
    }
    data.assign(reply.begin(), reply.begin() + len);
    return true;
}

uint16_t FeetechBus::encode_sign_magnitude(int value, int sign_bit) {
    int max_mag = (1 << sign_bit) - 1;
    int mag = std::abs(value);
    if (mag > max_mag) mag = max_mag;
    return (uint16_t)(((value < 0) ? 1 : 0) << sign_bit) | (uint16_t)mag;
}

int FeetechBus::decode_sign_magnitude(uint16_t value, int sign_bit) {
    int mag = value & ((1 << sign_bit) - 1);
    return ((value >> sign_bit) & 1) ? -mag : mag;
}

bool FeetechBus::write_u8(int id, uint8_t addr, uint8_t value) {
    return write_reg(id, addr, {value});
}

bool FeetechBus::write_u16(int id, uint8_t addr, int value, bool sign_magnitude) {
    uint16_t encoded = sign_magnitude ? encode_sign_magnitude(value) : (uint16_t)value;
    return write_reg(id, addr, {lo(encoded), hi(encoded)});
}

bool FeetechBus::read_u8(int id, uint8_t addr, uint8_t& value) {
    std::vector<uint8_t> data;
    if (!read_reg(id, addr, 1, data)) return false;
    value = data[0];
    return true;
}

bool FeetechBus::read_u16(int id, uint8_t addr, int& value, bool sign_magnitude) {
    std::vector<uint8_t> data;
    if (!read_reg(id, addr, 2, data)) return false;
    uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    value = sign_magnitude ? decode_sign_magnitude(raw) : (int)raw;
    return true;
}

bool FeetechBus::sync_write_u16(uint8_t addr, const std::vector<std::pair<int, int>>& id_values,
                                bool sign_magnitude) {
    std::vector<uint8_t> params;
    params.push_back(addr);
    params.push_back(2);
    for (auto& item : id_values) {
        uint16_t encoded = sign_magnitude ? encode_sign_magnitude(item.second) : (uint16_t)item.second;
        params.push_back((uint8_t)item.first);
        params.push_back(lo(encoded));
        params.push_back(hi(encoded));
    }
    std::vector<uint8_t> reply;
    return tx_rx(BROADCAST_ID, INST_SYNC_WRITE, params, reply);
}

bool FeetechBus::set_operating_mode(int id, OperatingMode mode) {
    return write_u8(id, reg::OPERATING_MODE, (uint8_t)mode);
}

bool FeetechBus::enable_torque(int id, bool enable) {
    bool ok = write_u8(id, reg::TORQUE_ENABLE, enable ? 1 : 0);
    if (!enable && ok) {
        // Some STS servos reject LOCK writes depending on firmware/alarm state.
        // Torque control is the required operation here; LOCK is only a best-effort unlock
        // for optional parameter writes.
        write_u8(id, reg::LOCK, 0);
    }
    return ok;
}

bool FeetechBus::set_acceleration(int id, uint8_t acceleration) {
    return write_u8(id, reg::ACCELERATION, acceleration);
}

bool FeetechBus::set_goal_position(int id, int position) {
    return write_u16(id, reg::GOAL_POSITION, position, true);
}

bool FeetechBus::set_goal_velocity(int id, int velocity) {
    return write_u16(id, reg::GOAL_VELOCITY, velocity, true);
}

bool FeetechBus::read_status(int id, MotorStatus& status) {
    status.id = id;
    bool ok = true;
    ok = read_u16(id, reg::PRESENT_POSITION, status.position, true) && ok;
    ok = read_u16(id, reg::PRESENT_VELOCITY, status.velocity, true) && ok;
    uint8_t v = 0, t = 0;
    ok = read_u8(id, reg::PRESENT_VOLTAGE, v) && ok;
    ok = read_u8(id, reg::PRESENT_TEMPERATURE, t) && ok;
    status.voltage = v;
    status.temperature = t;
    return ok;
}

} // namespace feetech
