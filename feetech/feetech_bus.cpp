#include "feetech_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <cstdarg>
#include <string>
#include <stdio.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

namespace feetech {

static int parse_env_int(const char* name, int default_value) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') return default_value;
    char* end = nullptr;
    long parsed = std::strtol(value, &end, 0);
    if (end == value || parsed < 0 || parsed > 0xffff) return default_value;
    return (int)parsed;
}

FeetechBus::FeetechBus(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {
    debug_ = std::getenv("LEKIWI_USB_DEBUG") != nullptr ||
             port_ == "auto" || port_ == "usb" || port_ == "libusb" || port_ == "cdc";
}

FeetechBus::~FeetechBus() {
    close();
}

bool FeetechBus::is_open() const {
    return backend_ == Backend::TTY ? fd_ >= 0 : usb_handle_ != nullptr;
}

bool FeetechBus::open() {
    if (is_open()) return true;

    if (port_ == "auto") {
        log_debug("auto backend: trying userspace libusb CDC first");
        if (open_usb_cdc()) {
            backend_ = Backend::USB_CDC;
            log_debug("auto backend selected: libusb CDC bus endpoint in=0x%02x out=0x%02x",
                      usb_ep_in_, usb_ep_out_);
            return true;
        }
        std::string usb_error = last_error_;
        log_debug("auto backend: libusb CDC unavailable: %s", usb_error.c_str());
        port_ = "/dev/ttyACM0";
        log_debug("auto backend: falling back to TTY %s", port_.c_str());
        if (!open_port()) {
            set_error("auto backend failed: libusb CDC: " + usb_error + "; tty: " + last_error_);
            return false;
        }
        backend_ = Backend::TTY;
        return configure_port();
    }

    if (port_ == "usb" || port_ == "libusb" || port_ == "cdc") {
        if (!open_usb_cdc()) return false;
        backend_ = Backend::USB_CDC;
        return true;
    }

    if (!open_port()) return false;
    backend_ = Backend::TTY;
    return configure_port();
}

void FeetechBus::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    close_usb();
    backend_ = Backend::NONE;
}

bool FeetechBus::open_port() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        set_error("open " + port_ + " failed: " + std::strerror(errno));
        return false;
    }
    log_debug("opened TTY %s baud=%d", port_.c_str(), baudrate_);
    return true;
}

static const char* libusb_err_name(int err) {
    return libusb_error_name(err);
}

static void warm_usbfs_device_dir() {
    DIR* dir = opendir("/dev/bus/usb/001");
    if (!dir) return;

    while (dirent* ent = readdir(dir)) {
        if (ent->d_name[0] == '.') continue;
        std::string path = std::string("/dev/bus/usb/001/") + ent->d_name;
        struct stat st;
        (void)stat(path.c_str(), &st);
    }
    closedir(dir);
}

void FeetechBus::log_debug(const char* fmt, ...) const {
    if (!debug_) return;
    fprintf(stderr, "[FeetechBus] ");
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
}

void FeetechBus::close_usb() {
    if (usb_handle_) {
        if (usb_data_claimed_ && usb_data_iface_ >= 0)
            libusb_release_interface(usb_handle_, usb_data_iface_);
        if (usb_control_claimed_ && usb_control_iface_ >= 0 && usb_control_iface_ != usb_data_iface_)
            libusb_release_interface(usb_handle_, usb_control_iface_);
        libusb_close(usb_handle_);
        usb_handle_ = nullptr;
    }
    if (usb_ctx_) {
        libusb_exit(usb_ctx_);
        usb_ctx_ = nullptr;
    }
    usb_control_iface_ = -1;
    usb_data_iface_ = -1;
    usb_control_claimed_ = false;
    usb_data_claimed_ = false;
    usb_ep_in_ = 0;
    usb_ep_out_ = 0;
}

static bool is_bulk_in(const libusb_endpoint_descriptor& ep) {
    return (ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN &&
           (ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK;
}

static bool is_bulk_out(const libusb_endpoint_descriptor& ep) {
    return (ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT &&
           (ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK;
}

bool FeetechBus::open_usb_cdc() {
    close_usb();
    warm_usbfs_device_dir();

    int rc = libusb_init(&usb_ctx_);
    if (rc != 0) {
        set_error("libusb_init failed: " + std::string(libusb_err_name(rc)));
        return false;
    }
    log_debug("libusb initialized");

    libusb_device** list = nullptr;
    ssize_t count = libusb_get_device_list(usb_ctx_, &list);
    if (count < 0) {
        set_error("libusb_get_device_list failed: " + std::string(libusb_err_name((int)count)));
        close_usb();
        return false;
    }
    if (count == 0) {
        libusb_free_device_list(list, 1);
        close_usb();
        usleep(100000);
        warm_usbfs_device_dir();

        rc = libusb_init(&usb_ctx_);
        if (rc != 0) {
            set_error("libusb_init retry failed: " + std::string(libusb_err_name(rc)));
            return false;
        }
        list = nullptr;
        count = libusb_get_device_list(usb_ctx_, &list);
        if (count < 0) {
            set_error("libusb_get_device_list retry failed: " + std::string(libusb_err_name((int)count)));
            close_usb();
            return false;
        }
    }
    log_debug("libusb device count=%zd", count);

    bool opened = false;
    std::string last_candidate_error;
    for (ssize_t i = 0; i < count && !opened; i++) {
        libusb_device* dev = list[i];
        libusb_device_descriptor desc{};
        rc = libusb_get_device_descriptor(dev, &desc);
        if (rc != 0) {
            log_debug("device[%zd] descriptor failed: %s", i, libusb_err_name(rc));
            continue;
        }

        uint8_t bus = libusb_get_bus_number(dev);
        uint8_t addr = libusb_get_device_address(dev);
        log_debug("device[%zd] bus=%u addr=%u vid=%04x pid=%04x class=%02x/%02x/%02x configs=%u",
                  i, bus, addr, desc.idVendor, desc.idProduct,
                  desc.bDeviceClass, desc.bDeviceSubClass, desc.bDeviceProtocol,
                  desc.bNumConfigurations);

        libusb_config_descriptor* cfg = nullptr;
        rc = libusb_get_active_config_descriptor(dev, &cfg);
        if (rc != 0) rc = libusb_get_config_descriptor(dev, 0, &cfg);
        if (rc != 0 || !cfg) {
            log_debug("device bus=%u addr=%u config failed: %s", bus, addr, libusb_err_name(rc));
            continue;
        }

        int control_iface = -1;
        int data_iface = -1;
        uint8_t ep_in = 0;
        uint8_t ep_out = 0;

        for (int if_idx = 0; if_idx < cfg->bNumInterfaces; if_idx++) {
            const libusb_interface& iface = cfg->interface[if_idx];
            for (int alt_idx = 0; alt_idx < iface.num_altsetting; alt_idx++) {
                const libusb_interface_descriptor& alt = iface.altsetting[alt_idx];
                log_debug("  iface=%d alt=%d class=%02x/%02x/%02x eps=%u",
                          alt.bInterfaceNumber, alt.bAlternateSetting,
                          alt.bInterfaceClass, alt.bInterfaceSubClass,
                          alt.bInterfaceProtocol, alt.bNumEndpoints);
                if (alt.bInterfaceClass == 0x02 && control_iface < 0) {
                    control_iface = alt.bInterfaceNumber;
                    log_debug("    candidate control interface=%d", control_iface);
                }
                if (alt.bInterfaceClass != 0x0a) continue;

                uint8_t found_in = 0;
                uint8_t found_out = 0;
                for (int ep_idx = 0; ep_idx < alt.bNumEndpoints; ep_idx++) {
                    const libusb_endpoint_descriptor& ep = alt.endpoint[ep_idx];
                    if (is_bulk_in(ep)) found_in = ep.bEndpointAddress;
                    if (is_bulk_out(ep)) found_out = ep.bEndpointAddress;
                    log_debug("    ep=0x%02x attr=0x%02x max=%u",
                              ep.bEndpointAddress, ep.bmAttributes, ep.wMaxPacketSize);
                }
                if (found_in && found_out) {
                    data_iface = alt.bInterfaceNumber;
                    ep_in = found_in;
                    ep_out = found_out;
                    log_debug("    candidate data interface=%d ep_in=0x%02x ep_out=0x%02x",
                              data_iface, ep_in, ep_out);
                }
            }
        }

        if (data_iface < 0 || !ep_in || !ep_out) {
            libusb_free_config_descriptor(cfg);
            continue;
        }
        if (control_iface < 0) control_iface = data_iface;

        rc = libusb_open(dev, &usb_handle_);
        if (rc != 0 || !usb_handle_) {
            last_candidate_error = "libusb_open bus=" + std::to_string(bus) +
                                   " addr=" + std::to_string(addr) +
                                   " failed: " + libusb_err_name(rc);
            log_debug("%s", last_candidate_error.c_str());
            libusb_free_config_descriptor(cfg);
            continue;
        }

        libusb_set_auto_detach_kernel_driver(usb_handle_, 1);
        log_debug("auto detach requested for bus=%u addr=%u", bus, addr);

        if (control_iface >= 0) {
            rc = libusb_claim_interface(usb_handle_, control_iface);
            if (rc == 0) {
                usb_control_claimed_ = true;
                log_debug("claimed control interface=%d", control_iface);
            } else {
                log_debug("claim control interface=%d failed: %s", control_iface, libusb_err_name(rc));
            }
        }

        if (data_iface == control_iface && usb_control_claimed_) {
            usb_data_claimed_ = true;
            log_debug("data interface=%d already claimed as control interface", data_iface);
        } else {
            rc = libusb_claim_interface(usb_handle_, data_iface);
            if (rc != 0) {
                last_candidate_error = "claim data interface=" + std::to_string(data_iface) +
                                       " failed: " + libusb_err_name(rc);
                log_debug("%s", last_candidate_error.c_str());
                libusb_free_config_descriptor(cfg);
                close_usb();
                continue;
            }
            usb_data_claimed_ = true;
        }

        usb_control_iface_ = control_iface;
        usb_data_iface_ = data_iface;
        usb_ep_in_ = ep_in;
        usb_ep_out_ = ep_out;
        usb_rx_buffer_.clear();
        usb_rx_offset_ = 0;
        rc = libusb_clear_halt(usb_handle_, usb_ep_in_);
        log_debug("clear halt ep_in=0x%02x rc=%d", usb_ep_in_, rc);
        rc = libusb_clear_halt(usb_handle_, usb_ep_out_);
        log_debug("clear halt ep_out=0x%02x rc=%d", usb_ep_out_, rc);
        libusb_free_config_descriptor(cfg);

        if (!configure_usb_cdc()) {
            last_candidate_error = last_error_;
            close_usb();
            continue;
        }

        opened = true;
        log_debug("selected CDC device bus=%u addr=%u vid=%04x pid=%04x control_if=%d data_if=%d",
                  bus, addr, desc.idVendor, desc.idProduct, usb_control_iface_, usb_data_iface_);
    }

    libusb_free_device_list(list, 1);
    if (!opened) {
        set_error(last_candidate_error.empty()
                  ? "no USB CDC ACM device with bulk in/out endpoints found"
                  : last_candidate_error);
        close_usb();
        return false;
    }
    return true;
}

bool FeetechBus::configure_usb_cdc() {
    if (!usb_handle_) return false;

    // CDC ACM line coding: dwDTERate, bCharFormat, bParityType, bDataBits.
    uint8_t line_coding[7] = {
        (uint8_t)(baudrate_ & 0xff),
        (uint8_t)((baudrate_ >> 8) & 0xff),
        (uint8_t)((baudrate_ >> 16) & 0xff),
        (uint8_t)((baudrate_ >> 24) & 0xff),
        0,
        0,
        8,
    };
    int iface = usb_control_iface_ >= 0 ? usb_control_iface_ : usb_data_iface_;
    int rc = libusb_control_transfer(
        usb_handle_,
        LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
        0x20, // SET_LINE_CODING
        0,
        (uint16_t)iface,
        line_coding,
        sizeof(line_coding),
        1000);
    log_debug("SET_LINE_CODING iface=%d baud=%d rc=%d", iface, baudrate_, rc);
    if (rc < 0) {
        set_error("SET_LINE_CODING failed: " + std::string(libusb_err_name(rc)));
        return false;
    }

    int control_line_state = parse_env_int("LEKIWI_CDC_CONTROL_LINE_STATE", 0x0003);
    rc = libusb_control_transfer(
        usb_handle_,
        LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
        0x22, // SET_CONTROL_LINE_STATE
        (uint16_t)control_line_state,
        (uint16_t)iface,
        nullptr,
        0,
        1000);
    log_debug("SET_CONTROL_LINE_STATE iface=%d value=0x%04x rc=%d",
              iface, control_line_state, rc);
    if (rc < 0) {
        set_error("SET_CONTROL_LINE_STATE failed: " + std::string(libusb_err_name(rc)));
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

bool FeetechBus::write_bytes(const uint8_t* data, size_t len) {
    if (backend_ == Backend::USB_CDC) {
        int transferred = 0;
        int rc = libusb_bulk_transfer(usb_handle_, usb_ep_out_, const_cast<uint8_t*>(data),
                                      (int)len, &transferred, 1000);
        if (rc != 0 || transferred != (int)len) {
            char msg[160];
            snprintf(msg, sizeof(msg),
                     "usb bulk OUT ep=0x%02x len=%zu transferred=%d failed: %s",
                     usb_ep_out_, len, transferred, libusb_err_name(rc));
            set_error(msg);
            return false;
        }
        return true;
    }

    ssize_t n = ::write(fd_, data, len);
    if (n != (ssize_t)len) {
        set_error("serial write failed: " + std::string(std::strerror(errno)));
        return false;
    }
    tcdrain(fd_);
    return true;
}

bool FeetechBus::read_byte(uint8_t& byte, int timeout_ms) {
    if (backend_ == Backend::USB_CDC) {
        if (usb_rx_offset_ < usb_rx_buffer_.size()) {
            byte = usb_rx_buffer_[usb_rx_offset_++];
            if (usb_rx_offset_ >= usb_rx_buffer_.size()) {
                usb_rx_buffer_.clear();
                usb_rx_offset_ = 0;
            }
            return true;
        }

        if (!read_usb_packet(timeout_ms)) return false;
        return read_byte(byte, 0);
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int rc = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (rc <= 0) return false;
    return ::read(fd_, &byte, 1) == 1;
}

bool FeetechBus::read_usb_packet(int timeout_ms) {
    if (backend_ != Backend::USB_CDC) return false;
    usb_rx_buffer_.clear();
    usb_rx_offset_ = 0;

    uint8_t tmp[64];
    int transferred = 0;
    int rc = libusb_bulk_transfer(usb_handle_, usb_ep_in_, tmp, sizeof(tmp), &transferred,
                                  timeout_ms);
    if (rc == LIBUSB_ERROR_TIMEOUT || transferred == 0) return false;
    if (rc != 0) {
        char msg[160];
        snprintf(msg, sizeof(msg), "usb bulk IN ep=0x%02x len=%zu failed: %s",
                 usb_ep_in_, sizeof(tmp), libusb_err_name(rc));
        set_error(msg);
        return false;
    }
    if (transferred < 0) {
        char msg[160];
        snprintf(msg, sizeof(msg), "usb bulk IN ep=0x%02x invalid length=%d",
                 usb_ep_in_, transferred);
        set_error(msg);
        return false;
    }

    usb_rx_buffer_.assign(tmp, tmp + transferred);
    usb_rx_offset_ = 0;
    return true;
}

void FeetechBus::flush_input() {
    if (backend_ == Backend::USB_CDC) {
        // Avoid a timeout-based drain: cancelled IN URBs can remain queued on Starry.
        usb_rx_buffer_.clear();
        usb_rx_offset_ = 0;
        return;
    }
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

bool FeetechBus::tx_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params) {
    if (!is_open() && !open()) return false;

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

    return write_bytes(pkt.data(), pkt.size());
}

bool FeetechBus::rx_status(uint8_t expected_id, std::vector<uint8_t>& params, uint8_t* error_out,
                           int timeout_ms) {
    params.clear();
    if (error_out) *error_out = 0xff;

    const int max_packets = expected_id == BROADCAST_ID ? 1 : 4;
    int attempts_left = max_packets;
    while (attempts_left-- > 0) {
        std::vector<uint8_t> buf;
        buf.reserve(64);

        uint8_t b = 0;
        int elapsed = 0;
        while (elapsed < timeout_ms) {
            if (backend_ == Backend::USB_CDC && usb_rx_offset_ >= usb_rx_buffer_.size()) {
                if (!read_usb_packet(timeout_ms - elapsed)) {
                    elapsed = timeout_ms;
                    break;
                }
            }
            if (!read_byte(b, 5)) { elapsed += 5; continue; }
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
        if (!read_byte(id, 20) || !read_byte(length, 20) || !read_byte(error, 20)) {
            set_error("rx timeout reading status header");
            return false;
        }
        if (length < 2) {
            set_error("invalid status length");
            return false;
        }

        int param_len = length - 2;
        std::vector<uint8_t> packet_params(param_len);
        for (int i = 0; i < param_len; i++) {
            if (!read_byte(packet_params[i], 20)) {
                set_error("rx timeout reading params");
                return false;
            }
        }
        uint8_t chk = 0;
        if (!read_byte(chk, 20)) {
            set_error("rx timeout reading checksum");
            return false;
        }

        uint16_t sum = id + length + error;
        for (uint8_t p : packet_params) sum += p;
        uint8_t expected_chk = (uint8_t)(~sum & 0xff);
        if (chk != expected_chk) {
            set_error("status checksum mismatch");
            return false;
        }

        if (expected_id != BROADCAST_ID && id != expected_id) {
            char msg[160];
            snprintf(msg, sizeof(msg), "unexpected status id: expected=%u got=%u len=%u",
                     expected_id, id, length);
            set_error(msg);
            if (attempts_left > 0) {
                log_debug("discarding stale status packet: %s", msg);
                continue;
            }
            return false;
        }

        params.swap(packet_params);
        if (error_out) *error_out = error;
        return true;
    }

    set_error("rx exhausted while waiting for matching status id");
    return false;
}

bool FeetechBus::tx_rx(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params,
                       std::vector<uint8_t>& reply, uint8_t* error_out) {
    flush_input();
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
    int attempts = backend_ == Backend::USB_CDC ? 2 : 1;
    for (int attempt = 0; attempt < attempts; attempt++) {
        found.clear();
        for (int id = first_id; id <= last_id; id++) {
            if (ping(id)) found.push_back(id);
            usleep(10000);
        }
        if (!found.empty() || attempt + 1 >= attempts) break;
        log_debug("scan returned no motors; retrying once after USB CDC settle");
        usleep(200000);
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
