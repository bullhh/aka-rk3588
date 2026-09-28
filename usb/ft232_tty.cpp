#include "usb/ft232_tty.hpp"

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <termios.h>
#include <time.h>
#include <vector>

#include <unistd.h>

namespace ft232 {

namespace {

const uint64_t TTY_WRITE_DEADLINE_US  = 400000;
const uint64_t TTY_READ_DEADLINE_US   = 400000;
const uint64_t TTY_LEFTOVER_TIMEOUT_US = 5000;
const size_t   MAX_LOOPBACK_FRAME     = 256;
const char*    SYSFS_USB_DEVICES      = "/sys/bus/usb/devices";

uint64_t monotonic_us()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

bool read_attribute(const std::string& path, std::string& out)
{
    FILE* file = fopen(path.c_str(), "r");
    if (!file) return false;
    char buffer[256];
    const size_t n = fread(buffer, 1, sizeof(buffer) - 1, file);
    fclose(file);
    if (n == 0) return false;
    buffer[n] = '\0';
    out = ftdi_proto::trim_attribute_value(std::string(buffer));
    return true;
}

// Map a baud rate to a Linux termios speed. Only the rates we can name exactly
// are accepted; anything else is reported as unsupported rather than silently
// running at the wrong speed.
bool tty_speed(int baudrate, speed_t& speed)
{
    switch (baudrate) {
#ifdef B1000000
    case 1000000: speed = B1000000; return true;
#endif
#ifdef B921600
    case 921600:  speed = B921600;  return true;
#endif
#ifdef B115200
    case 115200:  speed = B115200;  return true;
#endif
#ifdef B9600
    case 9600:    speed = B9600;    return true;
#endif
    default:
        return false;
    }
}

// Poll for readiness in bounded slices so a caller deadline is honoured.
// Returns >0 ready, 0 timeout slice, -1 error (errno set).
int poll_until(int fd, short events, uint64_t deadline_us)
{
    const uint64_t now = monotonic_us();
    if (now >= deadline_us) return 0;
    long remaining_ms = static_cast<long>((deadline_us - now) / 1000);
    if (remaining_ms <= 0) remaining_ms = 1;
    if (remaining_ms > 100) remaining_ms = 100;
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = events;
    pfd.revents = 0;
    int rc;
    do {
        rc = poll(&pfd, 1, static_cast<int>(remaining_ms));
    } while (rc < 0 && errno == EINTR);
    return rc;
}

// Append every "/dev/ttyUSB<n>" node found directly under `dir`, de-duplicated.
// Both the usual USB-serial layout (ttyUSBn is a direct child of the interface)
// and the alternative ".../tty/ttyUSBn" layout are supported; anything that is
// not a ttyUSB node is ignored.
void collect_tty_nodes(const std::string& dir, const std::string& serial,
                       bool serial_unreadable,
                       std::vector<ftdi_proto::Ft232Candidate>& out)
{
    DIR* node = opendir(dir.c_str());
    if (!node) return;
    struct dirent* entry;
    while ((entry = readdir(node)) != nullptr) {
        if (strncmp(entry->d_name, "ttyUSB", 6) != 0) continue;
        const std::string path = std::string("/dev/") + entry->d_name;
        if (!ftdi_proto::is_allowed_tty_path(path)) continue;
        bool duplicate = false;
        for (size_t i = 0; i < out.size(); i++) {
            if (out[i].device_path == path) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;
        ftdi_proto::Ft232Candidate candidate;
        candidate.vid = ftdi_proto::FTDI_VID;
        candidate.pid = ftdi_proto::FT232R_PID;
        candidate.serial = serial;
        candidate.serial_unreadable = serial_unreadable;
        candidate.device_path = path;
        out.push_back(candidate);
    }
    closedir(node);
}

} // namespace

Ft232Tty::Ft232Tty() {}

Ft232Tty::~Ft232Tty()
{
    close();
}

bool Ft232Tty::enumerate_candidates(
    std::vector<ftdi_proto::Ft232Candidate>& out, std::string& error)
{
    DIR* devices = opendir(SYSFS_USB_DEVICES);
    if (!devices) {
        error = std::string("cannot scan ") + SYSFS_USB_DEVICES;
        return false;
    }
    struct dirent* entry;
    while ((entry = readdir(devices)) != nullptr) {
        if (entry->d_name[0] == '.') continue;
        const std::string device_dir =
            std::string(SYSFS_USB_DEVICES) + "/" + entry->d_name;
        std::string vendor;
        std::string product;
        if (!read_attribute(device_dir + "/idVendor", vendor)) continue;
        if (!read_attribute(device_dir + "/idProduct", product)) continue;
        if (vendor != "0403" || product != "6001") continue;
        std::string serial;
        // The sysfs serial is the strict identity used by the TTY fallback. If
        // it is absent/unreadable, a non-empty serial selector cannot be proven
        // to match, so mark the candidate unreadable instead of pretending it
        // has no serial.
        const bool serial_readable =
            read_attribute(device_dir + "/serial", serial);

        DIR* node = opendir(device_dir.c_str());
        if (!node) continue;
        struct dirent* interface;
        while ((interface = readdir(node)) != nullptr) {
            if (interface->d_name[0] == '.') continue;
            // Interface directories are named "<dev>:<cfg>.<iface>".
            if (strchr(interface->d_name, ':') == nullptr) continue;
            const std::string interface_dir = device_dir + "/" + interface->d_name;
            // Common layout: the ttyUSB node is a direct child of the interface.
            collect_tty_nodes(interface_dir, serial, !serial_readable, out);
            // Alternative layout: ".../<usb-iface>/tty/ttyUSBn".
            collect_tty_nodes(interface_dir + "/tty", serial, !serial_readable, out);
        }
        closedir(node);
    }
    closedir(devices);
    return true;
}

bool Ft232Tty::open(const std::string& serial_selector, int baudrate,
                    std::string& error)
{
    close();
    open_failure_ = OpenFailure::NONE;

    std::vector<ftdi_proto::Ft232Candidate> candidates;
    if (!enumerate_candidates(candidates, error)) {
        open_failure_ = OpenFailure::OTHER;
        return false;
    }

    ftdi_proto::Ft232Candidate chosen;
    const ftdi_proto::SelectResult selection =
        ftdi_proto::select_ft232(candidates, serial_selector, chosen);
    if (selection != ftdi_proto::SelectResult::OK) {
        error = std::string("ft232 tty selection failed: ") +
                ftdi_proto::select_result_string(selection);
        // The TTY backend has already used sysfs serial; there is no further
        // fallback that could resolve an unreadable or unmatched selector.
        open_failure_ = selection == ftdi_proto::SelectResult::NONE
            ? OpenFailure::NO_DEVICE : OpenFailure::AMBIGUOUS;
        return false;
    }

    // Defence in depth: never open anything but a Linux USB-serial node.
    if (!ftdi_proto::is_allowed_tty_path(chosen.device_path)) {
        error = "refusing non-ttyUSB path: " + chosen.device_path;
        open_failure_ = OpenFailure::OTHER;
        return false;
    }

    fd_ = ::open(chosen.device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        error = "open " + chosen.device_path + " failed: " + strerror(errno);
        open_failure_ = (errno == EACCES || errno == EPERM)
            ? OpenFailure::ACCESS_DENIED : OpenFailure::OTHER;
        return false;
    }

    if (!configure_termios(baudrate, error)) {
        ::close(fd_);
        fd_ = -1;
        open_failure_ = OpenFailure::OTHER;
        return false;
    }

    identity_ = DeviceIdentity{};
    identity_.vid = ftdi_proto::FTDI_VID;
    identity_.pid = ftdi_proto::FT232R_PID;
    identity_.serial = chosen.serial;
    identity_.tty_path = chosen.device_path;
    open_failure_ = OpenFailure::NONE;
    return true;
}

bool Ft232Tty::configure_termios(int baudrate, std::string& error)
{
    speed_t speed = 0;
    if (!tty_speed(baudrate, speed)) {
        error = "unsupported tty baud rate";
        return false;
    }
    struct termios tio;
    if (tcgetattr(fd_, &tio) != 0) {
        error = std::string("tcgetattr failed: ") + strerror(errno);
        return false;
    }
    // Raw 8N1: no canonical processing, no echo, no flow control.
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                     IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    tio.c_cflag |= (CS8 | CREAD | CLOCAL);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        error = std::string("cfsetispeed/cfsetospeed failed: ") + strerror(errno);
        return false;
    }
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        error = std::string("tcsetattr failed: ") + strerror(errno);
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

bool Ft232Tty::write_all(const uint8_t* data, size_t len, std::string& error)
{
    size_t written = 0;
    const uint64_t deadline = monotonic_us() + TTY_WRITE_DEADLINE_US;
    while (written < len) {
        const ssize_t n = ::write(fd_, data + written, len - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            error = std::string("tty write failed: ") + strerror(errno);
            return false;
        }
        const int ready = poll_until(fd_, POLLOUT, deadline);
        if (ready < 0) {
            error = std::string("poll(POLLOUT) failed: ") + strerror(errno);
            return false;
        }
        if (ready == 0 && monotonic_us() >= deadline) {
            error = "tty write timeout";
            return false;
        }
    }
    return true;
}

bool Ft232Tty::read_exact(const uint8_t* /*expected*/, size_t len, uint8_t* got,
                          size_t& got_len, std::string& error)
{
    got_len = 0;
    const uint64_t deadline = monotonic_us() + TTY_READ_DEADLINE_US;
    while (got_len < len) {
        const int ready = poll_until(fd_, POLLIN, deadline);
        if (ready < 0) {
            error = std::string("poll(POLLIN) failed: ") + strerror(errno);
            return false;
        }
        if (ready == 0) {
            if (monotonic_us() >= deadline) {
                error = "tty read timeout";
                return false;
            }
            continue;
        }
        const ssize_t n = ::read(fd_, got + got_len, len - got_len);
        if (n > 0) {
            got_len += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            error = std::string("tty read failed: ") + strerror(errno);
            return false;
        }
        if (monotonic_us() >= deadline) {
            error = "tty read timeout";
            return false;
        }
    }
    return true;
}

bool Ft232Tty::exchange(const uint8_t* frame, size_t len, std::string& error,
                        uint64_t* latency_us)
{
    if (fd_ < 0) { error = "tty not open"; return false; }
    if (!frame || len == 0 || len > MAX_LOOPBACK_FRAME) {
        error = "invalid loopback frame";
        return false;
    }
    const uint64_t start = monotonic_us();

    if (!write_all(frame, len, error)) return false;

    uint8_t got[MAX_LOOPBACK_FRAME];
    size_t got_len = 0;
    if (!read_exact(frame, len, got, got_len, error)) return false;

    // Nothing may remain buffered: leftover bytes mean a duplicated/stale frame.
    {
        const uint64_t deadline = monotonic_us() + TTY_LEFTOVER_TIMEOUT_US;
        const int ready = poll_until(fd_, POLLIN, deadline);
        if (ready > 0) {
            uint8_t extra[MAX_LOOPBACK_FRAME];
            const ssize_t n = ::read(fd_, extra, sizeof(extra));
            if (n > 0) { error = "trailing/duplicated echo bytes"; return false; }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                error = std::string("tty read failed: ") + strerror(errno);
                return false;
            }
        } else if (ready < 0) {
            error = std::string("poll(POLLIN) failed: ") + strerror(errno);
            return false;
        }
    }

    const ftdi_proto::EchoResult result =
        ftdi_proto::classify_echo(frame, len, got, got_len);
    if (result != ftdi_proto::EchoResult::MATCH) {
        error = std::string("echo mismatch: ") + ftdi_proto::echo_result_string(result);
        return false;
    }
    if (latency_us) *latency_us = monotonic_us() - start;
    return true;
}

bool Ft232Tty::close()
{
    bool ok = true;
    if (fd_ >= 0) {
        if (::close(fd_) != 0) {
            ok = false;
            cleanup_error_ = std::string("close_failed: ") + strerror(errno);
        }
        fd_ = -1;
    }
    return ok;
}

} // namespace ft232
