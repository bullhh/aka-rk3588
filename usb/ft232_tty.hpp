#ifndef FT232_TTY_HPP
#define FT232_TTY_HPP

// Linux /dev/ttyUSBx fallback transport for the FT232R loopback workload.
//
// Used when the raw libusb open is denied (for example when /dev/bus/usb/... is
// root-only and the board user cannot sudo). The device is still identified as
// an FT232R by 0403:6001 through sysfs, and only a "/dev/ttyUSB" node that
// belongs to that exact USB device is ever opened, so /dev/ttyS6 and other USB
// serial ports can never be selected. Raw libusb remains the path for StarryOS
// and root Linux runs.
//
// termios raw 8N1 at 1 Mbps, bounded write/read deadlines and strict
// byte-for-byte frame verification (shared with the USB transport).

#include <string>
#include <vector>

#include "usb/ftdi_protocol.hpp"
#include "usb/loopback_transport.hpp"

namespace ft232 {

class Ft232Tty : public LoopbackTransport {
public:
    Ft232Tty();
    ~Ft232Tty() override;
    Ft232Tty(const Ft232Tty&) = delete;
    Ft232Tty& operator=(const Ft232Tty&) = delete;

    bool open(const std::string& serial_selector, int baudrate,
              std::string& error) override;
    bool exchange(const uint8_t* frame, size_t len, std::string& error,
                  uint64_t* latency_us) override;
    bool close() override;
    bool is_open() const override { return fd_ >= 0; }
    const DeviceIdentity& identity() const override { return identity_; }
    const char* transport_name() const override { return "tty"; }
    const std::string& cleanup_error() const override { return cleanup_error_; }
    OpenFailure open_failure() const override { return open_failure_; }

    // Exposed for the pure selection/transport tests (no hardware needed).
    static bool enumerate_candidates(std::vector<ftdi_proto::Ft232Candidate>& out,
                                     std::string& error);

private:
    bool configure_termios(int baudrate, std::string& error);
    bool write_all(const uint8_t* data, size_t len, std::string& error);
    bool read_exact(const uint8_t* expected, size_t len, uint8_t* got,
                    size_t& got_len, std::string& error);

    int         fd_ = -1;
    DeviceIdentity identity_{};
    std::string cleanup_error_;
    OpenFailure open_failure_ = OpenFailure::NONE;
};

} // namespace ft232

#endif // FT232_TTY_HPP
