#ifndef FT232_LOOPBACK_HPP
#define FT232_LOOPBACK_HPP

// libusb-backed FT232R (0403:6001) binary loopback transport plus the bounded
// cadence worker used by the vision+USB CI workload.
//
// The device is driven through raw vendor control requests and bulk endpoints
// so Linux and StarryOS both exercise USB host / usbfs. This deliberately does
// not use the Linux ftdi_sio TTY driver, so a PASS here says nothing about
// /dev/ttyUSB0 TTY support.

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "usb/ftdi_protocol.hpp"
#include "usb/loopback_transport.hpp"

struct libusb_context;
struct libusb_device_handle;

namespace ft232 {

class Ft232Loopback : public LoopbackTransport {
public:
    Ft232Loopback();
    ~Ft232Loopback() override;
    Ft232Loopback(const Ft232Loopback&) = delete;
    Ft232Loopback& operator=(const Ft232Loopback&) = delete;

    // Enumerate exactly one 0403:6001 device (optionally by serial), open it,
    // detach the kernel driver when needed and run the FTDI init required for
    // binary traffic at `baudrate`. Returns false and fills `error` otherwise.
    bool open(const std::string& serial_selector, int baudrate,
              std::string& error) override;
    // Release the interface and reattach the kernel driver if we detached it.
    // Returns false when a release/reattach step failed; `cleanup_error()` then
    // describes it so the caller can refuse to report success.
    bool close() override;
    const std::string& cleanup_error() const override { return cleanup_error_; }
    bool is_open() const override { return handle_ != nullptr; }
    const DeviceIdentity& identity() const override { return identity_; }
    const char* transport_name() const override { return "usb"; }
    OpenFailure open_failure() const override { return open_failure_; }

    // One full write + echo read + exact compare with bounded timeouts and no
    // implicit retry. Returns true only when the echo matches byte-for-byte.
    bool exchange(const uint8_t* frame, size_t len, std::string& error,
                  uint64_t* latency_us) override;

private:
    bool enumerate_candidates(std::vector<ftdi_proto::Ft232Candidate>& out,
                              std::string& error);
    bool configure_endpoints(std::string& error);
    bool ftdi_init(int baudrate, std::string& error);
    bool control_out(uint8_t request, uint16_t value, uint16_t index,
                     std::string& error);
    bool read_echo(const uint8_t* expected, size_t len, uint8_t* got,
                   std::string& error);

    libusb_context*       ctx_    = nullptr;
    libusb_device_handle* handle_ = nullptr;
    int        interface_number_ = 0;
    uint8_t    ep_in_  = 0;
    uint8_t    ep_out_ = 0;
    bool       claimed_  = false;
    bool       detached_ = false;
    DeviceIdentity identity_{};
    std::string cleanup_error_;
    OpenFailure open_failure_ = OpenFailure::NONE;
};

// Drives Ft232Loopback at a fixed cadence on a background thread while the main
// thread runs the visual windows. Stats are bucketed per window so the caller
// can report per-window counts and latency without unbounded history.
class LoopbackWorker {
public:
    LoopbackWorker(LoopbackTransport& device, int cadence_hz, uint8_t frame_id);
    ~LoopbackWorker();
    LoopbackWorker(const LoopbackWorker&) = delete;
    LoopbackWorker& operator=(const LoopbackWorker&) = delete;

    void start();
    void stop();
    void set_window(int index);
    ftdi_proto::WindowStats snapshot(int index) const;
    bool fatal() const { return fatal_.load(); }
    std::string fatal_error() const;

private:
    void run();

    LoopbackTransport& device_;
    int      cadence_hz_;
    uint8_t  frame_id_;

    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    bool                    stopping_ = false;
    std::atomic<int>        window_{0};
    std::atomic<bool>       fatal_{false};
    std::string             fatal_error_;
    ftdi_proto::WindowStats windows_[2];
    std::thread             thread_;
};

} // namespace ft232

#endif // FT232_LOOPBACK_HPP
