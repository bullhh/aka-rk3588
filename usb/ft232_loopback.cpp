#include "usb/ft232_loopback.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <time.h>
#include <vector>

#include <libusb-1.0/libusb.h>

namespace ft232 {

namespace {

const int      FTDI_READ_CHUNK          = 64;   // FT232R full-speed bulk packet
const int      FTDI_WRITE_TIMEOUT_MS    = 200;
const int      FTDI_READ_TIMEOUT_MS     = 20;
const uint64_t FTDI_READ_DEADLINE_US    = 200000;
const int      FTDI_LEFTOVER_TIMEOUT_MS = 3;
const size_t   MAX_LOOPBACK_FRAME       = 256;

uint64_t monotonic_us()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

std::string error_name(int rc)
{
    const char* name = libusb_error_name(rc);
    return name ? std::string(name) : (std::string("rc=") + std::to_string(rc));
}

} // namespace

// ── Ft232Loopback ─────────────────────────────────────────────────────────────
Ft232Loopback::Ft232Loopback() {}

Ft232Loopback::~Ft232Loopback()
{
    close();
}

bool Ft232Loopback::enumerate_candidates(
    std::vector<ftdi_proto::Ft232Candidate>& out, std::string& error)
{
    if (!ctx_) { error = "libusb context not initialised"; return false; }
    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(ctx_, &list);
    if (count < 0) { error = "libusb_get_device_list failed"; return false; }

    for (ssize_t i = 0; i < count; i++) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
        if (desc.idVendor != ftdi_proto::FTDI_VID ||
            desc.idProduct != ftdi_proto::FT232R_PID) {
            continue;
        }
        ftdi_proto::Ft232Candidate candidate;
        candidate.vid = desc.idVendor;
        candidate.pid = desc.idProduct;
        candidate.bus = libusb_get_bus_number(list[i]);
        candidate.address = libusb_get_device_address(list[i]);
        if (desc.iSerialNumber != 0) {
            libusb_device_handle* probe = nullptr;
            bool serial_read = false;
            if (libusb_open(list[i], &probe) == 0) {
                unsigned char buffer[128];
                const int length = libusb_get_string_descriptor_ascii(
                    probe, desc.iSerialNumber, buffer, sizeof(buffer));
                if (length > 0) {
                    candidate.serial.assign(reinterpret_cast<char*>(buffer),
                                            static_cast<size_t>(length));
                    serial_read = true;
                }
                libusb_close(probe);
            }
            // A string descriptor exists, but this user could not read it (for
            // example /dev/bus/usb is root-only). Do not treat that as a
            // nonexistent serial: the sysfs/TTY backend can still select the
            // same device by serial.
            candidate.serial_unreadable = !serial_read;
        }
        out.push_back(candidate);
    }
    libusb_free_device_list(list, 1);
    return true;
}

bool Ft232Loopback::open(const std::string& serial_selector, int baudrate,
                         std::string& error)
{
    close();
    open_failure_ = OpenFailure::NONE;
    if (libusb_init(&ctx_) != 0) {
        ctx_ = nullptr;
        error = "libusb_init failed";
        open_failure_ = OpenFailure::UNSUPPORTED;
        return false;
    }

    std::vector<ftdi_proto::Ft232Candidate> candidates;
    if (!enumerate_candidates(candidates, error)) {
        open_failure_ = OpenFailure::OTHER;
        close();
        return false;
    }

    ftdi_proto::Ft232Candidate chosen;
    const ftdi_proto::SelectResult selection =
        ftdi_proto::select_ft232(candidates, serial_selector, chosen);
    if (selection != ftdi_proto::SelectResult::OK) {
        error = std::string("ft232 selection failed: ") +
                ftdi_proto::select_result_string(selection);
        if (selection == ftdi_proto::SelectResult::NONE) {
            open_failure_ = OpenFailure::NO_DEVICE;
        } else if (selection == ftdi_proto::SelectResult::SERIAL_UNREADABLE) {
            open_failure_ = OpenFailure::SERIAL_UNREADABLE;
        } else {
            open_failure_ = OpenFailure::AMBIGUOUS;
        }
        close();
        return false;
    }

    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(ctx_, &list);
    if (count < 0) { error = "libusb_get_device_list failed"; close(); return false; }
    libusb_device* target = nullptr;
    for (ssize_t i = 0; i < count; i++) {
        if (libusb_get_bus_number(list[i]) == chosen.bus &&
            libusb_get_device_address(list[i]) == chosen.address) {
            target = list[i];
            break;
        }
    }
    if (!target) {
        libusb_free_device_list(list, 1);
        error = "selected ft232 disappeared";
        close();
        return false;
    }

    const int open_rc = libusb_open(target, &handle_);
    libusb_free_device_list(list, 1);
    if (open_rc != 0) {
        handle_ = nullptr;
        error = "libusb_open failed: " + error_name(open_rc);
        open_failure_ = open_rc == LIBUSB_ERROR_ACCESS
            ? OpenFailure::ACCESS_DENIED : OpenFailure::OTHER;
        close();
        return false;
    }

    // Some adapters enumerate with a non-default active configuration.
    int active_config = 0;
    if (libusb_get_configuration(handle_, &active_config) == 0 && active_config == 0) {
        const int set_rc = libusb_set_configuration(handle_, 1);
        if (set_rc != 0 && set_rc != LIBUSB_ERROR_BUSY) {
            error = "libusb_set_configuration failed: " + error_name(set_rc);
            open_failure_ = set_rc == LIBUSB_ERROR_ACCESS
                ? OpenFailure::ACCESS_DENIED : OpenFailure::OTHER;
            close();
            return false;
        }
    }

    if (!configure_endpoints(error)) {
        open_failure_ = OpenFailure::OTHER;
        close();
        return false;
    }

    if (libusb_kernel_driver_active(handle_, interface_number_) == 1) {
        const int detach_rc = libusb_detach_kernel_driver(handle_, interface_number_);
        if (detach_rc != 0) {
            error = "detach kernel driver failed: " + error_name(detach_rc);
            open_failure_ = detach_rc == LIBUSB_ERROR_ACCESS
                ? OpenFailure::ACCESS_DENIED : OpenFailure::OTHER;
            close();
            return false;
        }
        detached_ = true;
    }

    const int claim_rc = libusb_claim_interface(handle_, interface_number_);
    if (claim_rc != 0) {
        error = "libusb_claim_interface failed: " + error_name(claim_rc);
        open_failure_ = claim_rc == LIBUSB_ERROR_ACCESS
            ? OpenFailure::ACCESS_DENIED : OpenFailure::OTHER;
        close();
        return false;
    }
    claimed_ = true;

    if (!ftdi_init(baudrate, error)) {
        open_failure_ = OpenFailure::OTHER;
        close();
        return false;
    }

    identity_.vid = chosen.vid;
    identity_.pid = chosen.pid;
    identity_.bus = chosen.bus;
    identity_.address = chosen.address;
    identity_.interface_number = interface_number_;
    identity_.endpoint_in = ep_in_;
    identity_.endpoint_out = ep_out_;
    identity_.serial = chosen.serial;
    open_failure_ = OpenFailure::NONE;
    return true;
}

bool Ft232Loopback::configure_endpoints(std::string& error)
{
    libusb_device* device = libusb_get_device(handle_);
    libusb_config_descriptor* config = nullptr;
    if (libusb_get_active_config_descriptor(device, &config) == 0 && config) {
        if (config->bNumInterfaces > 0) {
            const libusb_interface& interface = config->interface[0];
            if (interface.num_altsetting > 0) {
                const libusb_interface_descriptor& alt = interface.altsetting[0];
                interface_number_ = alt.bInterfaceNumber;
                for (int i = 0; i < alt.bNumEndpoints; i++) {
                    const uint8_t address = alt.endpoint[i].bEndpointAddress;
                    const uint8_t attributes = alt.endpoint[i].bmAttributes &
                                               LIBUSB_TRANSFER_TYPE_MASK;
                    if (attributes != LIBUSB_TRANSFER_TYPE_BULK) continue;
                    if ((address & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN)
                        ep_in_ = address;
                    else
                        ep_out_ = address;
                }
            }
        }
        libusb_free_config_descriptor(config);
    }

    // FT232R defaults when the descriptor walk did not find both endpoints.
    if (ep_in_ == 0)  ep_in_  = 0x81;
    if (ep_out_ == 0) ep_out_ = 0x02;
    if (ep_in_ == 0 || ep_out_ == 0) {
        error = "ft232 bulk endpoints not found";
        return false;
    }
    return true;
}

bool Ft232Loopback::control_out(uint8_t request, uint16_t value, uint16_t index,
                                std::string& error)
{
    const int rc = libusb_control_transfer(
        handle_, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE |
                 LIBUSB_ENDPOINT_OUT,
        request, value, index, nullptr, 0, 1000);
    if (rc < 0) {
        char text[64];
        snprintf(text, sizeof(text), "FTDI control 0x%02x failed: ", request);
        error = std::string(text) + error_name(rc);
        return false;
    }
    return true;
}

bool Ft232Loopback::ftdi_init(int baudrate, std::string& error)
{
    // Reset the controller, then flush both directions before configuring.
    if (!control_out(ftdi_proto::FTDI_REQ_RESET, ftdi_proto::FTDI_RESET_SIO, 0, error))
        return false;
    if (!control_out(ftdi_proto::FTDI_REQ_RESET, ftdi_proto::FTDI_RESET_PURGE_RX, 0, error))
        return false;
    if (!control_out(ftdi_proto::FTDI_REQ_RESET, ftdi_proto::FTDI_RESET_PURGE_TX, 0, error))
        return false;

    // FT232R: wValue carries the low 16 bits of the divisor, wIndex the high
    // 16 bits. 1 Mbps maps to divisor 3, so wValue=3, wIndex=0.
    const uint32_t divisor = ftdi_proto::ft232r_divisor(baudrate);
    if (!control_out(ftdi_proto::FTDI_REQ_SET_BAUDRATE,
                     static_cast<uint16_t>(divisor & 0xFFFFu),
                     static_cast<uint16_t>((divisor >> 16) & 0xFFFFu), error))
        return false;
    if (!control_out(ftdi_proto::FTDI_REQ_SET_DATA, ftdi_proto::FTDI_DATA_8N1, 0, error))
        return false;
    // No flow control: loopback wiring has no handshake lines.
    if (!control_out(ftdi_proto::FTDI_REQ_SET_FLOW_CTRL, 0, 0, error))
        return false;
    if (!control_out(ftdi_proto::FTDI_REQ_SET_LATENCY_TIMER,
                     ftdi_proto::FTDI_LATENCY_TIMER_MS, 0, error))
        return false;
    return true;
}

bool Ft232Loopback::close()
{
    bool ok = true;
    if (handle_) {
        if (claimed_) {
            const int release_rc = libusb_release_interface(handle_, interface_number_);
            if (release_rc != 0) {
                ok = false;
                cleanup_error_ = "release_interface_failed: " + error_name(release_rc);
            }
        }
        if (detached_) {
            const int attach_rc = libusb_attach_kernel_driver(handle_, interface_number_);
            if (attach_rc != 0) {
                ok = false;
                if (cleanup_error_.empty())
                    cleanup_error_ = "attach_kernel_driver_failed: " + error_name(attach_rc);
            }
        }
        libusb_close(handle_);
        handle_ = nullptr;
    }
    if (ctx_) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }
    claimed_ = false;
    detached_ = false;
    interface_number_ = 0;
    ep_in_ = 0;
    ep_out_ = 0;
    identity_ = DeviceIdentity{};
    return ok;
}

bool Ft232Loopback::read_echo(const uint8_t* expected, size_t len, uint8_t* got,
                              std::string& error)
{
    size_t got_len = 0;
    bool overflow = false;
    const uint64_t start = monotonic_us();
    const uint64_t deadline = start + FTDI_READ_DEADLINE_US;

    while (got_len < len) {
        if (monotonic_us() >= deadline) { error = "echo read timeout"; return false; }
        uint8_t raw[FTDI_READ_CHUNK];
        int transferred = 0;
        const int rc = libusb_bulk_transfer(handle_, ep_in_, raw, FTDI_READ_CHUNK,
                                            &transferred, FTDI_READ_TIMEOUT_MS);
        if (rc == LIBUSB_ERROR_TIMEOUT) continue;  // blocking wait, not busy poll
        if (rc != 0) { error = "bulk read failed: " + error_name(rc); return false; }
        uint8_t data[FTDI_READ_CHUNK];
        const int data_len = ftdi_proto::strip_status_bytes(
            raw, transferred, data, static_cast<int>(sizeof(data)));
        if (data_len < 0) { error = "malformed FTDI IN packet"; return false; }
        for (int i = 0; i < data_len; i++) {
            if (got_len < len) got[got_len++] = data[i];
            else overflow = true;
        }
    }
    if (overflow) { error = "duplicated echo bytes"; return false; }

    // Nothing may remain buffered: leftover data means a duplicated/stale frame.
    {
        uint8_t raw[FTDI_READ_CHUNK];
        int transferred = 0;
        const int rc = libusb_bulk_transfer(handle_, ep_in_, raw, FTDI_READ_CHUNK,
                                            &transferred, FTDI_LEFTOVER_TIMEOUT_MS);
        if (rc == 0 && transferred > 0) {
            uint8_t data[FTDI_READ_CHUNK];
            const int data_len = ftdi_proto::strip_status_bytes(
                raw, transferred, data, static_cast<int>(sizeof(data)));
            if (data_len > 0) { error = "trailing/duplicated echo bytes"; return false; }
        } else if (rc != 0 && rc != LIBUSB_ERROR_TIMEOUT) {
            error = "bulk read failed: " + error_name(rc);
            return false;
        }
    }

    if (got_len == len && memcmp(got, expected, len) == 0) return true;

    const ftdi_proto::EchoResult result =
        ftdi_proto::classify_echo(expected, len, got, got_len);
    error = std::string("echo mismatch: ") +
            ftdi_proto::echo_result_string(result);
    return false;
}

bool Ft232Loopback::exchange(const uint8_t* frame, size_t len, std::string& error,
                             uint64_t* latency_us)
{
    if (!handle_) { error = "ft232 not open"; return false; }
    if (!frame || len == 0 || len > MAX_LOOPBACK_FRAME) {
        error = "invalid loopback frame";
        return false;
    }
    const uint64_t start = monotonic_us();

    size_t written = 0;
    while (written < len) {
        int transferred = 0;
        const int rc = libusb_bulk_transfer(
            handle_, ep_out_, const_cast<uint8_t*>(frame + written),
            static_cast<int>(len - written), &transferred, FTDI_WRITE_TIMEOUT_MS);
        if (rc != 0) { error = "bulk write failed: " + error_name(rc); return false; }
        if (transferred <= 0) { error = "bulk write made no progress"; return false; }
        written += static_cast<size_t>(transferred);
    }

    uint8_t got[MAX_LOOPBACK_FRAME];
    if (!read_echo(frame, len, got, error)) return false;
    if (latency_us) *latency_us = monotonic_us() - start;
    return true;
}

// ── LoopbackWorker ────────────────────────────────────────────────────────────
LoopbackWorker::LoopbackWorker(LoopbackTransport& device, int cadence_hz, uint8_t frame_id)
    : device_(device),
      cadence_hz_(cadence_hz > 0 ? cadence_hz : 1),
      frame_id_(frame_id)
{}

LoopbackWorker::~LoopbackWorker()
{
    stop();
}

void LoopbackWorker::start()
{
    if (thread_.joinable()) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = false;
        fatal_ = false;
        fatal_error_.clear();
        for (int i = 0; i < 2; i++) windows_[i] = ftdi_proto::WindowStats{};
    }
    window_ = 0;
    thread_ = std::thread(&LoopbackWorker::run, this);
}

void LoopbackWorker::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void LoopbackWorker::set_window(int index)
{
    if (index < 0 || index > 1) return;
    window_.store(index);
}

ftdi_proto::WindowStats LoopbackWorker::snapshot(int index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < 0 || index > 1) return ftdi_proto::WindowStats{};
    return windows_[index];
}

std::string LoopbackWorker::fatal_error() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fatal_error_;
}

void LoopbackWorker::run()
{
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / cadence_hz_);
    auto next_tick = clock::now();
    uint16_t seq = 0;
    uint32_t nonce = 0x9E3779B9u;
    uint8_t payload[32];

    while (true) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stopping_) return;
        }

        for (size_t i = 0; i < sizeof(payload); i++) {
            payload[i] = static_cast<uint8_t>((nonce >> (8 * (i % 4))) ^ (seq + i));
        }
        uint8_t frame[MAX_LOOPBACK_FRAME];
        const int frame_len = ftdi_proto::build_loopback_frame(
            frame, sizeof(frame), frame_id_, seq, nonce, payload, sizeof(payload));

        const int bucket = window_.load();
        bool ok = false;
        std::string error;
        uint64_t latency_us = 0;
        if (frame_len > 0) {
            ok = device_.exchange(frame, static_cast<size_t>(frame_len), error,
                                  &latency_us);
        } else {
            error = "frame build failed";
        }

        bool fatal_now = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ftdi_proto::WindowStats& stats = windows_[bucket < 0 || bucket > 1 ? 0 : bucket];
            stats.attempts++;
            if (ok) {
                stats.matches++;
                stats.total_latency_us += latency_us;
                if (latency_us > stats.max_latency_us) stats.max_latency_us = latency_us;
            } else {
                stats.errors++;
                if (!fatal_.load()) {
                    fatal_.store(true);
                    fatal_error_ = error;
                }
                fatal_now = true;
            }
        }
        // A fatal loopback error stops the workload immediately so the first
        // failure is not buried under a full window of repeated errors. There is
        // no retry: the failing frame is never resent.
        if (fatal_now) return;

        seq++;
        nonce = nonce * 1664525u + 1013904223u;

        next_tick += period;
        const auto now = clock::now();
        if (next_tick < now) next_tick = now + period;  // never burst to catch up
        std::unique_lock<std::mutex> lock(mutex_);
        if (cv_.wait_until(lock, next_tick, [this] { return stopping_; })) return;
    }
}

} // namespace ft232
