#ifndef FTDI_PROTOCOL_HPP
#define FTDI_PROTOCOL_HPP

// Pure, host-testable helpers for the FT232R loopback workload.
//
// Nothing here depends on libusb so the framing, status stripping, device
// selection and verdict rules can be exercised by a plain host unit test.
// The libusb transport lives in ft232_loopback.{hpp,cpp}.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ftdi_proto {

// ── FTDI USB identity ─────────────────────────────────────────────────────────
static const uint16_t FTDI_VID   = 0x0403;
static const uint16_t FT232R_PID = 0x6001;

// ── Feetech instruction bytes ─────────────────────────────────────────────────
// The loopback workload must never issue WRITE: if the FT232R is accidentally
// wired to a real Feetech bus, a WRITE frame could change a servo register or
// cause motion. PING has no register side effects, so a real servo can at most
// answer (and make the strict byte-for-byte echo comparison fail) without
// changing state.
enum : uint8_t {
    FEETECH_INSTRUCTION_PING  = 0x01,
    FEETECH_INSTRUCTION_WRITE = 0x03,
};
enum : uint8_t {
    FEETECH_BROADCAST_ID = 0xFE,
};
static const uint8_t LOOPBACK_INSTRUCTION = FEETECH_INSTRUCTION_PING;
static_assert(LOOPBACK_INSTRUCTION != FEETECH_INSTRUCTION_WRITE,
              "loopback frames must never use the Feetech WRITE instruction");

// ── FTDI vendor control requests (bRequest) ───────────────────────────────────
enum : uint8_t {
    FTDI_REQ_RESET              = 0x00,
    FTDI_REQ_SET_MODEM_CTRL     = 0x01,
    FTDI_REQ_SET_FLOW_CTRL      = 0x02,
    FTDI_REQ_SET_BAUDRATE       = 0x03,
    FTDI_REQ_SET_DATA           = 0x04,
    FTDI_REQ_SET_LATENCY_TIMER  = 0x09,
    FTDI_REQ_GET_LATENCY_TIMER  = 0x0A,
};

// FTDI_REQ_RESET wValue selectors.
enum : uint16_t {
    FTDI_RESET_SIO       = 0,  // reset the whole port (needed after open)
    FTDI_RESET_PURGE_RX  = 1,
    FTDI_RESET_PURGE_TX  = 2,
};

// FTDI_REQ_SET_DATA wValue for 8 data bits / no parity / 1 stop bit.
static const uint16_t FTDI_DATA_8N1 = 0x0008;
// Smallest FT232R latency timer that still lets small frames flush promptly.
static const uint16_t FTDI_LATENCY_TIMER_MS = 2;

// ── Baud rate ─────────────────────────────────────────────────────────────────
// FT232R reference clock is 3 MHz. The value returned is the divisor written to
// FTDI_REQ_SET_BAUDRATE as wValue (low 16 bits) and wIndex (high 16 bits).
// 0 selects 3 Mbaud and 1 selects 2 Mbaud, matching the FT232R datasheet.
// Non-standard rates use integer truncation, not rounding to the nearest divisor.
uint16_t ft232r_divisor(int baudrate);

// ── Per-USB-IN-packet FTDI status bytes ───────────────────────────────────────
// The FT232R prepends two status bytes to the data of every USB IN packet
// (modem status + line status). They must be removed before the payload can be
// interpreted, and a packet that carries only those two bytes yields no data.
static const int FTDI_STATUS_BYTES = 2;

// Strip the two status bytes from one bulk IN result.
// Returns the number of data bytes copied into `out` (0 for a status-only
// packet), or -1 when the packet holds fewer than two status bytes or `out` is
// too small.
int strip_status_bytes(const uint8_t* in, int in_len, uint8_t* out, int out_cap);

// ── Feetech frame format ──────────────────────────────────────────────────────
// 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
// LENGTH counts INSTRUCTION + PARAM... + CHECKSUM.
uint8_t feetech_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                         const uint8_t* params, size_t params_len);

// Build a Feetech command frame. Returns the total frame length, or -1 when the
// argument combination does not fit in `cap`.
int build_feetech_frame(uint8_t* out, size_t cap, uint8_t id, uint8_t instruction,
                        const uint8_t* params, size_t params_len);

// ── Loopback exchange frame ───────────────────────────────────────────────────
// PARAM... = SEQ_LO SEQ_HI NONCE0..3 PLEN PAYLOAD...
// The changing sequence/nonce let the echo be matched exactly, so stale,
// reordered, duplicated and corrupt echoes all fail the explicit comparison.
// The instruction is always LOOPBACK_INSTRUCTION (Feetech PING), never WRITE.
static const int LOOPBACK_SEQ_OFFSET = 5;  // first PARAM byte
static const int LOOPBACK_MIN_LEN    = 13; // header + seq + nonce + plen + chk

int build_loopback_frame(uint8_t* out, size_t cap, uint8_t id, uint16_t seq,
                         uint32_t nonce, const uint8_t* payload, size_t payload_len);

// Sequence number carried by a loopback frame, or -1 when the framing is
// malformed.
int loopback_frame_seq(const uint8_t* frame, size_t len);

// ── Echo classification ───────────────────────────────────────────────────────
// Shared by the libusb and Linux TTY transports so both apply identical strict
// byte-for-byte verification.
enum class EchoResult {
    MATCH,
    SHORT,
    LONG,
    CORRUPT,
    STALE,
    REORDERED,
};

EchoResult classify_echo(const uint8_t* expected, size_t expected_len,
                         const uint8_t* got, size_t got_len);
const char* echo_result_string(EchoResult result);

// ── Device selection ──────────────────────────────────────────────────────────
struct Ft232Candidate {
    uint16_t    vid = 0;
    uint16_t    pid = 0;
    int         bus = 0;
    int         address = 0;
    std::string serial;
    // True when the transport could not read the serial (for example the raw
    // libusb string descriptor is permission-denied). Such a candidate cannot
    // prove a serial match, so a serial selector must not be treated as if the
    // device were absent; auto mode may retry the same device through sysfs.
    bool        serial_unreadable = false;
    // Set by the Linux TTY transport to the resolved /dev/ttyUSBx node. Empty
    // for the raw libusb transport.
    std::string device_path;
};

enum class SelectResult {
    OK,
    NONE,             // no 0403:6001 device present
    MULTIPLE,         // more than one device and no serial selector
    SERIAL_NOT_FOUND, // selector given, no device carries that serial
    SERIAL_MULTIPLE,  // selector given, several devices carry that serial
    SERIAL_UNREADABLE, // selector given, but a serial could not be read
};

// Select exactly one FT232R. With an empty selector there must be exactly one
// 0403:6001 device; a non-empty selector must match exactly one readable serial.
// An unreadable serial returns SERIAL_UNREADABLE so the caller can decide whether
// a sysfs-based fallback is safe instead of mistaking it for true ambiguity.
SelectResult select_ft232(const std::vector<Ft232Candidate>& candidates,
                          const std::string& serial_selector,
                          Ft232Candidate& out);

const char* select_result_string(SelectResult result);

// True only for "/dev/ttyUSB" followed by one or more digits. The FT232R TTY
// fallback refuses anything else, so it can never touch /dev/ttyS6.
bool is_allowed_tty_path(const std::string& path);

// Trim leading/trailing whitespace from a sysfs attribute value.
std::string trim_attribute_value(const std::string& value);

// ── Verdict ───────────────────────────────────────────────────────────────────
struct WindowStats {
    uint64_t attempts = 0;
    uint64_t matches = 0;
    uint64_t errors = 0;
    uint64_t total_latency_us = 0;
    uint64_t max_latency_us = 0;

    double avg_latency_ms() const {
        return matches > 0
            ? static_cast<double>(total_latency_us) / matches / 1000.0
            : 0.0;
    }
};

struct WindowPerf {
    double   elapsed_s = 0.0;
    uint64_t processed = 0;

    double fps() const { return elapsed_s > 0.0 ? processed / elapsed_s : 0.0; }
};

struct VerdictInput {
    double      min_fps = 0.0;
    uint64_t    min_tx_per_window = 0;
    WindowPerf  perf[2];
    WindowStats loopback[2];
};

struct VerdictResult {
    bool        pass = false;
    std::string reason;
};

// Fixed CI floor: at the default 20 Hz cadence a full 10s window must produce at
// least 160 successful exchanges. The application applies this as a hard floor
// (an env override may only raise it), and run_vision_usb_ci_once.sh applies the
// matching total floor of 2 * 160 = 320.
static const uint64_t MIN_TX_PER_WINDOW_FLOOR = 160;

// Pass requires: two >=10s windows with positive processed counts, an aggregate
// FPS at or above the threshold, zero loopback errors and at least
// min_tx_per_window successful exchanges in every window.
VerdictResult evaluate_verdict(const VerdictInput& input);

} // namespace ftdi_proto

#endif // FTDI_PROTOCOL_HPP
