#include "usb/ftdi_protocol.hpp"

#include <cmath>
#include <cstring>

namespace ftdi_proto {

uint16_t ft232r_divisor(int baudrate)
{
    if (baudrate <= 0) return 0;
    if (baudrate >= 3000000) return 0;
    if (baudrate == 2000000) return 1;
    // FT232R: divisor = 3000000 / baudrate, using integer truncation (floor).
    // 1 Mbps maps to 3 and 9600 to 312, matching the datasheet table. This is
    // deliberately not rounded to the nearest achievable divisor.
    const int divisor = 3000000 / baudrate;
    return static_cast<uint16_t>(divisor);
}

int strip_status_bytes(const uint8_t* in, int in_len, uint8_t* out, int out_cap)
{
    if (!in || in_len < FTDI_STATUS_BYTES) return -1;
    const int data_len = in_len - FTDI_STATUS_BYTES;
    if (data_len > out_cap) return -1;
    if (data_len > 0) {
        if (!out) return -1;
        memcpy(out, in + FTDI_STATUS_BYTES, static_cast<size_t>(data_len));
    }
    return data_len;
}

uint8_t feetech_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                         const uint8_t* params, size_t params_len)
{
    uint32_t sum = static_cast<uint32_t>(id) + length + instruction;
    for (size_t i = 0; i < params_len; i++) sum += params[i];
    return static_cast<uint8_t>(~(sum & 0xFFu) & 0xFFu);
}

int build_feetech_frame(uint8_t* out, size_t cap, uint8_t id, uint8_t instruction,
                        const uint8_t* params, size_t params_len)
{
    if (!out || params_len > 250) return -1;
    const size_t total = 6 + params_len;
    if (cap < total) return -1;
    const uint8_t length = static_cast<uint8_t>(params_len + 2);
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = id;
    out[3] = length;
    out[4] = instruction;
    for (size_t i = 0; i < params_len; i++) out[5 + i] = params[i];
    out[5 + params_len] =
        feetech_checksum(id, length, instruction, params, params_len);
    return static_cast<int>(total);
}

int build_loopback_frame(uint8_t* out, size_t cap, uint8_t id, uint16_t seq,
                         uint32_t nonce, const uint8_t* payload, size_t payload_len)
{
    if (payload_len > 40) return -1;
    uint8_t params[7 + 40];
    params[0] = static_cast<uint8_t>(seq & 0xFF);
    params[1] = static_cast<uint8_t>((seq >> 8) & 0xFF);
    params[2] = static_cast<uint8_t>(nonce & 0xFF);
    params[3] = static_cast<uint8_t>((nonce >> 8) & 0xFF);
    params[4] = static_cast<uint8_t>((nonce >> 16) & 0xFF);
    params[5] = static_cast<uint8_t>((nonce >> 24) & 0xFF);
    params[6] = static_cast<uint8_t>(payload_len & 0xFF);
    for (size_t i = 0; i < payload_len; i++) {
        params[7 + i] = payload ? payload[i] : static_cast<uint8_t>(i);
    }
    // Never send a Feetech WRITE from the loopback workload: a miswired adapter
    // could otherwise move a real servo. PING only elicits a response, which the
    // strict byte-for-byte echo comparison rejects without changing device state.
    return build_feetech_frame(out, cap, id, LOOPBACK_INSTRUCTION, params,
                               7 + payload_len);
}

int loopback_frame_seq(const uint8_t* frame, size_t len)
{
    if (!frame || len < static_cast<size_t>(LOOPBACK_MIN_LEN)) return -1;
    if (frame[0] != 0xFF || frame[1] != 0xFF) return -1;
    const uint8_t length = frame[3];
    if (static_cast<size_t>(length) + 4 != len) return -1;
    return static_cast<int>(frame[LOOPBACK_SEQ_OFFSET]) |
           (static_cast<int>(frame[LOOPBACK_SEQ_OFFSET + 1]) << 8);
}

EchoResult classify_echo(const uint8_t* expected, size_t expected_len,
                         const uint8_t* got, size_t got_len)
{
    if (!expected || !got) return EchoResult::CORRUPT;
    if (got_len == expected_len && memcmp(got, expected, expected_len) == 0)
        return EchoResult::MATCH;
    if (got_len < expected_len) return EchoResult::SHORT;
    if (got_len > expected_len) return EchoResult::LONG;
    const int expected_seq = loopback_frame_seq(expected, expected_len);
    const int got_seq = loopback_frame_seq(got, got_len);
    if (expected_seq >= 0 && got_seq >= 0) {
        if (got_seq < expected_seq) return EchoResult::STALE;
        if (got_seq > expected_seq) return EchoResult::REORDERED;
    }
    return EchoResult::CORRUPT;
}

const char* echo_result_string(EchoResult result)
{
    switch (result) {
    case EchoResult::MATCH:     return "match";
    case EchoResult::SHORT:     return "short";
    case EchoResult::LONG:      return "long";
    case EchoResult::CORRUPT:   return "corrupt";
    case EchoResult::STALE:     return "stale";
    case EchoResult::REORDERED: return "reordered";
    }
    return "unknown";
}

bool is_allowed_tty_path(const std::string& path)
{
    static const char prefix[] = "/dev/ttyUSB";
    const size_t prefix_len = sizeof(prefix) - 1;
    if (path.size() <= prefix_len) return false;
    if (path.compare(0, prefix_len, prefix) != 0) return false;
    for (size_t i = prefix_len; i < path.size(); i++) {
        if (path[i] < '0' || path[i] > '9') return false;
    }
    return true;
}

std::string trim_attribute_value(const std::string& value)
{
    size_t begin = 0;
    while (begin < value.size() &&
           (value[begin] == ' ' || value[begin] == '\t' ||
            value[begin] == '\r' || value[begin] == '\n')) {
        begin++;
    }
    size_t end = value.size();
    while (end > begin &&
           (value[end - 1] == ' ' || value[end - 1] == '\t' ||
            value[end - 1] == '\r' || value[end - 1] == '\n')) {
        end--;
    }
    return value.substr(begin, end - begin);
}

SelectResult select_ft232(const std::vector<Ft232Candidate>& candidates,
                          const std::string& serial_selector,
                          Ft232Candidate& out)
{
    std::vector<Ft232Candidate> matching;
    for (size_t i = 0; i < candidates.size(); i++) {
        if (candidates[i].vid == FTDI_VID && candidates[i].pid == FT232R_PID)
            matching.push_back(candidates[i]);
    }
    if (matching.empty()) return SelectResult::NONE;

    if (serial_selector.empty()) {
        if (matching.size() > 1) return SelectResult::MULTIPLE;
        out = matching[0];
        return SelectResult::OK;
    }

    std::vector<Ft232Candidate> serial_match;
    bool any_unreadable = false;
    for (size_t i = 0; i < matching.size(); i++) {
        if (matching[i].serial_unreadable) {
            any_unreadable = true;
            continue;
        }
        if (matching[i].serial == serial_selector) serial_match.push_back(matching[i]);
    }
    if (serial_match.size() > 1) return SelectResult::SERIAL_MULTIPLE;
    // A readable match cannot be proven unique while another candidate's serial
    // is unknown: report it as unreadable rather than pretending the selector was
    // simply not found or that the bus is truly ambiguous.
    if (any_unreadable) return SelectResult::SERIAL_UNREADABLE;
    if (serial_match.empty()) return SelectResult::SERIAL_NOT_FOUND;
    out = serial_match[0];
    return SelectResult::OK;
}

const char* select_result_string(SelectResult result)
{
    switch (result) {
    case SelectResult::OK:                return "ok";
    case SelectResult::NONE:              return "no_0403_6001_device";
    case SelectResult::MULTIPLE:          return "multiple_0403_6001_devices";
    case SelectResult::SERIAL_NOT_FOUND:  return "serial_not_found";
    case SelectResult::SERIAL_MULTIPLE:   return "serial_multiple";
    case SelectResult::SERIAL_UNREADABLE: return "serial_unreadable";
    }
    return "unknown";
}

VerdictResult evaluate_verdict(const VerdictInput& input)
{
    VerdictResult result;
    if (!(input.min_fps > 0.0)) {
        result.reason = "invalid_min_fps";
        return result;
    }
    for (int i = 0; i < 2; i++) {
        if (input.perf[i].elapsed_s < 10.0) {
            result.reason = "window_shorter_than_10s";
            return result;
        }
        if (input.perf[i].processed == 0) {
            result.reason = "no_processed_frames";
            return result;
        }
        if (input.loopback[i].errors != 0) {
            result.reason = "loopback_errors";
            return result;
        }
        if (input.loopback[i].matches < input.min_tx_per_window) {
            result.reason = "loopback_below_min_tx";
            return result;
        }
    }
    const double total_elapsed = input.perf[0].elapsed_s + input.perf[1].elapsed_s;
    const uint64_t total_processed = input.perf[0].processed + input.perf[1].processed;
    const double fps = total_elapsed > 0.0 ? total_processed / total_elapsed : 0.0;
    if (!std::isfinite(fps) || fps < input.min_fps) {
        result.reason = "fps_below_threshold";
        return result;
    }
    result.pass = true;
    result.reason = "ok";
    return result;
}

} // namespace ftdi_proto
