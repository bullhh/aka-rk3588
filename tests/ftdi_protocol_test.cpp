// Host unit test for the pure FT232R loopback helpers. No hardware, no libusb.
#include "usb/ftdi_protocol.hpp"
#include "usb/loopback_transport.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (condition) return true;
    std::fprintf(stderr, "FAIL: %s\n", message);
    return false;
}

ftdi_proto::Ft232Candidate cand(int bus, int address, const char* serial,
                                uint16_t vid = ftdi_proto::FTDI_VID,
                                uint16_t pid = ftdi_proto::FT232R_PID) {
    ftdi_proto::Ft232Candidate c;
    c.vid = vid;
    c.pid = pid;
    c.bus = bus;
    c.address = address;
    c.serial = serial ? serial : "";
    return c;
}

} // namespace

int main() {
    bool ok = true;

    // ── FT232R baud divisor ───────────────────────────────────────────────────
    ok = expect(ftdi_proto::ft232r_divisor(1000000) == 3,
                "1 Mbps must map to divisor 3") && ok;
    ok = expect(ftdi_proto::ft232r_divisor(115200) == 26,
                "115200 must map to divisor 26") && ok;
    ok = expect(ftdi_proto::ft232r_divisor(9600) == 312,
                "9600 must map to divisor 312") && ok;
    ok = expect(ftdi_proto::ft232r_divisor(2000000) == 1,
                "2 Mbps must map to divisor 1") && ok;
    ok = expect(ftdi_proto::ft232r_divisor(3000000) == 0,
                "3 Mbps must map to divisor 0") && ok;

    // ── Per-USB-IN-packet status stripping ────────────────────────────────────
    {
        const uint8_t packet[] = {0x01, 0x60, 'A', 'B', 'C'};
        uint8_t out[8] = {};
        const int n = ftdi_proto::strip_status_bytes(packet, 5, out, 8);
        ok = expect(n == 3 && out[0] == 'A' && out[1] == 'B' && out[2] == 'C',
                    "two status bytes must be stripped without dropping payload") && ok;

        const int status_only = ftdi_proto::strip_status_bytes(packet, 2, out, 8);
        ok = expect(status_only == 0,
                    "a status-only packet must yield zero data bytes") && ok;

        const int short_packet = ftdi_proto::strip_status_bytes(packet, 1, out, 8);
        ok = expect(short_packet == -1,
                    "a packet with fewer than two status bytes is malformed") && ok;

        const int tiny_out = ftdi_proto::strip_status_bytes(packet, 5, out, 1);
        ok = expect(tiny_out == -1, "undersized output buffer must be rejected") && ok;
    }

    // ── Feetech frame format ──────────────────────────────────────────────────
    {
        const uint8_t params[] = {0x10, 0x20, 0x30};
        uint8_t frame[16] = {};
        const int len = ftdi_proto::build_feetech_frame(frame, sizeof(frame), 0x07,
                                                        0x03, params, 3);
        ok = expect(len == 9, "frame length must be 6 + params") && ok;
        ok = expect(frame[0] == 0xFF && frame[1] == 0xFF && frame[2] == 0x07 &&
                        frame[3] == 0x05 && frame[4] == 0x03,
                    "frame header/id/length/instruction must be well formed") && ok;
        ok = expect(frame[5] == 0x10 && frame[6] == 0x20 && frame[7] == 0x30,
                    "params must be copied verbatim") && ok;
        const uint8_t expected =
            ftdi_proto::feetech_checksum(0x07, 0x05, 0x03, params, 3);
        ok = expect(frame[8] == expected, "frame checksum must match") && ok;
        const uint32_t sum = 0x07 + 0x05 + 0x03 + 0x10 + 0x20 + 0x30;
        ok = expect(frame[8] == static_cast<uint8_t>(~(sum & 0xFF)),
                    "checksum must be the inverted low byte sum") && ok;

        uint8_t too_small[8] = {};
        ok = expect(ftdi_proto::build_feetech_frame(too_small, sizeof(too_small), 0x07,
                                                    0x03, params, 3) == -1,
                    "a frame that does not fit must be rejected") && ok;
    }

    // ── Loopback exchange frame ───────────────────────────────────────────────
    {
        const uint8_t payload[4] = {0xA1, 0xB2, 0xC3, 0xD4};
        uint8_t frame[64] = {};
        const int len = ftdi_proto::build_loopback_frame(
            frame, sizeof(frame), 0x01, 0x1234, 0xDEADBEEF, payload, 4);
        ok = expect(len == 17, "loopback frame length must be 13 + payload") && ok;
        ok = expect(frame[2] == 0x01 && frame[3] == 0x0D &&
                        frame[4] == ftdi_proto::FEETECH_INSTRUCTION_PING,
                    "loopback header must use the safe PING instruction") && ok;
        ok = expect(frame[4] != ftdi_proto::FEETECH_INSTRUCTION_WRITE,
                    "loopback frames must never carry a Feetech WRITE "
                    "instruction") && ok;
        ok = expect(frame[2] != ftdi_proto::FEETECH_BROADCAST_ID,
                    "loopback frames must not use the broadcast id") && ok;
        ok = expect(ftdi_proto::LOOPBACK_INSTRUCTION !=
                        ftdi_proto::FEETECH_INSTRUCTION_WRITE &&
                        ftdi_proto::LOOPBACK_INSTRUCTION ==
                            ftdi_proto::FEETECH_INSTRUCTION_PING,
                    "the loopback instruction constant must stay PING, not "
                    "WRITE") && ok;
        ok = expect(frame[5] == 0x34 && frame[6] == 0x12,
                    "sequence must be stored little-endian") && ok;
        ok = expect(frame[7] == 0xEF && frame[8] == 0xBE && frame[9] == 0xAD &&
                        frame[10] == 0xDE,
                    "nonce must be stored little-endian") && ok;
        ok = expect(frame[11] == 4 &&
                        frame[12] == 0xA1 && frame[13] == 0xB2 &&
                        frame[14] == 0xC3 && frame[15] == 0xD4,
                    "payload length and bytes must be carried") && ok;
        ok = expect(ftdi_proto::loopback_frame_seq(frame, len) == 0x1234,
                    "sequence must be recoverable from the frame") && ok;
        ok = expect(ftdi_proto::loopback_frame_seq(frame, 3) == -1,
                    "a truncated frame has no recoverable sequence") && ok;
        frame[0] = 0x00;
        ok = expect(ftdi_proto::loopback_frame_seq(frame, len) == -1,
                    "a bad header must be rejected") && ok;
    }

    // ── Device selection ──────────────────────────────────────────────────────
    {
        ftdi_proto::Ft232Candidate selected;
        std::vector<ftdi_proto::Ft232Candidate> none = {
            cand(1, 2, "OTHER", 0x1a86, 0x55d3)};
        ok = expect(ftdi_proto::select_ft232(none, "", selected) ==
                        ftdi_proto::SelectResult::NONE,
                    "non-FTDI devices must not be selected") && ok;

        std::vector<ftdi_proto::Ft232Candidate> single = {cand(1, 4, "SN0001")};
        ok = expect(ftdi_proto::select_ft232(single, "", selected) ==
                        ftdi_proto::SelectResult::OK &&
                        selected.serial == "SN0001",
                    "a single FT232R must be selected without a serial") && ok;

        std::vector<ftdi_proto::Ft232Candidate> multiple = {
            cand(1, 4, "AAA"), cand(1, 5, "BBB")};
        ok = expect(ftdi_proto::select_ft232(multiple, "", selected) ==
                        ftdi_proto::SelectResult::MULTIPLE,
                    "two FT232R devices require an explicit serial") && ok;
        ok = expect(ftdi_proto::select_ft232(multiple, "BBB", selected) ==
                        ftdi_proto::SelectResult::OK &&
                        selected.serial == "BBB",
                    "a serial selector must pick the matching adapter") && ok;
        ok = expect(ftdi_proto::select_ft232(multiple, "CCC", selected) ==
                        ftdi_proto::SelectResult::SERIAL_NOT_FOUND,
                    "an unknown serial must not fall back to another adapter") && ok;
        std::vector<ftdi_proto::Ft232Candidate> duplicate = {
            cand(1, 4, "AAA"), cand(1, 5, "AAA")};
        ok = expect(ftdi_proto::select_ft232(duplicate, "AAA", selected) ==
                        ftdi_proto::SelectResult::SERIAL_MULTIPLE,
                    "ambiguous serials must be rejected") && ok;

        // A permission-denied libusb serial read must not look like "not found"
        // or true ambiguity: auto mode needs this result to allow the sysfs/TTY
        // backend to redo strict serial selection.
        ftdi_proto::Ft232Candidate unreadable = cand(1, 6, "");
        unreadable.serial_unreadable = true;
        std::vector<ftdi_proto::Ft232Candidate> unreadable_only = {unreadable};
        ok = expect(ftdi_proto::select_ft232(unreadable_only, "SN0001", selected) ==
                        ftdi_proto::SelectResult::SERIAL_UNREADABLE,
                    "an unreadable serial must be reported as unreadable") && ok;

        std::vector<ftdi_proto::Ft232Candidate> mixed_readability = {
            cand(1, 4, "SN0001"), unreadable};
        ok = expect(ftdi_proto::select_ft232(mixed_readability, "SN0001", selected) ==
                        ftdi_proto::SelectResult::SERIAL_UNREADABLE,
                    "an unreadable peer makes serial uniqueness unprovable") && ok;
    }

    // ── TTY fallback policy ───────────────────────────────────────────────────
    {
        ok = expect(ft232::may_fall_back_to_tty(
                        ft232::OpenFailure::SERIAL_UNREADABLE),
                    "an unreadable raw-libusb serial must allow the TTY retry") && ok;
        ok = expect(ft232::may_fall_back_to_tty(
                        ft232::OpenFailure::ACCESS_DENIED),
                    "a denied raw-libusb open must allow the TTY retry") && ok;
        ok = expect(!ft232::may_fall_back_to_tty(
                        ft232::OpenFailure::AMBIGUOUS),
                    "true raw-libusb ambiguity must not fall back") && ok;
        ok = expect(!ft232::may_fall_back_to_tty(
                        ft232::OpenFailure::NO_DEVICE),
                    "a missing device must not fall back") && ok;
    }

    // ── Linux TTY candidate path guard ────────────────────────────────────────
    {
        ok = expect(ftdi_proto::is_allowed_tty_path("/dev/ttyUSB0"),
                    "/dev/ttyUSB0 must be allowed") && ok;
        ok = expect(ftdi_proto::is_allowed_tty_path("/dev/ttyUSB12"),
                    "/dev/ttyUSB12 must be allowed") && ok;
        ok = expect(!ftdi_proto::is_allowed_tty_path("/dev/ttyS6"),
                    "/dev/ttyS6 must never be allowed") && ok;
        ok = expect(!ftdi_proto::is_allowed_tty_path("/dev/ttyACM0"),
                    "non-FTDI ttyACM ports must never be allowed") && ok;
        ok = expect(!ftdi_proto::is_allowed_tty_path("/dev/ttyUSB"),
                    "a bare /dev/ttyUSB prefix must be rejected") && ok;
        ok = expect(!ftdi_proto::is_allowed_tty_path("/dev/ttyUSB0x"),
                    "trailing junk must be rejected") && ok;
    }

    // ── sysfs attribute trimming ──────────────────────────────────────────────
    {
        ok = expect(ftdi_proto::trim_attribute_value("0403\n") == "0403",
                    "trailing newline must be trimmed") && ok;
        ok = expect(ftdi_proto::trim_attribute_value("  BG02M9PI \r\n") == "BG02M9PI",
                    "surrounding whitespace must be trimmed") && ok;
        ok = expect(ftdi_proto::trim_attribute_value("") == "",
                    "an empty value must stay empty") && ok;
    }

    // ── Echo classification (shared by the USB and TTY transports) ────────────
    {
        const uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};
        uint8_t frame[64] = {};
        const int len = ftdi_proto::build_loopback_frame(
            frame, sizeof(frame), 0x01, 7, 0x11223344u, payload, 4);
        ok = expect(len > 0, "echo fixture must build") && ok;

        ok = expect(ftdi_proto::classify_echo(frame, len, frame, len) ==
                        ftdi_proto::EchoResult::MATCH,
                    "an identical echo must match") && ok;

        ok = expect(ftdi_proto::classify_echo(frame, len, frame, len - 1) ==
                        ftdi_proto::EchoResult::SHORT,
                    "a short echo must be reported short") && ok;
        ok = expect(ftdi_proto::classify_echo(frame, len, frame, len + 1) ==
                        ftdi_proto::EchoResult::LONG,
                    "a long echo must be reported long") && ok;

        uint8_t corrupt[64] = {};
        memcpy(corrupt, frame, len);
        corrupt[len - 1] ^= 0xFF;
        ok = expect(ftdi_proto::classify_echo(frame, len, corrupt, len) ==
                        ftdi_proto::EchoResult::CORRUPT,
                    "a same-length mismatch must be reported corrupt") && ok;

        uint8_t stale[64] = {};
        const int stale_len = ftdi_proto::build_loopback_frame(
            stale, sizeof(stale), 0x01, 6, 0x11223344u, payload, 4);
        ok = expect(ftdi_proto::classify_echo(frame, len, stale, stale_len) ==
                        ftdi_proto::EchoResult::STALE,
                    "an older sequence must be reported stale") && ok;

        uint8_t reordered[64] = {};
        const int reordered_len = ftdi_proto::build_loopback_frame(
            reordered, sizeof(reordered), 0x01, 8, 0x11223344u, payload, 4);
        ok = expect(ftdi_proto::classify_echo(frame, len, reordered, reordered_len) ==
                        ftdi_proto::EchoResult::REORDERED,
                    "a newer sequence must be reported reordered") && ok;
    }

    // ── Verdict ───────────────────────────────────────────────────────────────
    {
        ok = expect(ftdi_proto::MIN_TX_PER_WINDOW_FLOOR == 160,
                    "the fixed per-window exchange floor must stay at 160") && ok;

        ftdi_proto::VerdictInput input;
        input.min_fps = 28.0;
        input.min_tx_per_window = ftdi_proto::MIN_TX_PER_WINDOW_FLOOR;
        for (int i = 0; i < 2; i++) {
            input.perf[i].elapsed_s = 10.0;
            input.perf[i].processed = 300;
            input.loopback[i].attempts = 200;
            input.loopback[i].matches = 200;
            input.loopback[i].errors = 0;
        }
        ok = expect(ftdi_proto::evaluate_verdict(input).pass,
                    "a clean two-window run must pass") && ok;

        ftdi_proto::VerdictInput short_window = input;
        short_window.perf[0].elapsed_s = 9.9;
        ok = expect(ftdi_proto::evaluate_verdict(short_window).reason ==
                        "window_shorter_than_10s",
                    "a window under 10s must fail") && ok;

        ftdi_proto::VerdictInput no_frames = input;
        no_frames.perf[1].processed = 0;
        ok = expect(ftdi_proto::evaluate_verdict(no_frames).reason ==
                        "no_processed_frames",
                    "an empty window must fail") && ok;

        ftdi_proto::VerdictInput loopback_error = input;
        loopback_error.loopback[1].errors = 1;
        ok = expect(ftdi_proto::evaluate_verdict(loopback_error).reason ==
                        "loopback_errors",
                    "any loopback error must fail") && ok;

        ftdi_proto::VerdictInput few_tx = input;
        few_tx.loopback[0].matches = ftdi_proto::MIN_TX_PER_WINDOW_FLOOR - 1;
        ok = expect(ftdi_proto::evaluate_verdict(few_tx).reason ==
                        "loopback_below_min_tx",
                    "too few successful exchanges must fail") && ok;

        ftdi_proto::VerdictInput slow = input;
        slow.min_fps = 31.0;
        ok = expect(ftdi_proto::evaluate_verdict(slow).reason ==
                        "fps_below_threshold",
                    "fps under the threshold must fail") && ok;

        ftdi_proto::VerdictInput bad_threshold = input;
        bad_threshold.min_fps = 0.0;
        ok = expect(ftdi_proto::evaluate_verdict(bad_threshold).reason ==
                        "invalid_min_fps",
                    "a non-positive threshold must be rejected") && ok;
    }

    return ok ? 0 : 1;
}
