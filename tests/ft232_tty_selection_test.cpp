// Focused host test for the Linux FT232R TTY transport selection rules.
//
// It never opens a device: it exercises the pure selection logic and, on a
// Linux host, the sysfs enumeration invariant that every candidate is a
// "/dev/ttyUSBx" node of a 0403:6001 device (so /dev/ttyS6 can never leak in).
// Hardware transport verification stays with the board acceptance.
#include "usb/ft232_tty.hpp"

#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (condition) return true;
    std::fprintf(stderr, "FAIL: %s\n", message);
    return false;
}

ftdi_proto::Ft232Candidate tty_candidate(const char* path, const char* serial) {
    ftdi_proto::Ft232Candidate candidate;
    candidate.vid = ftdi_proto::FTDI_VID;
    candidate.pid = ftdi_proto::FT232R_PID;
    candidate.serial = serial ? serial : "";
    candidate.device_path = path ? path : "";
    return candidate;
}

} // namespace

int main() {
    bool ok = true;

    // ── Selection keeps the resolved tty node and handles ambiguity ──────────
    {
        ftdi_proto::Ft232Candidate selected;

        std::vector<ftdi_proto::Ft232Candidate> none;
        ok = expect(ftdi_proto::select_ft232(none, "", selected) ==
                        ftdi_proto::SelectResult::NONE,
                    "no FT232R tty must be reported as none") && ok;

        std::vector<ftdi_proto::Ft232Candidate> single = {
            tty_candidate("/dev/ttyUSB0", "SN0001")};
        ok = expect(ftdi_proto::select_ft232(single, "", selected) ==
                        ftdi_proto::SelectResult::OK &&
                        selected.device_path == "/dev/ttyUSB0",
                    "a single FT232R tty must select with its node") && ok;

        std::vector<ftdi_proto::Ft232Candidate> multiple = {
            tty_candidate("/dev/ttyUSB0", "AAA"),
            tty_candidate("/dev/ttyUSB1", "BBB")};
        ok = expect(ftdi_proto::select_ft232(multiple, "", selected) ==
                        ftdi_proto::SelectResult::MULTIPLE,
                    "two FT232R ttys require a serial selector") && ok;
        ok = expect(ftdi_proto::select_ft232(multiple, "BBB", selected) ==
                        ftdi_proto::SelectResult::OK &&
                        selected.device_path == "/dev/ttyUSB1",
                    "the serial selector must pick the matching tty node") && ok;
        ok = expect(ftdi_proto::select_ft232(multiple, "NOPE", selected) ==
                        ftdi_proto::SelectResult::SERIAL_NOT_FOUND,
                    "an unknown serial must not fall back to another node") && ok;

        std::vector<ftdi_proto::Ft232Candidate> unreadable = {
            tty_candidate("/dev/ttyUSB0", "")};
        unreadable[0].serial_unreadable = true;
        ok = expect(ftdi_proto::select_ft232(unreadable, "SN0001", selected) ==
                        ftdi_proto::SelectResult::SERIAL_UNREADABLE,
                    "an unreadable sysfs serial must be reported as unreadable") && ok;
    }

    // ── Sysfs enumeration invariant (skipped when sysfs is unavailable) ──────
    std::vector<ftdi_proto::Ft232Candidate> live;
    std::string scan_error;
    const bool have_sysfs = ft232::Ft232Tty::enumerate_candidates(live, scan_error);
    if (!have_sysfs) {
        std::fprintf(stderr, "note: sysfs enumeration unavailable: %s\n",
                     scan_error.c_str());
    } else {
        for (size_t i = 0; i < live.size(); i++) {
            const ftdi_proto::Ft232Candidate& candidate = live[i];
            ok = expect(candidate.vid == ftdi_proto::FTDI_VID &&
                            candidate.pid == ftdi_proto::FT232R_PID,
                        "every tty candidate must be a 0403:6001 device") && ok;
            ok = expect(ftdi_proto::is_allowed_tty_path(candidate.device_path),
                        "every tty candidate must be a /dev/ttyUSBx node") && ok;
        }

        // An unknown serial must fail during selection, never open a device.
        ft232::Ft232Tty probe;
        std::string error;
        const bool opened = probe.open("__no_such_ft232_serial__", 1000000, error);
        ok = expect(!opened, "an unknown serial must not open any tty device") && ok;
        ok = expect(probe.open_failure() ==
                        (live.empty() ? ft232::OpenFailure::NO_DEVICE
                                      : ft232::OpenFailure::AMBIGUOUS),
                    "the failure must be a selection failure, not an open") && ok;
        probe.close();
    }

    return ok ? 0 : 1;
}
