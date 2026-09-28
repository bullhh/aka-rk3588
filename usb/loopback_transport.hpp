#ifndef LOOPBACK_TRANSPORT_HPP
#define LOOPBACK_TRANSPORT_HPP

// Common interface for the FT232R loopback transports:
//   - Ft232Loopback : raw libusb vendor/bulk (Linux root and StarryOS)
//   - Ft232Tty      : Linux /dev/ttyUSBx termios fallback
//
// Both strictly verify the physical echo byte-for-byte so a PASS means the same
// thing regardless of which transport carried the frame.

#include <cstddef>
#include <cstdint>
#include <string>

namespace ft232 {

struct DeviceIdentity {
    uint16_t    vid = 0;
    uint16_t    pid = 0;
    int         bus = 0;
    int         address = 0;
    int         interface_number = 0;
    uint8_t     endpoint_in = 0;
    uint8_t     endpoint_out = 0;
    std::string serial;
    // Set only by the Linux TTY transport, e.g. "/dev/ttyUSB0".
    std::string tty_path;
};

enum class OpenFailure {
    NONE,
    NO_DEVICE,          // no 0403:6001 device found
    AMBIGUOUS,          // several candidates and no/again-ambiguous serial
    ACCESS_DENIED,      // device present but not openable by this user
    UNSUPPORTED,        // transport not available on this platform
    SERIAL_UNREADABLE,  // serial selector given, but a serial could not be read
    OTHER,
};

inline const char* open_failure_string(OpenFailure failure)
{
    switch (failure) {
    case OpenFailure::NONE:              return "none";
    case OpenFailure::NO_DEVICE:         return "no_device";
    case OpenFailure::AMBIGUOUS:         return "ambiguous";
    case OpenFailure::ACCESS_DENIED:     return "access_denied";
    case OpenFailure::UNSUPPORTED:       return "unsupported";
    case OpenFailure::SERIAL_UNREADABLE: return "serial_unreadable";
    case OpenFailure::OTHER:             return "other";
    }
    return "unknown";
}

// Auto mode may try the Linux TTY transport when raw libusb could not complete
// selection only because a serial string was unreadable (for example root-only
// /dev/bus/usb). True ambiguity and a missing device are not fallback-eligible:
// a TTY retry could not make them unambiguous.
inline bool may_fall_back_to_tty(OpenFailure failure)
{
    return failure == OpenFailure::ACCESS_DENIED ||
           failure == OpenFailure::UNSUPPORTED ||
           failure == OpenFailure::SERIAL_UNREADABLE ||
           failure == OpenFailure::OTHER;
}

class LoopbackTransport {
public:
    virtual ~LoopbackTransport() = default;
    LoopbackTransport(const LoopbackTransport&) = delete;
    LoopbackTransport& operator=(const LoopbackTransport&) = delete;

    virtual bool open(const std::string& serial_selector, int baudrate,
                      std::string& error) = 0;
    virtual bool exchange(const uint8_t* frame, size_t len, std::string& error,
                          uint64_t* latency_us) = 0;
    // Release the device. Returns false when a meaningful cleanup step failed.
    virtual bool close() = 0;
    virtual bool is_open() const = 0;
    virtual const DeviceIdentity& identity() const = 0;
    virtual const char* transport_name() const = 0;   // "usb" or "tty"
    virtual const std::string& cleanup_error() const = 0;
    virtual OpenFailure open_failure() const = 0;

protected:
    LoopbackTransport() = default;
};

} // namespace ft232

#endif // LOOPBACK_TRANSPORT_HPP
