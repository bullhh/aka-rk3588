#ifndef VISION_USB_CI_HPP
#define VISION_USB_CI_HPP

// Vision + FT232R USB loopback CI workload.
//
// Runs real UVC capture / JPEG decode / RKNN inference for two 10s windows
// concurrently with a bounded FT232R binary loopback exchange cadence, then
// emits a single final [VISION_USB_CI] APPLICATION_PASS only after every
// fallible step and cleanup has succeeded.
//
// Usage:
//   tennis vision-usb-ci [model.rknn] [min_fps] [uvc_index] [--ftdi-serial S]
//                        [--ftdi-transport usb|tty|auto]
//
// The FT232R transport is auto by default: raw libusb first (StarryOS/root),
// then the Linux /dev/ttyUSBx termios fallback when the raw node is denied.
int cmd_vision_usb_ci(int argc, char** argv);

#endif // VISION_USB_CI_HPP
