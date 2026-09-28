// Minimal virtual-branch CLI. Only the vision + FT232R loopback workload is
// built here; real arm/wheel control lives on a separate real-robot branch.

#include <cstdio>
#include <cstring>

#include "vision_usb_ci.hpp"

int main(int argc, char** argv)
{
    if (argc < 2 || std::strcmp(argv[1], "vision-usb-ci") != 0) {
        std::fprintf(stderr,
                     "Usage: %s vision-usb-ci [model.rknn] [min_fps] [uvc_index] "
                     "[--ftdi-serial S] [--ftdi-transport usb|tty|auto]\n",
                     argv[0]);
        return 2;
    }
    return cmd_vision_usb_ci(argc, argv);
}
