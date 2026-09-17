#include "robot/omni_base.hpp"
#include <cstring>
#include <cstdio>
#include <unistd.h>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    feetech::FeetechBus bus(argv[1]);
    if (!bus.open()) return 2;
    OmniBase base(bus);
    if (std::strcmp(argv[2], "command-error") == 0) {
        char target[4096]{};
        const ssize_t length = readlink(argv[1], target, sizeof(target) - 1);
        if (length < 0) return 2;
        bus.close();
        if (unlink(argv[1]) != 0) return 2;
        const bool wrongly_succeeded = base.forward();
        if (symlink(target, argv[1]) != 0 || wrongly_succeeded) return 2;
        if (!bus.open() || !base.stop()) {
            std::fprintf(stderr, "reopen/stop failed: %s\n", bus.last_error().c_str());
            return 2;
        }
        // A successful later stop must not erase a failed motion command.
        return base.commands_ok() ? 0 : 1;
    }
    return base.verify_wheel_motion() ? 0 : 1;
}
