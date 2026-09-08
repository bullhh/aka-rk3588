#include "robot/ivc_publish_retry.hpp"

#include <cerrno>

#define CHECK(condition)          \
    do {                          \
        if (!(condition)) {       \
            return __LINE__;      \
        }                         \
    } while (false)

int main() {
    int channel_storage = 0;
    int publish_calls = 0;
    int wait_calls = 0;
    int attempts = 0;
    int last_error = 0;

    auto* channel = retry_ivc_publish(
        [&]() -> int* {
            ++publish_calls;
            if (publish_calls < 3) {
                errno = EIO;
                return nullptr;
            }
            return &channel_storage;
        },
        [&]() { ++wait_calls; }, 50, attempts, last_error);

    CHECK(channel == &channel_storage);
    CHECK(attempts == 3);
    CHECK(wait_calls == 2);
    CHECK(last_error == 0);

    publish_calls = 0;
    wait_calls = 0;
    channel = retry_ivc_publish(
        [&]() -> int* {
            ++publish_calls;
            errno = EINVAL;
            return nullptr;
        },
        [&]() { ++wait_calls; }, 50, attempts, last_error);

    CHECK(channel == nullptr);
    CHECK(attempts == 1);
    CHECK(wait_calls == 0);
    CHECK(last_error == EINVAL);
    return 0;
}
