#pragma once

#include <cerrno>

inline bool ivc_publish_error_is_retryable(int error) {
    return error == EIO || error == EBUSY || error == EAGAIN ||
           error == ETIMEDOUT;
}

template <typename Publish, typename Wait>
auto retry_ivc_publish(Publish publish, Wait wait, int max_attempts,
                       int& attempts, int& last_error) -> decltype(publish()) {
    attempts = 0;
    last_error = EINVAL;
    if (max_attempts <= 0) return nullptr;

    for (attempts = 1; attempts <= max_attempts; ++attempts) {
        errno = 0;
        auto channel = publish();
        if (channel != nullptr) {
            last_error = 0;
            return channel;
        }

        last_error = errno;
        if (attempts == max_attempts ||
            !ivc_publish_error_is_retryable(last_error)) {
            break;
        }
        wait();
    }
    return nullptr;
}
