#include "perception/bucket_detector.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

int main() {
    constexpr int width = 100;
    constexpr int height = 80;
    std::vector<uint8_t> rgb(static_cast<size_t>(width) * height * 3, 0);

    for (int y = 20; y < 60; ++y) {
        for (int x = 30; x < 80; ++x) {
            rgb[(static_cast<size_t>(y) * width + x) * 3] = 255;
        }
    }

    perception::BucketDetector detector;
    auto result = detector.detect(rgb.data(), width, height, 1000);
    assert(result.visible);
    assert(result.center_x == 55);
    assert(result.center_y == 40);
    assert(result.width == 50);
    assert(result.height == 40);
    assert(result.red_area == 2000);

    std::fill(rgb.begin(), rgb.end(), 0);
    result = detector.detect(rgb.data(), width, height, 1000);
    assert(!result.visible);
    return 0;
}
