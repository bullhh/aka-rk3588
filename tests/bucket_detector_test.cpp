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

    // Exercise the HSV hue, saturation and brightness boundaries through the
    // complete detector, including reuse after a previously visible region.
    struct ColorCase { uint8_t r, g, b; bool visible; };
    for (const auto color : {
            ColorCase{255, 80, 0, true}, ColorCase{255, 86, 0, false},
            ColorCase{255, 0, 80, true}, ColorCase{255, 0, 86, false},
            ColorCase{255, 175, 175, true}, ColorCase{255, 176, 176, false},
            ColorCase{50, 0, 0, true}, ColorCase{49, 0, 0, false},
            ColorCase{255, 255, 0, false}, ColorCase{0, 255, 0, false},
            ColorCase{0, 0, 255, false}, ColorCase{255, 255, 255, false}}) {
        for (size_t pixel = 0; pixel < rgb.size() / 3; ++pixel) {
            rgb[pixel * 3] = color.r;
            rgb[pixel * 3 + 1] = color.g;
            rgb[pixel * 3 + 2] = color.b;
        }
        result = detector.detect(rgb.data(), width, height, 1000);
        assert(result.visible == color.visible);
        if (result.visible) assert(result.red_area == width * height);
    }

    std::vector<uint8_t> resized(30 * 20 * 3, 0);
    for (int y = 0; y < 20; ++y)
        for (int x = 0; x < 10; ++x) resized[(y * 30 + x) * 3] = 255;
    result = detector.detect(resized.data(), 30, 20, 100);
    assert(result.visible && result.red_area == 200);
    assert(result.width == 10 && result.height == 20);
    std::fill(resized.begin(), resized.end(), 0);
    assert(!detector.detect(resized.data(), 30, 20, 100).visible);

    // A later row joins two earlier runs; the isolated right column remains
    // a separate component, and diagonal contact alone never joins pixels.
    std::vector<uint8_t> bridge(8 * 4 * 3, 0);
    for (int x : {0, 1, 4, 5, 7}) bridge[x * 3] = 255;
    for (int x : {0, 1, 2, 3, 4, 5, 7}) bridge[(8 + x) * 3] = 255;
    result = detector.detect(bridge.data(), 8, 4, 1);
    assert(result.visible && result.red_area == 10);
    assert(result.width == 6 && result.height == 2);
    assert(result.center_x == 3 && result.center_y == 1);
    std::fill(bridge.begin(), bridge.end(), 0);
    for (int i = 0; i < 3; ++i) bridge[(i * 8 + i) * 3] = 255;
    result = detector.detect(bridge.data(), 8, 4, 1);
    assert(result.visible && result.red_area == 1);
    assert(result.center_x == 0 && result.center_y == 0);
    return 0;
}
