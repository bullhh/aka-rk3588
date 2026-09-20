#include "bucket_detector.hpp"

#include <algorithm>
#include <cstring>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace perception {
namespace {

bool is_red(uint8_t red, uint8_t green, uint8_t blue) {
    // The accepted hue interval is centered on red. Other dominant channels
    // cannot qualify, so avoid their HSV divisions entirely.
    if (red < 50 || red < green || red < blue) return false;
    const int delta = red - std::min(green, blue);
    // floor(delta * 255 / red) >= 80, without an integer division.
    if (delta * 255 < 80 * red) return false;
    const int difference = static_cast<int>(green) - static_cast<int>(blue);
    return 3 * (difference < 0 ? -difference : difference) <= delta;
}

void classify_red(const uint8_t* rgb, uint8_t* mask, size_t pixels) {
    size_t index = 0;
#if defined(__aarch64__)
    for (; index + 16 <= pixels; index += 16) {
        const uint8x16x3_t channels = vld3q_u8(rgb + index * 3);
        const uint8x16_t red = channels.val[0];
        const uint8x16_t green = channels.val[1];
        const uint8x16_t blue = channels.val[2];
        uint8x16_t valid = vandq_u8(vcgeq_u8(red, vmaxq_u8(green, blue)),
                                   vcgeq_u8(red, vdupq_n_u8(50)));
        const uint8x16_t delta = vsubq_u8(red, vminq_u8(green, blue));
        const uint8x16_t difference = vabdq_u8(green, blue);
        const auto classify_half = [](uint8x8_t r, uint8x8_t d, uint8x8_t diff) {
            const uint16x8_t saturated = vcgeq_u16(vmull_u8(d, vdup_n_u8(255)),
                                                   vmull_u8(r, vdup_n_u8(80)));
            const uint16x8_t hue = vcleq_u16(vmull_u8(diff, vdup_n_u8(3)), vmovl_u8(d));
            return vmovn_u16(vandq_u16(saturated, hue));
        };
        valid = vandq_u8(valid, vcombine_u8(
            classify_half(vget_low_u8(red), vget_low_u8(delta), vget_low_u8(difference)),
            classify_half(vget_high_u8(red), vget_high_u8(delta), vget_high_u8(difference))));
        vst1q_u8(mask + index, valid);
    }
#endif
    for (; index < pixels; ++index) {
        mask[index] = is_red(rgb[index * 3], rgb[index * 3 + 1], rgb[index * 3 + 2]);
    }
}

} // namespace

int BucketDetector::find_root(int label) {
    while (parents_[label] != label) {
        parents_[label] = parents_[parents_[label]];
        label = parents_[label];
    }
    return label;
}

void BucketDetector::join(int first, int second) {
    first = find_root(first);
    second = find_root(second);
    if (first == second) return;
    parents_[first] = second;
    areas_[second] += areas_[first];
    areas_[first] = 0;
    minimum_x_[second] = std::min(minimum_x_[second], minimum_x_[first]);
    minimum_y_[second] = std::min(minimum_y_[second], minimum_y_[first]);
    maximum_x_[second] = std::max(maximum_x_[second], maximum_x_[first]);
    maximum_y_[second] = std::max(maximum_y_[second], maximum_y_[first]);
}

BucketDetection BucketDetector::detect(const uint8_t* rgb, int width, int height,
                                        int minimum_area) {
    BucketDetection result;
    if (rgb == nullptr || width <= 0 || height <= 0) return result;

    mask_.resize(static_cast<size_t>(width));
    parents_.clear();
    minimum_x_.clear();
    minimum_y_.clear();
    maximum_x_.clear();
    maximum_y_.clear();
    areas_.clear();
    previous_runs_.clear();
    current_runs_.clear();
    const auto new_label = [&]() {
        const int label = static_cast<int>(parents_.size());
        parents_.push_back(label);
        minimum_x_.push_back(width);
        minimum_y_.push_back(height);
        maximum_x_.push_back(-1);
        maximum_y_.push_back(-1);
        areas_.push_back(0);
        return label;
    };
    new_label(); // Background label.

    // A horizontal run has the same label at every pixel in the original
    // four-connected scan. Merge only intersecting runs from the preceding row.
    // Preserve above -> left union order, including equal-area tie behavior.
    for (int y = 0; y < height; ++y) {
        current_runs_.clear();
        size_t previous = 0;
        // Consume one row while its RGB data and mask are still cache-hot.
        classify_red(rgb + static_cast<size_t>(y) * width * 3, mask_.data(), width);
        const uint8_t* row = mask_.data();
        int x = 0;
        while (x < width) {
            while (x < width && row[x] == 0) {
                if (x + 8 <= width) {
                    uint64_t word;
                    std::memcpy(&word, row + x, sizeof(word));
                    if (word == 0) { x += 8; continue; }
                }
                ++x;
            }
            if (x == width) break;
            const int first = x;
            while (x < width && row[x] != 0) ++x;
            const int last = x - 1;
            while (previous < previous_runs_.size() && previous_runs_[previous].last < first)
                ++previous;
            const int label = previous < previous_runs_.size() &&
                              previous_runs_[previous].first <= first
                                ? previous_runs_[previous].label : new_label();
            for (size_t i = previous; i < previous_runs_.size() &&
                                     previous_runs_[i].first <= last; ++i) {
                join(previous_runs_[i].label, label);
            }
            const int root = find_root(label);
            areas_[root] += last - first + 1;
            minimum_x_[root] = std::min(minimum_x_[root], first);
            minimum_y_[root] = std::min(minimum_y_[root], y);
            maximum_x_[root] = std::max(maximum_x_[root], last);
            maximum_y_[root] = std::max(maximum_y_[root], y);
            current_runs_.push_back({first, last, label});
        }
        previous_runs_.swap(current_runs_);
    }
    const int next_label = static_cast<int>(parents_.size());

    int best = 0;
    for (int label = 1; label < next_label; ++label) {
        if (find_root(label) == label && areas_[label] >= minimum_area &&
            (best == 0 || areas_[label] > areas_[best])) {
            best = label;
        }
    }
    if (best == 0) return result;

    result.visible = true;
    result.width = maximum_x_[best] - minimum_x_[best] + 1;
    result.height = maximum_y_[best] - minimum_y_[best] + 1;
    result.center_x = minimum_x_[best] + result.width / 2;
    result.center_y = minimum_y_[best] + result.height / 2;
    result.red_area = areas_[best];
    return result;
}

} // namespace perception
