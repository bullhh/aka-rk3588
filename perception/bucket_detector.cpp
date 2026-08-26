#include "bucket_detector.hpp"

#include <algorithm>

namespace perception {
namespace {

bool is_red(uint8_t red, uint8_t green, uint8_t blue) {
    const int maximum = std::max({static_cast<int>(red), static_cast<int>(green),
                                  static_cast<int>(blue)});
    const int minimum = std::min({static_cast<int>(red), static_cast<int>(green),
                                  static_cast<int>(blue)});
    const int delta = maximum - minimum;
    const int saturation = maximum == 0 ? 0 : delta * 255 / maximum;
    if (saturation < 80 || maximum < 50) return false;
    if (delta == 0) return true;

    float hue;
    if (maximum == red) {
        hue = 60.0f * ((static_cast<int>(green) - static_cast<int>(blue)) /
                       static_cast<float>(delta));
    } else if (maximum == green) {
        hue = 60.0f * ((static_cast<int>(blue) - static_cast<int>(red)) /
                       static_cast<float>(delta) + 2.0f);
    } else {
        hue = 60.0f * ((static_cast<int>(red) - static_cast<int>(green)) /
                       static_cast<float>(delta) + 4.0f);
    }
    if (hue < 0.0f) hue += 360.0f;
    return hue <= 20.0f || hue >= 340.0f;
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
    if (first != second) parents_[first] = second;
}

BucketDetection BucketDetector::detect(const uint8_t* rgb, int width, int height,
                                        int minimum_area) {
    BucketDetection result;
    if (rgb == nullptr || width <= 0 || height <= 0) return result;

    const size_t pixels = static_cast<size_t>(width) * height;
    mask_.assign(pixels, 0);
    labels_.assign(pixels, 0);
    for (size_t pixel = 0; pixel < pixels; ++pixel) {
        mask_[pixel] = is_red(rgb[pixel * 3], rgb[pixel * 3 + 1],
                              rgb[pixel * 3 + 2]) ? 1U : 0U;
    }

    parents_.clear();
    parents_.push_back(0);
    int next_label = 1;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int index = y * width + x;
            if (mask_[index] == 0U) continue;
            const int above = y > 0 ? labels_[(y - 1) * width + x] : 0;
            const int left = x > 0 ? labels_[y * width + x - 1] : 0;
            if (above == 0 && left == 0) {
                labels_[index] = next_label;
                parents_.push_back(next_label++);
            } else if (above == 0) {
                labels_[index] = left;
            } else if (left == 0) {
                labels_[index] = above;
            } else {
                labels_[index] = left;
                join(above, left);
            }
        }
    }

    minimum_x_.assign(next_label, width);
    minimum_y_.assign(next_label, height);
    maximum_x_.assign(next_label, -1);
    maximum_y_.assign(next_label, -1);
    areas_.assign(next_label, 0);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int index = y * width + x;
            if (labels_[index] == 0) continue;
            const int root = find_root(labels_[index]);
            labels_[index] = root;
            ++areas_[root];
            minimum_x_[root] = std::min(minimum_x_[root], x);
            minimum_y_[root] = std::min(minimum_y_[root], y);
            maximum_x_[root] = std::max(maximum_x_[root], x);
            maximum_y_[root] = std::max(maximum_y_[root], y);
        }
    }

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
