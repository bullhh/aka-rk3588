#ifndef PERCEPTION_BUCKET_DETECTOR_HPP
#define PERCEPTION_BUCKET_DETECTOR_HPP

#include <cstdint>
#include <vector>

namespace perception {

struct BucketDetection {
    bool visible = false;
    int center_x = 0;
    int center_y = 0;
    int width = 0;
    int height = 0;
    int red_area = 0;
};

// Allocation-stable red-bucket detector using the thresholds already validated
// by the full aka-rk3588 robot application.
class BucketDetector {
public:
    BucketDetection detect(const uint8_t* rgb, int width, int height,
                           int minimum_area = 1000);

private:
    int find_root(int label);
    void join(int first, int second);

    std::vector<uint8_t> mask_;
    std::vector<int> labels_;
    std::vector<int> parents_;
    std::vector<int> minimum_x_;
    std::vector<int> minimum_y_;
    std::vector<int> maximum_x_;
    std::vector<int> maximum_y_;
    std::vector<int> areas_;
};

} // namespace perception

#endif // PERCEPTION_BUCKET_DETECTOR_HPP
