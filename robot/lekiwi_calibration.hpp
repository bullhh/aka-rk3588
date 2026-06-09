#ifndef ROBOT_LEKIWI_CALIBRATION_HPP
#define ROBOT_LEKIWI_CALIBRATION_HPP

#include <map>
#include <string>

struct JointCalibration {
    int id = 0;
    int drive_mode = 0;
    int homing_offset = 0;
    int range_min = 0;
    int range_max = 4095;
};

class LekiwiCalibration {
public:
    bool load(const std::string& path);
    bool save(const std::string& path) const;
    void set(const std::string& name, const JointCalibration& cal);
    bool has(const std::string& name) const;
    const JointCalibration* get(const std::string& name) const;
    const std::string& last_error() const { return last_error_; }

private:
    std::map<std::string, JointCalibration> joints_;
    std::string last_error_;
};

#endif // ROBOT_LEKIWI_CALIBRATION_HPP
