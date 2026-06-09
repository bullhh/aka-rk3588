#include "lekiwi_calibration.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <vector>

bool LekiwiCalibration::load(const std::string& path) {
    joints_.clear();
    last_error_.clear();

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        last_error_ = "cannot open calibration file: " + path;
        return false;
    }

    std::stringstream ss;
    ss << ifs.rdbuf();
    std::string text = ss.str();

    std::regex block_re("\"([^\"]+)\"\\s*:\\s*\\{([^}]*)\\}");
    std::regex int_re("\"([^\"]+)\"\\s*:\\s*(-?[0-9]+)");

    auto begin = std::sregex_iterator(text.begin(), text.end(), block_re);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        std::string name = (*it)[1].str();
        std::string body = (*it)[2].str();
        JointCalibration cal;

        auto ib = std::sregex_iterator(body.begin(), body.end(), int_re);
        for (auto ii = ib; ii != end; ++ii) {
            std::string key = (*ii)[1].str();
            int value = std::stoi((*ii)[2].str());
            if      (key == "id") cal.id = value;
            else if (key == "drive_mode") cal.drive_mode = value;
            else if (key == "homing_offset") cal.homing_offset = value;
            else if (key == "range_min") cal.range_min = value;
            else if (key == "range_max") cal.range_max = value;
        }

        if (cal.id > 0 && cal.range_min <= cal.range_max) {
            joints_[name] = cal;
        }
    }

    if (joints_.empty()) {
        last_error_ = "no valid calibration entries in: " + path;
        return false;
    }
    return true;
}

bool LekiwiCalibration::save(const std::string& path) const {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;

    std::vector<std::string> order = {
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper",
        "base_left_wheel",
        "base_back_wheel",
        "base_right_wheel",
    };

    ofs << "{\n";
    bool first = true;
    auto write_entry = [&](const std::string& name, const JointCalibration& cal) {
        if (!first) ofs << ",\n";
        first = false;
        ofs << "    \"" << name << "\": {\n";
        ofs << "        \"id\": " << cal.id << ",\n";
        ofs << "        \"drive_mode\": " << cal.drive_mode << ",\n";
        ofs << "        \"homing_offset\": " << cal.homing_offset << ",\n";
        ofs << "        \"range_min\": " << cal.range_min << ",\n";
        ofs << "        \"range_max\": " << cal.range_max << "\n";
        ofs << "    }";
    };

    for (const auto& name : order) {
        auto it = joints_.find(name);
        if (it != joints_.end()) write_entry(name, it->second);
    }
    for (const auto& kv : joints_) {
        bool listed = false;
        for (const auto& name : order) {
            if (kv.first == name) { listed = true; break; }
        }
        if (!listed) write_entry(kv.first, kv.second);
    }
    ofs << "\n}\n";
    return true;
}

void LekiwiCalibration::set(const std::string& name, const JointCalibration& cal) {
    joints_[name] = cal;
}

bool LekiwiCalibration::has(const std::string& name) const {
    return joints_.find(name) != joints_.end();
}

const JointCalibration* LekiwiCalibration::get(const std::string& name) const {
    auto it = joints_.find(name);
    return it == joints_.end() ? nullptr : &it->second;
}
