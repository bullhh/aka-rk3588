#include "lekiwi_arm_poses.hpp"

#include <fstream>
#include <sstream>

bool LekiwiArmPoses::load(const std::string& path) {
    poses_.clear();
    last_error_.clear();

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        last_error_ = "cannot open arm pose file: " + path;
        return false;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string pose, joint;
        int raw = 0;
        if (!(iss >> pose >> joint >> raw)) continue;
        poses_[pose].joints[joint] = raw;
    }
    if (poses_.empty()) {
        last_error_ = "no poses in: " + path;
        return false;
    }
    return true;
}

bool LekiwiArmPoses::save(const std::string& path) const {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << "# pose_name joint_name raw_0_4095\n";
    for (const auto& pkv : poses_) {
        for (const auto& jkv : pkv.second.joints) {
            ofs << pkv.first << " " << jkv.first << " " << jkv.second << "\n";
        }
    }
    return true;
}

bool LekiwiArmPoses::has(const std::string& name) const {
    return poses_.find(name) != poses_.end();
}

const ArmRawPose* LekiwiArmPoses::get(const std::string& name) const {
    auto it = poses_.find(name);
    return it == poses_.end() ? nullptr : &it->second;
}

void LekiwiArmPoses::set(const std::string& name, const ArmRawPose& pose) {
    poses_[name] = pose;
}

std::vector<std::string> LekiwiArmPoses::names() const {
    std::vector<std::string> out;
    for (const auto& kv : poses_) out.push_back(kv.first);
    return out;
}
