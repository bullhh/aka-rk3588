#ifndef ROBOT_LEKIWI_ARM_POSES_HPP
#define ROBOT_LEKIWI_ARM_POSES_HPP

#include <map>
#include <string>
#include <vector>

struct ArmRawPose {
    std::map<std::string, int> joints;
};

class LekiwiArmPoses {
public:
    bool load(const std::string& path);
    bool save(const std::string& path) const;
    bool has(const std::string& name) const;
    const ArmRawPose* get(const std::string& name) const;
    void set(const std::string& name, const ArmRawPose& pose);
    std::vector<std::string> names() const;
    const std::string& last_error() const { return last_error_; }

private:
    std::map<std::string, ArmRawPose> poses_;
    std::string last_error_;
};

#endif // ROBOT_LEKIWI_ARM_POSES_HPP
