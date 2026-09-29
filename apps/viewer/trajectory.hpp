#pragma once

#include <csim/math/quaternion.hpp>
#include <filesystem>
#include <istream>
#include <vector>

namespace csim::viewer {
// Display-only SI samples, Z-up W and FLU B. Never fed back into physics.
struct Frame {
    double time = 0;
    math::Vector3 drone_position_W;
    math::Quaternion q_WB;
    math::Vector3 payload_position_W;
    double tension = 0;
    bool has_payload = true;
    bool cable_slack = false;
};
inline constexpr const char* trajectoryHeader =
    "time,drone_x,drone_y,drone_z,q_w,q_x,q_y,q_z,payload_x,payload_y,payload_z,tension";
std::vector<Frame> readTrajectory(std::istream& input);
std::vector<Frame> loadTrajectory(const std::filesystem::path& path);
} // namespace csim::viewer
