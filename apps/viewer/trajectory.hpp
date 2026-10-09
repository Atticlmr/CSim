#pragma once

#include <csim/math/quaternion.hpp>
#include <filesystem>
#include <istream>
#include <optional>
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
    bool rigid_payload = false;
    math::Quaternion q_WP;
    std::optional<math::Vector3> drone_attachment_W;
    std::optional<math::Vector3> payload_attachment_W;
};
inline constexpr const char* trajectoryHeader =
    "time,drone_x,drone_y,drone_z,q_w,q_x,q_y,q_z,payload_x,payload_y,payload_z,tension";
inline constexpr const char* rigidTrajectoryHeader =
    "time,drone_x,drone_y,drone_z,q_w,q_x,q_y,q_z,payload_x,payload_y,payload_z,tension,"
    "payload_q_w,payload_q_x,payload_q_y,payload_q_z,drone_attachment_x,drone_attachment_y,drone_attachment_z,"
    "payload_attachment_x,payload_attachment_y,payload_attachment_z,cable_slack,cable_length";
std::vector<Frame> readTrajectory(std::istream& input);
std::vector<Frame> loadTrajectory(const std::filesystem::path& path);
} // namespace csim::viewer
