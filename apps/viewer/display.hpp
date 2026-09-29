#pragma once
#include <csim/model/rigid_body.hpp>
#include "trajectory.hpp"
#include <deque>

namespace csim::viewer {
// Read-only rendering input; contains no physics runner or wall-clock scheduler.
struct Display {
    const Frame& frame;
    const std::deque<Frame>& trail;
    const model::RigidBodyAsset* asset=nullptr;
};
inline math::Vector3 centre(const Frame& f) {
    return f.has_payload ? (f.drone_position_W+f.payload_position_W)*0.5 : f.drone_position_W;
}
} // namespace csim::viewer
