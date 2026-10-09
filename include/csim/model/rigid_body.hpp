#pragma once
#include <csim/model/description.hpp>
#include <map>

namespace csim::model {
Pose compose(const Pose& parent, const Pose& child);
void validateInertial(const InertialDescription& inertial);
void validateDescription(const ModelDescription& description);

// Source link frames survive fixed-body aggregation; B is the assembly CoM.
struct FixedLink {
    std::string name;
    std::optional<std::string> parent;
    Pose body_from_link; // T_BL, not the visual or inertial frame.
    math::Vector3 center_L; // This link's own CoM, expressed in L.
};

struct RigidBodyAsset {
    double mass=0;
    math::Vector3 center_R;
    math::Matrix3 inertia_B;
    Pose initial_pose_WB;
    std::vector<GeometryInstance> visuals_B;
    std::vector<FixedLink> links; // Source order; includes links without visuals.
    std::map<std::string, Pose> attachments_B;
};
// Fixed roots/unspecified URDF mobility require explicit permission to free them.
// R_BR is the source root axes -> desired FLU body axes rotation; W is source A.
RigidBodyAsset buildRigidBody(const ModelDescription& description, bool free_base=false,
                             math::Quaternion q_BR={});
} // namespace csim::model
