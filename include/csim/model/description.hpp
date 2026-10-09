#pragma once

#include <csim/math/matrix.hpp>
#include <csim/math/quaternion.hpp>
#include <csim/math/vector.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace csim::model {

// Interchange data; validate before use. Not an executable simulation model.
struct SourceLocation {
    std::filesystem::path file;
    std::size_t line = 0; // 1-based; zero means unavailable.
    std::string element; // XML element path, including name when available.
};

// T_PC: x_P = q_PC.rotate(x_C) + position; metres, Hamilton wxyz.
// Parent/child refer to the particular frames named by the containing field.
// validateDescription checks finite position and nonzero finite orientation.
struct Pose {
    math::Vector3 position;
    math::Quaternion orientation;
};

struct Box {
    math::Vector3 size; // Full side lengths, not MJCF half-extents.
};

struct Sphere {
    double radius = 0.0;
};

struct Cylinder {
    double radius = 0.0;
    double length = 0.0; // Full length, centered on local Z.
};

struct TriangleMesh {
    std::vector<math::Vector3> vertices;
    std::vector<std::array<std::size_t,3>> triangles;
};

struct MeshReference {
    std::string uri;
    math::Vector3 scale{1.0, 1.0, 1.0}; // Dimensionless; apply exactly once.
    SourceLocation source; // Keep the declaring file for resource resolution.
    std::shared_ptr<const TriangleMesh> mesh; // Decoded local coordinates; scale not yet applied.
};

using Geometry = std::variant<std::monostate, Box, Sphere, Cylinder, MeshReference>;

struct GeometryInstance {
    std::string name;
    Pose body_from_geometry;
    Geometry shape; // monostate is invalid, not an implicit unit box.
    std::optional<std::array<double, 4>> rgba; // Resolved RGBA in [0, 1].
    SourceLocation source;
};

struct InertialDescription {
    double mass = 0.0; // kg; zero is not a usable drone mass.
    Pose body_from_inertial; // Inertial origin MUST be the body's center of mass.
    math::Matrix3 inertia; // kg*m^2, about that CoM, expressed in inertial axes.
    SourceLocation source;
    // validateDescription checks SPD and physical principal-moment inequalities.
    // Missing inertia is represented by BodyDescription::inertial == nullopt,
    // never by silently substituting zero, identity, or geometry-derived inertia.
};

struct BodyDescription {
    std::string name;
    std::optional<std::string> parent; // nullopt for the sole root.
    Pose parent_from_body; // For root: reference frame A from root frame R.
    std::optional<InertialDescription> inertial;
    std::vector<GeometryInstance> visuals;
    std::vector<GeometryInstance> collisions; // Geometry only, no contact solver.
    SourceLocation source;
    // All non-root edges are fixed. Moving joints MUST be rejected until the
    // representation includes ordered joint transforms, axes, limits and state.
    // validateDescription checks names, references, connectivity and cycles.
};

struct AttachmentDescription {
    std::string name;
    std::string body;
    Pose body_from_attachment;
    SourceLocation source;
};

enum class SourceFormat { urdf, mjcf };
enum class RootMotion { unspecified, fixed, free };

struct ModelDescription {
    std::string name;
    SourceFormat format = SourceFormat::urdf;
    SourceLocation source;
    RootMotion root_motion = RootMotion::unspecified;
    std::vector<BodyDescription> bodies;
    std::vector<AttachmentDescription> attachments;
    // URDF alone does not choose root mobility. MJCF root without a joint is
    // fixed; a supported root freejoint is free. Never silently unfix a model.
    // Preserve source-frame placement; choosing W/FLU and initial state
    // belongs to an explicit simulation adapter, not XML or mesh heuristics.
};

} // namespace csim::model
