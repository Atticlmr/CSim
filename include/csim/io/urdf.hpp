#pragma once

#include <csim/io/model_import.hpp>

namespace csim::io {

// Strict fixed-link URDF: primitives, explicit inertia, RGBA, local/package mesh.
// TODO: Moving joints, xacro/transmission/Gazebo semantics and textured materials.
ImportResult loadUrdf(const std::filesystem::path& file,
                      const ImportOptions& options = {});

} // namespace csim::io
