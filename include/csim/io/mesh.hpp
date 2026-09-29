#pragma once
#include <csim/model/description.hpp>
#include <filesystem>

namespace csim::io {
// Bounded local OBJ (untextured convex faces) and binary/ASCII STL loader.
model::TriangleMesh loadMesh(const std::filesystem::path& path);
} // namespace csim::io
