#pragma once

#include <csim/io/model_import.hpp>

namespace csim::io {

// Strict MJCF 3.3.1 subset, explicit inertia, fixed children, optional free root.
// TODO: default/class/childclass, include/attach, fromto, geometry-derived inertia,
// mesh refpos/refquat, moving joints, contact/actuator/sensor/tendon semantics.
ImportResult loadMjcf(const std::filesystem::path& file,
                      const ImportOptions& options = {});

} // namespace csim::io
