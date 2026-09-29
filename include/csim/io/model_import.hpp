#pragma once

#include <csim/model/description.hpp>

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace csim::io {

enum class DiagnosticSeverity { warning, error };
enum class ImportErrorCode {
    file_error,
    xml_error,
    invalid_value,
    invalid_topology,
    unsupported_feature,
    resource_error
};

struct ImportDiagnostic {
    DiagnosticSeverity severity = DiagnosticSeverity::error;
    ImportErrorCode code = ImportErrorCode::invalid_value;
    model::SourceLocation source;
    std::string message;
};

struct ImportOptions {
    // package://name/path resolution uses explicit local mappings.
    // No ROS process, environment probing, network download or xacro execution.
    // Relative mapping roots are relative to the top-level model directory.
    std::map<std::string, std::filesystem::path> package_roots;
    // XML and mesh budgets are enforced by the implementation.
    // Initial import is strict: no "ignore unsupported physics" switch.
};

struct ImportResult {
    std::optional<model::ModelDescription> description;
    std::vector<ImportDiagnostic> diagnostics;
    // Loader contract: description exists iff the whole supported
    // description was validated, with no error diagnostic. No partial success.
    // A description does NOT certify compatibility with CSim's dynamics.
};

// TinyXML2 DOM and numeric helpers live privately in src/io/.
// Copy strings/data out of XMLDocument; no DOM pointers in public descriptions.
// Reject malformed numbers, NaN/Inf, missing required attributes, duplicates,
// unsupported elements/attributes, DTD/external entities and unresolved assets.
// Report source file, line and element; never return a fake empty model.

} // namespace csim::io
