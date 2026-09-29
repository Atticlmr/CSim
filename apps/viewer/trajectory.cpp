#include "trajectory.hpp"

#include <array>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace csim::viewer {
namespace {
void trimCR(std::string& line) { if (!line.empty() && line.back()=='\r') line.pop_back(); }
Frame parse(const std::string& line) {
    std::array<double,12> values{};
    std::istringstream fields(line);
    for (std::size_t i=0; i<values.size(); ++i) {
        std::string field;
        if (!std::getline(fields,field,',')) throw std::invalid_argument("Expected 12 numeric columns");
        std::istringstream number(field); number.imbue(std::locale::classic());
        if (!(number>>values[i]) || !std::isfinite(values[i])) throw std::invalid_argument("Non-finite or invalid number");
        number>>std::ws;
        if (!number.eof()) throw std::invalid_argument("Trailing text in numeric column");
    }
    if (!fields.eof()) throw std::invalid_argument("Too many columns");
    Frame frame{values[0],{values[1],values[2],values[3]},
        {values[4],values[5],values[6],values[7]}, {values[8],values[9],values[10]},values[11]};
    if (frame.time<0 || frame.tension<=0) throw std::invalid_argument("Time must be nonnegative and tension positive");
    frame.q_WB=frame.q_WB.normalized();
    if (frame.drone_position_W.norm()>10000 || frame.payload_position_W.norm()>10000)
        throw std::invalid_argument("Viewer positions must be within 10 km of the world origin");
    return frame;
}
}
std::vector<Frame> readTrajectory(std::istream& input) {
    std::string line;
    if (!std::getline(input,line)) throw std::invalid_argument("Empty trajectory");
    trimCR(line);
    if (line!=trajectoryHeader) throw std::invalid_argument("Unsupported trajectory header; see docs/visualization.md");
    std::vector<Frame> frames;
    double length=0;
    std::size_t line_number=1;
    while (std::getline(input,line)) {
        ++line_number; trimCR(line);
        try {
            if (frames.size()>=1000000) throw std::invalid_argument("Trajectory exceeds one million samples");
            const auto frame=parse(line);
            const double current_length=(frame.payload_position_W-frame.drone_position_W).norm();
            if (!(current_length>=1e-4 && current_length<=1000))
                throw std::invalid_argument("Viewer cable length must be between 0.0001 and 1000 m");
            if (!frames.empty()) {
                if (!(frame.time>frames.back().time)) throw std::invalid_argument("Times must strictly increase");
                if (std::abs(current_length-length)>1e-7*std::max(1.0,length))
                    throw std::invalid_argument("Cable length changes between samples");
            } else length=current_length;
            frames.push_back(frame);
        } catch (const std::exception& error) {
            throw std::invalid_argument("Trajectory line "+std::to_string(line_number)+": "+error.what());
        }
    }
    if (input.bad()) throw std::runtime_error("Failed while reading trajectory");
    if (frames.empty()) throw std::invalid_argument("Trajectory contains no samples");
    return frames;
}
std::vector<Frame> loadTrajectory(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open trajectory: "+path.string());
    return readTrajectory(input);
}
} // namespace csim::viewer
