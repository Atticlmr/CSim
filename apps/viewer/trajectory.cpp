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
Frame parse(const std::string& line,bool rigid,double& length) {
    std::vector<double> values(rigid?24:12);
    std::istringstream fields(line);
    for (std::size_t i=0; i<values.size(); ++i) {
        std::string field;
        if (!std::getline(fields,field,',')) throw std::invalid_argument("Missing numeric columns");
        std::istringstream number(field); number.imbue(std::locale::classic());
        if (!(number>>values[i]) || !std::isfinite(values[i])) throw std::invalid_argument("Non-finite or invalid number");
        number>>std::ws;
        if (!number.eof()) throw std::invalid_argument("Trailing text in numeric column");
    }
    if (!fields.eof()) throw std::invalid_argument("Too many columns");
    Frame frame{values[0],{values[1],values[2],values[3]},
        {values[4],values[5],values[6],values[7]}, {values[8],values[9],values[10]},values[11]};
    if (rigid) {
        if (values[22]!=0 && values[22]!=1) throw std::invalid_argument("Cable slack flag must be 0 or 1");
        frame.rigid_payload=true;
        frame.q_WP=math::Quaternion{values[12],values[13],values[14],values[15]}.normalized();
        frame.drone_attachment_W=math::Vector3{values[16],values[17],values[18]};
        frame.payload_attachment_W=math::Vector3{values[19],values[20],values[21]};
        frame.cable_slack=values[22]==1;
        length=values[23];
        if (frame.drone_attachment_W->norm()>10000 || frame.payload_attachment_W->norm()>10000)
            throw std::invalid_argument("Viewer attachments must be within 10 km of the world origin");
    }
    if (frame.time<0 || (frame.cable_slack ? frame.tension!=0 : frame.tension<=0))
        throw std::invalid_argument("Require nonnegative time, zero slack tension and positive taut tension");
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
    const bool rigid=line==rigidTrajectoryHeader;
    if (!rigid && line!=trajectoryHeader) throw std::invalid_argument("Unsupported trajectory header");
    std::vector<Frame> frames;
    double length=0;
    std::size_t line_number=1;
    while (std::getline(input,line)) {
        ++line_number; trimCR(line);
        try {
            if (frames.size()>=1000000) throw std::invalid_argument("Trajectory exceeds one million samples");
            double nominal_length=0;
            const auto frame=parse(line,rigid,nominal_length);
            const double distance=rigid ? (*frame.payload_attachment_W-*frame.drone_attachment_W).norm()
                                        : (frame.payload_position_W-frame.drone_position_W).norm();
            const double current_length=rigid ? nominal_length : distance;
            if (!(current_length>=1e-4 && current_length<=1000))
                throw std::invalid_argument("Viewer cable length must be between 0.0001 and 1000 m");
            if (rigid && (frame.cable_slack ? distance>current_length+1e-7*std::max(1.,current_length)
                : std::abs(distance-current_length)>1e-7*std::max(1.,current_length)))
                throw std::invalid_argument("Attachment distance violates cable mode or length");
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
