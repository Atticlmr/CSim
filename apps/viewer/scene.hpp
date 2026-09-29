#pragma once

#include "trajectory.hpp"
#include <csim/simulation/suspended_payload.hpp>
#include <deque>
#include <optional>

namespace csim::viewer {
// Window-independent fixed-step simulation and recording playback.
class Scene {
public:
    explicit Scene(std::vector<Frame> recording = {});
    const Frame& frame() const { return frame_; }
    const std::deque<Frame>& trail() const { return trail_; }
    bool replay() const { return !recording_.empty(); }
    bool paused() const { return paused_; }
    bool ended() const { return replay() && index_+1==recording_.size(); }
    bool limited() const { return limited_; }
    double speed() const { return speed_; }
    void setPaused(bool value);
    void setSpeed(double value);
    void advance(double wall_seconds);
    void singleStep();
    void reset();
private:
    double nextInterval() const;
    void advanceOne();
    void appendTrail();
    Frame liveFrame() const;
    std::vector<Frame> recording_;
    std::size_t index_=0;
    std::shared_ptr<simulation::SuspendedPayloadModel> model_;
    std::optional<simulation::SuspendedPayloadData> data_;
    Frame frame_;
    std::deque<Frame> trail_;
    bool paused_=false, limited_=false;
    double pending_=0, speed_=1;
};
} // namespace csim::viewer
