#include "scene.hpp"

#include <algorithm>
#include <cmath>

namespace csim::viewer {
Scene::Scene(std::vector<Frame> recording) : recording_(std::move(recording)) {
    if (!replay()) model_=std::make_shared<simulation::SuspendedPayloadModel>(2,0.5,1.2,
        dynamics::Drone::defaultInertia(),9.81,0.002);
    reset();
}
Frame Scene::liveFrame() const {
    const auto s=simulation::getState(*model_,*data_);
    Frame result{s.time,s.state.drone.position_W,s.state.drone.q_WB,s.physical.payload_position_W,s.physical.tension};
    result.cable_slack=s.physical.slack;
    return result;
}
void Scene::reset() {
    if (replay()) { index_=0; frame_=recording_.front(); }
    else {
        dynamics::SuspendedPayloadState state;
        state.drone.position_W={0,0,5};
        state.cable_direction_W=math::Vector3{0.3,0.2,-1}.normalized();
        state.cable_angular_velocity_W=state.cable_direction_W.cross({0.2,-0.3,0.1});
        data_=simulation::makeData(model_,{24.525,{}},state);
        frame_=liveFrame();
    }
    pending_=0; limited_=false; trail_.clear(); trail_.push_back(frame_);
}
void Scene::setPaused(bool value) { paused_=value; pending_=0; limited_=false; }
void Scene::setSpeed(double value) {
    if (!std::isfinite(value) || value<0.125 || value>4) throw std::invalid_argument("Playback speed must be in [0.125, 4]");
    speed_=value; pending_=0;
}
double Scene::nextInterval() const {
    if (!replay()) return model_->timestep();
    return ended() ? 0 : recording_[index_+1].time-frame_.time;
}
void Scene::appendTrail() {
    // Live history sampled at 50 Hz independently of the display rate.
    if (frame_.time-trail_.back().time>=0.02-1e-12 || ended()) {
        trail_.push_back(frame_);
        if (trail_.size()>1500) trail_.pop_front();
    }
}
void Scene::advanceOne() {
    if (ended()) return;
    if (replay()) frame_=recording_[++index_];
    else { simulation::step(*model_,*data_); frame_=liveFrame(); }
    appendTrail();
}
void Scene::singleStep() { setPaused(true); advanceOne(); }
void Scene::advance(double wall_seconds) {
    if (!std::isfinite(wall_seconds) || wall_seconds<0) throw std::invalid_argument("Elapsed time must be finite and nonnegative");
    limited_=false;
    if (paused_ || ended()) return;
    // Drop long wall-clock stalls and bound work. Never enlarge the physics dt.
    limited_=wall_seconds>0.25;
    pending_+=std::min(wall_seconds,0.25)*speed_;
    unsigned steps=0;
    while (!ended() && steps<128) {
        const double dt=nextInterval();
        if (pending_<dt && dt-pending_>1e-12*dt) break;
        advanceOne();
        pending_=std::max(0.0,pending_-dt);
        ++steps;
    }
    if (ended()) pending_=0;
    else if (steps==128 && pending_>=nextInterval()) { pending_=0; limited_=true; }
}
} // namespace csim::viewer
