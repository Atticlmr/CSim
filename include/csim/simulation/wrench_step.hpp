#pragma once

#include <csim/dynamics/drone.hpp>
#include <utility>

namespace csim::simulation {
template<class Model, class Data>
void stepWithControl(const Model& model, Data& data, dynamics::DroneControl control) {
    auto candidate = data;
    setControl(model, candidate, control);
    step(model, candidate);
    data = std::move(candidate);
}
}
