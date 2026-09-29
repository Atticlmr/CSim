#pragma once

#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace csim::numerics::detail {

template <typename State>
bool stateIsFinite(const State& state) {
    if constexpr (std::is_floating_point_v<State>) {
        return std::isfinite(state);
    } else {
        return state.isFinite();
    }
}

template <typename State>
void checkComputedState(const State& state) {
    if (!stateIsFinite(state)) {
        throw std::overflow_error("Integration stage, derivative or result is not finite");
    }
}

} // namespace csim::numerics::detail
