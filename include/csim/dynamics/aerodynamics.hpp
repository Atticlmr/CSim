#pragma once
#include <csim/math/matrix.hpp>
#include <cmath>
#include <string>
#include <stdexcept>

namespace csim::dynamics {
// Deterministic W-frame field: mean + affine spatial variation + sinusoidal gust.
// Evaluated at each body's own position and every RK stage time, never wall time.
struct WindField {
    math::Vector3 velocity_W{}, reference_W{}, gust_amplitude_W{};
    math::Matrix3 gradient_W{}; // (m/s)/m; row = velocity component
    double gust_frequency=0, gust_phase=0; // Hz, radians
    void validate() const {
        if (!velocity_W.isFinite()||!reference_W.isFinite()||!gust_amplitude_W.isFinite()
            ||!gradient_W.isFinite()||!std::isfinite(gust_frequency)||gust_frequency<0||!std::isfinite(gust_phase))
            throw std::invalid_argument("Wind parameters must be finite; gust frequency must be nonnegative");
    }
    math::Vector3 at(math::Vector3 position,double time) const {
        if (!position.isFinite()||!std::isfinite(time)) throw std::invalid_argument("Wind sample requires finite position/time");
        auto value=velocity_W;
        bool spatial=false;
        for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) spatial|=gradient_W(i,j)!=0;
        if (spatial) value+=gradient_W*(position-reference_W);
        if (gust_amplitude_W.squaredNorm()!=0) {
            const double phase=2*std::acos(-1.0)*gust_frequency*time+gust_phase;
            if (!std::isfinite(phase)) throw std::overflow_error("Wind gust phase overflow");
            value+=gust_amplitude_W*std::sin(phase);
        }
        if (!value.isFinite()) throw std::overflow_error("Wind field overflow");
        return value;
    }
};
struct AirLoads {
    math::Vector3 wind_velocity_W{}, air_velocity_W{};
    math::Vector3 linear_force_W{}, quadratic_force_W{}, spad_force_W{};
    math::Vector3 total_force_W{}, wind_induced_force_W{};
    double air_power=0, mechanical_power=0;
};
struct DragConfig {
    double k1=0, k2=0, k0=0; // kg/s, kg/m, N. Quadratic force includes 1/2.
    std::string sign_mode="exact"; // componentwise sign in fixed W axes
    double epsilon_v=0; // m/s, required positive for tanh; zero for exact
    bool enabled() const noexcept { return k1!=0||k2!=0||k0!=0; }
    void validate(bool payload=true) const {
        for (double k:{k1,k2,k0}) if (!std::isfinite(k)||k<0) throw std::invalid_argument("Drag coefficients must be finite and nonnegative");
        if (!payload&&k0!=0) throw std::invalid_argument("SPAD is supported only for the point payload");
        if (sign_mode!="exact"&&sign_mode!="tanh") throw std::invalid_argument("SPAD sign_mode must be exact or tanh");
        if (!std::isfinite(epsilon_v)||(sign_mode=="tanh" ? epsilon_v<=0 : epsilon_v!=0))
            throw std::invalid_argument("tanh requires positive epsilon_v; exact requires epsilon_v=0");
    }
    AirLoads forces(math::Vector3 velocity,math::Vector3 wind) const {
        if (!velocity.isFinite()||!wind.isFinite()) throw std::invalid_argument("Air velocities must be finite");
        AirLoads result; result.wind_velocity_W=wind; result.air_velocity_W=velocity-wind;
        if (!result.air_velocity_W.isFinite()) throw std::overflow_error("Relative air velocity overflow");
        auto terms=[&](math::Vector3 v,AirLoads& out) {
            if (k1!=0) out.linear_force_W=v*(-k1);
            if (k2!=0) out.quadratic_force_W=v*(-.5*k2*v.norm());
            if (k0!=0) {
                auto sign=[&](double x) { return sign_mode=="tanh" ? std::tanh(x/epsilon_v) : double((x>0)-(x<0)); };
                out.spad_force_W={-k0*sign(v.x),-k0*sign(v.y),-k0*sign(v.z)};
            }
            out.total_force_W=out.linear_force_W+out.quadratic_force_W+out.spad_force_W;
            if (!out.linear_force_W.isFinite()||!out.quadratic_force_W.isFinite()||!out.spad_force_W.isFinite()||!out.total_force_W.isFinite())
                throw std::overflow_error("Aerodynamic force overflow");
        };
        if (enabled()) {
            terms(result.air_velocity_W,result);
            AirLoads still_air; terms(velocity,still_air);
            // Diagnostic counterfactual difference, NOT an additional applied force.
            result.wind_induced_force_W=result.total_force_W-still_air.total_force_W;
            result.air_power=result.total_force_W.dot(result.air_velocity_W);
            result.mechanical_power=result.total_force_W.dot(velocity);
            if (!result.wind_induced_force_W.isFinite()||!std::isfinite(result.air_power)||!std::isfinite(result.mechanical_power))
                throw std::overflow_error("Aerodynamic power overflow");
        }
        return result;
    }
};
} // namespace csim::dynamics
