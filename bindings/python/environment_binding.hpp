#pragma once
#include <csim/dynamics/aerodynamics.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace csim::binding {
inline std::array<double,3> triple(math::Vector3 v) { return {v.x,v.y,v.z}; }
inline pybind11::dict airLoads(const dynamics::AirLoads& a) {
    pybind11::dict d;
    d["wind_velocity_W"]=triple(a.wind_velocity_W); d["air_velocity_W"]=triple(a.air_velocity_W);
    d["linear_force_W"]=triple(a.linear_force_W); d["quadratic_force_W"]=triple(a.quadratic_force_W);
    d["spad_force_W"]=triple(a.spad_force_W); d["total_force_W"]=triple(a.total_force_W);
    d["wind_induced_force_W"]=triple(a.wind_induced_force_W);
    d["air_power"]=a.air_power; d["mechanical_power"]=a.mechanical_power;
    return d;
}
inline pybind11::dict dragConfig(const dynamics::DragConfig& c) {
    pybind11::dict d; d["k1"]=c.k1; d["k2"]=c.k2; d["k0"]=c.k0;
    d["sign_mode"]=c.sign_mode; d["epsilon_v"]=c.epsilon_v; return d;
}
inline pybind11::dict windConfig(const dynamics::WindField& w) {
    pybind11::dict d; d["velocity_W"]=triple(w.velocity_W); d["reference_W"]=triple(w.reference_W);
    d["gust_amplitude_W"]=triple(w.gust_amplitude_W); d["gust_frequency"]=w.gust_frequency; d["gust_phase"]=w.gust_phase;
    std::array<std::array<double,3>,3> g{};
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) g[i][j]=w.gradient_W(i,j);
    d["gradient_W"]=g; return d;
}
}
void bindEnvironment(pybind11::module_&);
void bindModelConfig(pybind11::module_&);
