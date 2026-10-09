#pragma once
#include <csim/numerics/state_codec.hpp>
#include <csim/dynamics/pendulum.hpp>
#include <csim/dynamics/rigid_payload.hpp>

namespace csim::numerics {
template<> struct StateCodec<dynamics::PendulumState> {
    static constexpr std::size_t size=2;
    static math::Matrix<2,1> pack(const dynamics::PendulumState& state) { return math::Matrix<2,1>{state.angle,state.angular_velocity}; }
    static dynamics::PendulumState unpack(const math::Matrix<2,1>& value,const dynamics::PendulumState&) { return {value(0,0),value(1,0)}; }
};
template<> struct StateCodec<dynamics::DroneState> {
    static constexpr std::size_t size=13;
    static math::Matrix<13,1> pack(const dynamics::DroneState& state) {
        return math::Matrix<13,1>{state.position_W.x,state.position_W.y,state.position_W.z,
            state.velocity_W.x,state.velocity_W.y,state.velocity_W.z,state.q_WB.w,state.q_WB.x,state.q_WB.y,state.q_WB.z,
            state.angular_velocity_B.x,state.angular_velocity_B.y,state.angular_velocity_B.z};
    }
    static dynamics::DroneState unpack(const math::Matrix<13,1>& value,const dynamics::DroneState&) {
        return {{value(0,0),value(1,0),value(2,0)},{value(3,0),value(4,0),value(5,0)},
            {value(6,0),value(7,0),value(8,0),value(9,0)},{value(10,0),value(11,0),value(12,0)}};
    }
};
template<> struct StateCodec<dynamics::RigidPayloadState> {
    static constexpr std::size_t size=26;
    static math::Matrix<26,1> pack(const dynamics::RigidPayloadState& state) {
        const auto drone=StateCodec<dynamics::DroneState>::pack(state.drone),payload=StateCodec<dynamics::DroneState>::pack(state.payload);
        math::Matrix<26,1> result;
        for (std::size_t index=0;index<13;++index) { result(index,0)=drone(index,0); result(index+13,0)=payload(index,0); }
        return result;
    }
    static dynamics::RigidPayloadState unpack(const math::Matrix<26,1>& value,const dynamics::RigidPayloadState& reference) {
        math::Matrix<13,1> drone,payload;
        for (std::size_t index=0;index<13;++index) { drone(index,0)=value(index,0); payload(index,0)=value(index+13,0); }
        return {StateCodec<dynamics::DroneState>::unpack(drone,reference.drone),StateCodec<dynamics::DroneState>::unpack(payload,reference.payload),reference.slack};
    }
};
template<> struct StateCodec<dynamics::SuspendedPayloadState> {
    static constexpr std::size_t size=25;
    static math::Matrix<25,1> pack(const dynamics::SuspendedPayloadState& state) {
        math::Matrix<25,1> result;
        const auto drone=StateCodec<dynamics::DroneState>::pack(state.drone);
        for (std::size_t index=0;index<13;++index) result(index,0)=drone(index,0);
        std::size_t index=13;
        for (const auto& vector:{state.cable_direction_W,state.cable_angular_velocity_W,state.payload_position_W,state.payload_velocity_W})
            for (double component:{vector.x,vector.y,vector.z}) result(index++,0)=component;
        return result;
    }
    static dynamics::SuspendedPayloadState unpack(const math::Matrix<25,1>& value,const dynamics::SuspendedPayloadState& reference) {
        math::Matrix<13,1> drone;
        for (std::size_t index=0;index<13;++index) drone(index,0)=value(index,0);
        auto result=reference; result.drone=StateCodec<dynamics::DroneState>::unpack(drone,reference.drone);
        result.cable_direction_W={value(13,0),value(14,0),value(15,0)};
        result.cable_angular_velocity_W={value(16,0),value(17,0),value(18,0)};
        result.payload_position_W={value(19,0),value(20,0),value(21,0)};
        result.payload_velocity_W={value(22,0),value(23,0),value(24,0)};
        return result;
    }
};
}
