#pragma once
#include <csim/math/matrix.hpp>

namespace csim::numerics {
template<class State> struct StateCodec;
template<> struct StateCodec<double> {
    static constexpr std::size_t size=1;
    static math::Matrix<1,1> pack(double value) { return math::Matrix<1,1>{value}; }
    static double unpack(const math::Matrix<1,1>& value,double) { return value(0,0); }
};
template<std::size_t count> struct StateCodec<math::Matrix<count,1>> {
    static constexpr std::size_t size=count;
    static math::Matrix<count,1> pack(const math::Matrix<count,1>& value) { return value; }
    static math::Matrix<count,1> unpack(const math::Matrix<count,1>& value,const math::Matrix<count,1>&) { return value; }
};
}
