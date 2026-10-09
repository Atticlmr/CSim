#pragma once
#include <csim/numerics/dormand_prince.hpp>
#include <csim/numerics/state_codec.hpp>
#include <csim/math/lu.hpp>
#include <optional>

namespace csim::numerics {
struct NewtonOptions {
    double atol=1e-12,rtol=1e-10;
    std::size_t max_iterations=20;
    void validate() const {
        if (!std::isfinite(atol) || atol<=0 || !std::isfinite(rtol) || rtol<0
            || !max_iterations || max_iterations>1000) throw std::invalid_argument("Invalid Newton options");
    }
};
namespace implicit_detail {
template<std::size_t count,class Residual>
math::Matrix<count,1> newton(math::Matrix<count,1> value,Residual&& residual,const NewtonOptions& options) {
    options.validate();
    auto norm=[&](const math::Matrix<count,1>& error,const math::Matrix<count,1>& candidate) {
        detail::checkComputedState(error); detail::checkComputedState(candidate);
        double result=0;
        for (std::size_t row=0;row<count;++row)
            result=std::max(result,std::abs(error(row,0))/(options.atol+options.rtol*std::abs(candidate(row,0))));
        if (!std::isfinite(result)) throw IntegrationFailure("Newton residual is not representable");
        return result;
    };
    for (std::size_t iteration=0;iteration<options.max_iterations;++iteration) {
        const auto error=residual(value);
        const double current=norm(error,value);
        if (current<=1) return value;
        math::Matrix<count,count> jacobian;
        for (std::size_t column=0;column<count;++column) {
            auto probe=value;
            probe(column,0)+=std::sqrt(std::numeric_limits<double>::epsilon())*std::max(1.,std::abs(value(column,0)));
            const double difference=probe(column,0)-value(column,0);
            if (!std::isfinite(difference) || difference==0) throw IntegrationFailure("Newton perturbation cannot be resolved");
            const auto changed=residual(probe);
            for (std::size_t row=0;row<count;++row) jacobian(row,column)=(changed(row,0)-error(row,0))/difference;
        }
        math::Matrix<count,1> correction;
        try { correction=math::PartialPivLU<count>(jacobian).solve(error); }
        catch (const math::SingularMatrixError&) { throw IntegrationFailure("Newton Jacobian is singular"); }
        bool accepted=false;
        for (double damping=1;damping>=1./1024;damping*=.5) {
            const auto candidate=value-correction*damping;
            if (norm(residual(candidate),candidate)<current) { value=candidate; accepted=true; break; }
        }
        if (!accepted) throw IntegrationFailure("Newton line search failed");
    }
    const auto error=residual(value);
    if (norm(error,value)<=1) return value;
    throw IntegrationFailure("Newton iteration limit exceeded");
}
}
template<class State,class Control,class Dynamics,class Codec=StateCodec<State>>
State implicitThetaStep(double time,const State& state,const Control& control,double dt,
                        Dynamics&& dynamics,double theta,const NewtonOptions& options={}) {
    detail::validateExplicitTime(time,dt);
    if (theta!=1 && theta!=.5) throw std::invalid_argument("Implicit theta must be 1 or 0.5");
    detail::checkComputedState(state);
    const auto initial=Codec::pack(state);
    const auto initial_rate=Codec::pack(dynamics(time,state,control));
    auto residual=[&](const auto& values) {
        const auto stage=initial*(1-theta)+values*theta;
        return values-initial-Codec::pack(dynamics(time+dt*theta,Codec::unpack(stage,state),control))*dt;
    };
    return Codec::unpack(implicit_detail::newton(initial+initial_rate*dt,residual,options),state);
}
template<class State,class Codec=StateCodec<State>> class RadauIIA5 {
public:
    explicit RadauIIA5(NewtonOptions options={}):options_(options) { options_.validate(); }
    template<class Control,class Dynamics>
    State step(double time,const State& state,const Control& control,double dt,Dynamics&& dynamics) const {
        detail::validateExplicitTime(time,dt); detail::checkComputedState(state);
        constexpr auto count=Codec::size;
        const double root=std::sqrt(6.);
        const std::array<double,3> nodes{(4-root)/10,(4+root)/10,1};
        const double coefficients[3][3]={
            {(88-7*root)/360,(296-169*root)/1800,(-2+3*root)/225},
            {(296+169*root)/1800,(88+7*root)/360,(-2-3*root)/225},
            {(16-root)/36,(16+root)/36,1./9}};
        const auto initial=Codec::pack(state);
        const auto rate=Codec::pack(dynamics(time,state,control));
        math::Matrix<3*count,1> guess;
        for (std::size_t stage=0;stage<3;++stage)
            for (std::size_t row=0;row<count;++row) guess(stage*count+row,0)=initial(row,0)+dt*nodes[stage]*rate(row,0);
        auto residual=[&](const math::Matrix<3*count,1>& values) {
            std::array<math::Matrix<count,1>,3> stages,rates;
            for (std::size_t stage=0;stage<3;++stage) {
                for (std::size_t row=0;row<count;++row) stages[stage](row,0)=values(stage*count+row,0);
                rates[stage]=Codec::pack(dynamics(time+dt*nodes[stage],Codec::unpack(stages[stage],state),control));
            }
            math::Matrix<3*count,1> result;
            for (std::size_t stage=0;stage<3;++stage) {
                auto error=stages[stage]-initial;
                for (std::size_t column=0;column<3;++column) error-=rates[column]*(dt*coefficients[stage][column]);
                for (std::size_t row=0;row<count;++row) result(stage*count+row,0)=error(row,0);
            }
            return result;
        };
        const auto solved=implicit_detail::newton(guess,residual,options_);
        math::Matrix<count,1> final;
        for (std::size_t row=0;row<count;++row) final(row,0)=solved(2*count+row,0);
        return Codec::unpack(final,state);
    }
private:
    NewtonOptions options_;
};
template<class State,class Codec=StateCodec<State>> class Bdf {
public:
    Bdf(double time,State state,unsigned order=2,NewtonOptions options={})
        :time_(time),state_(std::move(state)),order_(order),options_(options) {
        if ((order!=1 && order!=2) || !std::isfinite(time)) throw std::invalid_argument("BDF order must be 1 or 2, with finite time");
        options_.validate(); detail::checkComputedState(state_);
    }
    double time() const { return time_; }
    const State& state() const { return state_; }
    void reset(double time,State state) {
        if (!std::isfinite(time)) throw std::invalid_argument("BDF time must be finite");
        detail::checkComputedState(state);
        time_=time; state_=std::move(state); previous_.reset(); previous_dt_=0;
    }
    template<class Control,class Dynamics> void step(const Control& control,double dt,Dynamics&& dynamics) {
        detail::validateExplicitTime(time_,dt);
        State candidate=state_;
        if (order_==1 || !previous_) {
            candidate=implicitThetaStep<State,Control,Dynamics&,Codec>(time_,state_,control,dt,dynamics,order_==1 ? 1 : .5,options_);
        } else {
            const double ratio=dt/previous_dt_;
            if (!std::isfinite(ratio) || ratio>2) throw std::invalid_argument("BDF2 step ratio must not exceed 2");
            const double coefficient=(1+2*ratio)/(1+ratio);
            const auto initial=Codec::pack(state_),previous=Codec::pack(*previous_);
            const auto constant=initial*(1+ratio)-previous*(ratio*ratio/(1+ratio));
            auto residual=[&](const auto& values) {
                return values*coefficient-constant-Codec::pack(dynamics(time_+dt,Codec::unpack(values,state_),control))*dt;
            };
            candidate=Codec::unpack(implicit_detail::newton(initial,residual,options_),state_);
        }
        detail::checkComputedState(candidate);
        previous_=state_; state_=std::move(candidate); previous_dt_=dt; time_+=dt;
    }
private:
    double time_;
    State state_;
    unsigned order_;
    NewtonOptions options_;
    std::optional<State> previous_;
    double previous_dt_=0;
};
}
