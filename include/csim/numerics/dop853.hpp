#pragma once

#include <csim/numerics/dormand_prince.hpp>

namespace csim::numerics {
template<class State>
struct Dop853Trial {
    State state, error5, error3;
};

template<class State,class Control,class Dynamics>
Dop853Trial<State> dop853Trial(double time,const State& state,const Control& control,double dt,Dynamics&& dynamics) {
    detail::validateExplicitTime(time,dt);
    if (!detail::stateIsFinite(state)) throw std::invalid_argument("DOP853 requires finite state");
    constexpr std::array<double,12> nodes{0,.0526001519587677319,.0789002279381515978,.118350341907227397,
        .281649658092772603,1./3,.25,4./13,.651282051282051282,.6,6./7,1};
    constexpr std::array<std::array<double,12>,12> tableau{{
        {},
        {.0526001519587677319},
        {.0197250569845378995,.0591751709536136984},
        {.0295875854768068492,0,.0887627564304205475},
        {.241365134159266686,0,-.884549479328286085,.924834003261792003},
        {.037037037037037037,0,0,.170828608729473871,.125467687566822425},
        {.037109375,0,0,.170252211019544039,.0602165389804559607,-.017578125},
        {.0370920001185047927,0,0,.170383925712239994,.107262030446373285,-.0153194377486244018,.00827378916381402289},
        {.624110958716075717,0,0,-3.36089262944694129,-.868219346841726007,27.5920996994467083,20.1540675504778934,-43.4898841810699588},
        {.477662536438264366,0,0,-2.48811461997166764,-.590290826836842996,21.2300514481811942,15.2792336328824236,-33.2882109689848629,-.0203312017085086261},
        {-.937142430085987326,0,0,5.18637242884406371,1.09143734899672958,-8.14978701074692613,-18.5200656599969599,22.7394870993505043,2.49360555267965239,-3.0467644718982195},
        {2.27331014751653821,0,0,-10.5344954667372502,-2.0008720582248625,-17.9589318631187989,27.9488845294199601,-2.85899827713502369,-8.87285693353062954,12.3605671757943031,.64339274601576353}
    }};
    constexpr std::array<double,12> weights{.0542937341165687622,0,0,0,0,4.45031289275240888,1.89151789931450038,
        -5.80120396001058478,.311164366957819894,-.152160949662516079,.201365400804030348,.0447106157277725905};
    constexpr std::array<double,12> error_weights{.0131200449941948807,0,0,0,0,-1.22515644637620444,-.495758949657250192,
        1.66437718245498654,-.350328848749973682,.334179118713017479,.0819232064851157125,-.0223553078638862953};
    std::array<double,12> times{};
    for (std::size_t stage=0;stage<times.size();++stage) {
        times[stage]=time+dt*nodes[stage];
        if (!std::isfinite(times[stage])) throw std::overflow_error("DOP853 stage time overflow");
        for (std::size_t previous=0;previous<stage;++previous)
            if (times[stage]==times[previous]) throw std::overflow_error("DOP853 stage times are not distinct");
    }
    std::array<State,12> stages;
    auto evaluate=[&](double stage_time,const State& value) {
        detail::checkComputedState(value);
        State derivative=dynamics(stage_time,value,control);
        detail::checkComputedState(derivative);
        return derivative;
    };
    stages[0]=evaluate(time,state);
    for (std::size_t stage=1;stage<stages.size();++stage) {
        State rate=stages[0]*tableau[stage][0];
        for (std::size_t previous=1;previous<stage;++previous)
            if (tableau[stage][previous]!=0) rate=rate+stages[previous]*tableau[stage][previous];
        stages[stage]=evaluate(times[stage],state+rate*dt);
    }
    State rate=stages[0]*weights[0],error5=stages[0]*error_weights[0];
    for (std::size_t stage=1;stage<stages.size();++stage) {
        if (weights[stage]!=0) rate=rate+stages[stage]*weights[stage];
        if (error_weights[stage]!=0) error5=error5+stages[stage]*error_weights[stage];
    }
    State candidate=state+rate*dt;
    State error3=(rate+stages[0]*(-.244094488188976378)+stages[8]*(-.733846688281611857)
                  +stages[11]*(-.0220588235294117647))*dt;
    error5=error5*dt;
    detail::checkComputedState(candidate); detail::checkComputedState(error3); detail::checkComputedState(error5);
    return {std::move(candidate),std::move(error5),std::move(error3)};
}

template<class State>
class Dop853 {
    static_assert(std::is_nothrow_move_assignable_v<State>);
public:
    Dop853(double time,State initial,double first_step,AdaptiveStepOptions options={})
        :time_(time),state_(std::move(initial)),next_step_(first_step),options_(options) {
        if (!std::isfinite(options.min_step) || options.min_step<=0 || !std::isfinite(options.max_step)
            || options.max_step<options.min_step || !options.max_attempts)
            throw std::invalid_argument("Invalid DOP853 adaptive limits");
        validateInitial(time_,state_,first_step);
    }
    double time() const noexcept { return time_; }
    const State& state() const noexcept { return state_; }
    double nextStepSize() const noexcept { return next_step_; }
    void reset(double time,State initial,double first_step) {
        validateInitial(time,initial,first_step);
        state_=std::move(initial); time_=time; next_step_=first_step;
    }
    template<class Control,class Dynamics,class Norm>
    AdaptiveStepReport step(const Control& control,Dynamics&& dynamics,Norm&& norm,double bound) {
        if (!std::isfinite(bound) || bound<=time_) throw std::invalid_argument("Invalid DOP853 time bound");
        const double remaining=bound-time_;
        if (!std::isfinite(remaining)) throw std::overflow_error("DOP853 interval overflow");
        double interval=std::min(next_step_,remaining);
        for (std::size_t attempt=0;attempt<options_.max_attempts;++attempt) {
            const double endpoint=interval>=remaining ? bound : std::min(time_+interval,bound);
            interval=endpoint-time_;
            if (interval<=0 || time_+interval!=endpoint) throw IntegrationFailure("DOP853 time cannot advance");
            auto trial=dop853Trial(time_,state_,control,interval,dynamics);
            const double fifth=norm(std::as_const(state_),std::as_const(trial.state),std::as_const(trial.error5));
            const double third=norm(std::as_const(state_),std::as_const(trial.state),std::as_const(trial.error3));
            if (std::isnan(fifth) || std::isnan(third) || fifth<0 || third<0)
                throw std::invalid_argument("DOP853 error norm must be nonnegative and not NaN");
            const double error=!std::isfinite(fifth) || !std::isfinite(third) ? std::numeric_limits<double>::infinity()
                : fifth==0 ? 0 : fifth*(fifth/std::hypot(fifth,.1*third));
            double factor=error==0 ? 5 : std::clamp(.9*std::pow(error,-1./8),.2,5.);
            if (error<=1) {
                if (attempt) factor=std::min(factor,1.);
                const double suggested=interval>options_.max_step/factor ? options_.max_step
                    : std::clamp(interval*factor,options_.min_step,options_.max_step);
                state_=std::move(trial.state); time_=endpoint; next_step_=suggested;
                return {interval,error,attempt+1,attempt};
            }
            if (interval<=options_.min_step) throw IntegrationFailure("DOP853 minimum step reached");
            const double smaller=std::max(options_.min_step,interval*factor);
            if (!(smaller<interval)) throw IntegrationFailure("DOP853 step cannot shrink");
            interval=smaller;
        }
        throw IntegrationFailure("DOP853 attempt limit exceeded");
    }
private:
    void validateInitial(double time,const State& state,double first_step) const {
        if (!std::isfinite(time) || !detail::stateIsFinite(state) || !std::isfinite(first_step)
            || first_step<options_.min_step || first_step>options_.max_step)
            throw std::invalid_argument("Invalid DOP853 initial values");
    }
    double time_;
    State state_;
    double next_step_;
    AdaptiveStepOptions options_;
};
}
