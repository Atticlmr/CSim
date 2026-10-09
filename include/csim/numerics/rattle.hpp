#pragma once
#include <csim/numerics/implicit.hpp>

namespace csim::numerics {
template<std::size_t coordinates,std::size_t constraints=1> class Rattle {
public:
    using Vector=math::Matrix<coordinates,1>;
    explicit Rattle(Vector inverse_mass,NewtonOptions options={})
        :inverse_mass_(inverse_mass),options_(options) {
        options_.validate();
        for (std::size_t index=0;index<coordinates;++index)
            if (!std::isfinite(inverse_mass(index,0)) || inverse_mass(index,0)<=0)
                throw std::invalid_argument("RATTLE requires finite positive inverse masses");
    }
    template<class Force,class Constraint,class Jacobian>
    std::pair<Vector,Vector> step(double time,const Vector& position,const Vector& momentum,double dt,
                                 Force&& force,Constraint&& constraint,Jacobian&& jacobian) const {
        detail::validateExplicitTime(time,dt);
        detail::checkComputedState(position); detail::checkComputedState(momentum);
        auto maximum=[](const auto& value) {
            detail::checkComputedState(value);
            double result=0;
            for (std::size_t row=0;row<value.rows;++row) result=std::max(result,std::abs(value(row,0)));
            return result;
        };
        auto velocity=[&](Vector value) {
            for (std::size_t row=0;row<coordinates;++row) value(row,0)*=inverse_mass_(row,0);
            return value;
        };
        const auto initial_gradient=jacobian(position);
        const double tolerance=options_.atol;
        if (maximum(constraint(position))>tolerance*100 || maximum(initial_gradient*velocity(momentum))>tolerance*100)
            throw std::invalid_argument("RATTLE initial position/momentum violate constraints");
        auto half=momentum+force(time,position)*(dt*.5);
        Vector next=position+velocity(half)*dt;
        bool converged=false;
        for (std::size_t iteration=0;iteration<options_.max_iterations;++iteration) {
            const auto error=constraint(next);
            if (maximum(error)<=tolerance) { converged=true; break; }
            auto weighted=initial_gradient.transposed();
            for (std::size_t row=0;row<coordinates;++row)
                for (std::size_t column=0;column<constraints;++column) weighted(row,column)*=inverse_mass_(row,0)*dt;
            math::Matrix<constraints,1> multiplier;
            try { multiplier=math::PartialPivLU<constraints>(jacobian(next)*weighted).solve(error); }
            catch (const math::SingularMatrixError&) { throw IntegrationFailure("RATTLE position Jacobian is singular"); }
            half-=initial_gradient.transposed()*multiplier;
            next=position+velocity(half)*dt;
        }
        if (!converged && maximum(constraint(next))>tolerance) throw IntegrationFailure("RATTLE position iteration limit exceeded");
        auto final=half+force(time+dt,next)*(dt*.5);
        const auto gradient=jacobian(next);
        auto weighted=gradient.transposed();
        for (std::size_t row=0;row<coordinates;++row)
            for (std::size_t column=0;column<constraints;++column) weighted(row,column)*=inverse_mass_(row,0);
        try { final-=gradient.transposed()*math::PartialPivLU<constraints>(gradient*weighted).solve(gradient*velocity(final)); }
        catch (const math::SingularMatrixError&) { throw IntegrationFailure("RATTLE momentum Jacobian is singular"); }
        detail::checkComputedState(next); detail::checkComputedState(final);
        if (maximum(gradient*velocity(final))>tolerance*100) throw IntegrationFailure("RATTLE momentum constraint unresolved");
        return {next,final};
    }
private:
    Vector inverse_mass_;
    NewtonOptions options_;
};
}
