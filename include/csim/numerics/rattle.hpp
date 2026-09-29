#pragma once

namespace csim::numerics {

// Reserved type only, NOT implemented. Constrained mechanical State contains
// generalized position and momentum; it is NOT an arbitrary ODE state.
// TODO: Define step with mass/inverse-mass, force, holonomic g(q)=0 and its
//       Jacobian G(q), multiplier solves, tolerances and iteration limits.
//       Enforce BOTH position and velocity constraints G(q)*M^-1*p=0.
//       Test constrained pendulum, constraint residuals, order two and energy.
//       RATTLE is not equivalent to normalizing the cable direction after RK4.
//       Standard guarantees concern conservative, regular constrained systems;
//       thrust/drag and slack/taut transitions need separate modeling/events.
//       No assumption about a physical cable or its tension belongs in this class.
template <typename MechanicalState>
class Rattle;

} // namespace csim::numerics
