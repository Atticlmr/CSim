#pragma once

namespace csim::numerics {

// Reserved types only, NOT implemented or constructible. Method signatures wait
// for nonlinear-solver/Jacobian contracts; see docs/numerical-integration.md.

// TODO: Three-stage fifth-order Radau IIA for stiff explicit ODEs x' = f(t,x,u).
//       Define step/reset, Jacobian, Newton/linear solver policies, tolerances,
//       adaptive acceptance and failure diagnostics. Never accept failed Newton.
//       Test stiff decay accuracy/stability and convergence failures.
//       This ODE interface does NOT imply support for high-index constraint DAEs.
template <typename State>
class RadauIIA5;

// TODO: Stateful implicit BDF ODE solver. Start with BDF1/BDF2; variable order
//       requires its own coefficients, history, startup and error controller.
//       Define initialize/step/reset; reset history after events/state changes.
//       Plan Jacobian/Newton/linear solves and report nonlinear failure explicitly.
//       DAE support would require a separate residual F(t,x,xdot,u)=0 contract,
//       consistent initial conditions and an index policy; not just this ODE API.
template <typename State>
class Bdf;

} // namespace csim::numerics
