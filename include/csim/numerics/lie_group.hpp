#pragma once

namespace csim::numerics {

// Reserved type only, NOT implemented. See the mechanical-system/payload paper
// linked in docs/numerical-integration.md; this is a family, not a fixed tableau.
// TODO: Select RKMK or commutator-free scheme and its order before defining step.
//       Separate manifold State from tangent/algebra derivative; define group
//       action, exp/retraction, stage composition and coupled Euclidean updates.
//       Preserve the project's Hamilton wxyz, body-rate, body-to-world convention.
//       Verify constant-rate exact reference, varying noncommuting rotations,
//       order and manifold residuals for SO(3)/S^2 as applicable.
//       A single exp(dt*omega) with frozen omega is NOT general fourth order.
//       Manifold preservation does NOT automatically imply symplecticity or
//       exact energy conservation, nor enforce all coupled cable constraints.
template <typename State, typename Geometry>
class LieGroupIntegrator;

} // namespace csim::numerics
