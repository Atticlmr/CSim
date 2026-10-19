"""Transport-independent bridge boundary in CSim's units and coordinates."""

from dataclasses import dataclass
import math
from typing import Any, Mapping, Protocol, runtime_checkable


@dataclass(frozen=True)
class PhysicalWrench:
    """Applied +Z_B thrust (N) and CoM torque (N m), in the FLU body frame."""

    thrust: float
    torque_B: tuple[float, float, float] = (0., 0., 0.)

    def __post_init__(self):
        thrust = float(self.thrust)
        torque = tuple(float(value) for value in self.torque_B)
        if not math.isfinite(thrust) or thrust < 0:
            raise ValueError("Thrust must be finite and nonnegative")
        if len(torque) != 3 or not all(math.isfinite(value) for value in torque):
            raise ValueError("torque_B must contain three finite values")
        object.__setattr__(self, "thrust", thrust)
        object.__setattr__(self, "torque_B", torque)

    def as_control(self):
        """Return keyword arguments for the atomic csim.step(model, data, ...) API."""
        return {"thrust": self.thrust, "torque_B": list(self.torque_B)}


@runtime_checkable
class BridgePlugin(Protocol):
    """The runner owns Data; plugins exchange detached configuration and truth."""

    def start(self, model_config: Mapping[str, Any]) -> None:
        """Open transport and initialize sensor/actuator adapters from get_config."""
        ...

    def publish_state(self, state: Mapping[str, Any]) -> None:
        """Consume a get_state snapshot; derive protocol measurements in the plugin."""
        ...

    def receive_wrench(self, simulation_time: float) -> PhysicalWrench | None:
        """Poll without blocking; return None when no valid input is ready for this tick."""
        ...

    def reset(self) -> None:
        """Begin a new session at simulation time zero and discard stale queued data."""
        ...

    def close(self) -> None:
        """Release transport and workers; repeated close calls must be harmless."""
        ...
