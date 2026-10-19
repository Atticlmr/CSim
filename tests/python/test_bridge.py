import copy
from dataclasses import FrozenInstanceError
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import csim
from csim_bridge import BridgePlugin, PhysicalWrench, available_bridges, load_bridge


class LocalBridge:
    def __init__(self, thrust):
        self.wrench = PhysicalWrench(thrust)
        self.state = None
        self.config = None

    def start(self, model_config):
        self.config = copy.deepcopy(model_config)
        self.reset()

    def publish_state(self, state):
        self.state = copy.deepcopy(state)

    def receive_wrench(self, simulation_time):
        if self.state is None or self.state["time"] != simulation_time:
            return None
        return self.wrench

    def reset(self):
        self.state = None

    def close(self):
        self.reset()


class BridgeTests(unittest.TestCase):
    def test_import_has_no_native_or_transport_dependencies(self):
        package_root = str(Path(__file__).resolve().parents[2] / "bindings/python")
        script = (
            "import sys; "
            f"sys.path.insert(0, {package_root!r}); "
            "import csim_bridge; "
            "assert not {'csim', 'rclpy', 'pymavlink', 'px4_msgs'} & set(sys.modules)"
        )
        subprocess.run([sys.executable, "-I", "-S", "-c", script], check=True, timeout=10)

    def test_wrench_validation_and_independent_control_arguments(self):
        torque = [1, 2, 3]
        wrench = PhysicalWrench(4, torque)
        torque[0] = 99
        control = wrench.as_control()
        control["torque_B"][0] = 100
        self.assertEqual(wrench.as_control(), {"thrust": 4., "torque_B": [1., 2., 3.]})
        with self.assertRaises(FrozenInstanceError):
            wrench.thrust = 0
        for thrust in (-1, math.nan, math.inf):
            with self.subTest(thrust=thrust), self.assertRaises(ValueError):
                PhysicalWrench(thrust)
        for torque in ([0, 0], [0, 0, 0, 0], [0, math.nan, 0], [0, 0, math.inf]):
            with self.subTest(torque=torque), self.assertRaises(ValueError):
                PhysicalWrench(0, torque)

    def test_discovery_is_lazy_and_loading_does_not_start_transport(self):
        factory = Mock(return_value=LocalBridge(12))
        entry = SimpleNamespace(name="local", load=Mock(return_value=factory))
        with patch("csim_bridge.entry_points", return_value=(entry,)) as discover:
            self.assertEqual(available_bridges(), ("local",))
            entry.load.assert_not_called()
            plugin = load_bridge("local", thrust=12)
            discover.assert_called_with(group="csim.bridges")
        factory.assert_called_once_with(thrust=12)
        self.assertIsInstance(plugin, BridgePlugin)
        self.assertIsNone(plugin.config)

    def test_missing_duplicate_and_incomplete_plugins_fail_explicitly(self):
        with patch("csim_bridge.entry_points", return_value=()):
            with self.assertRaisesRegex(LookupError, "not installed"):
                load_bridge("px4")
        entry = SimpleNamespace(name="local", load=Mock(return_value=lambda: object()))
        with patch("csim_bridge.entry_points", return_value=(entry, entry)):
            with self.assertRaisesRegex(ValueError, "Multiple"):
                load_bridge("local")
            entry.load.assert_not_called()
        with patch("csim_bridge.entry_points", return_value=(entry,)):
            with self.assertRaisesRegex(TypeError, "must implement"):
                load_bridge("local")

    def test_bridge_hooks_drive_all_drone_models_without_advancing_on_missing_input(self):
        root = Path(__file__).resolve().parents[2] / "examples/models/rigid_payload"
        models = (
            csim.DroneModel(),
            csim.SuspendedPayloadModel(),
            csim.load_suspended_model(str(root / "drone.urdf"), str(root / "cable.json"),
                                      str(root / "payload.urdf")),
        )
        for model in models:
            with self.subTest(model=type(model).__name__):
                mass = model.mass if hasattr(model, "mass") else model.drone_mass + model.payload_mass
                thrust = mass * model.gravity
                data = csim.make_data(model) if hasattr(model, "mass") else csim.make_data(model, thrust=thrust)
                bridge = LocalBridge(thrust)
                try:
                    bridge.start(csim.get_config(model))
                    self.assertIsNone(bridge.receive_wrench(0))
                    self.assertEqual(csim.get_state(model, data)["time"], 0)
                    for _ in range(3):
                        before = csim.get_state(model, data)
                        bridge.publish_state(before)
                        before["position_W"][0] = 99
                        self.assertNotEqual(bridge.state["position_W"][0], 99)
                        wrench = bridge.receive_wrench(before["time"])
                        self.assertIsInstance(wrench, PhysicalWrench)
                        csim.step(model, data, **wrench.as_control())
                        self.assertIsNone(bridge.receive_wrench(csim.get_state(model, data)["time"]))
                    self.assertAlmostEqual(csim.get_state(model, data)["time"], 3 * model.timestep)
                    initial = {} if hasattr(model, "mass") else {"thrust": thrust}
                    csim.reset(model, data, **initial)
                    bridge.reset()
                    self.assertIsNone(bridge.receive_wrench(0))
                finally:
                    bridge.close()
                    bridge.close()


if __name__ == "__main__":
    unittest.main()
