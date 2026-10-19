"""Exercise bridge hooks locally; this does not connect to ROS2 or PX4."""

import csim
from csim_bridge import PhysicalWrench


class LocalHoverBridge:
    def start(self, model_config):
        self.wrench = PhysicalWrench(model_config["mass"] * model_config["gravity"])
        self.reset()

    def publish_state(self, state):
        self.latest_state = state

    def receive_wrench(self, simulation_time):
        if self.latest_state is None or self.latest_state["time"] != simulation_time:
            return None
        return self.wrench

    def reset(self):
        self.latest_state = None

    def close(self):
        self.reset()


def main():
    model = csim.DroneModel(timestep=.002)
    data = csim.make_data(model, position_W=[0, 0, 1])
    bridge = LocalHoverBridge()
    try:
        bridge.start(csim.get_config(model))
        for _ in range(100):
            state = csim.get_state(model, data)
            bridge.publish_state(state)
            wrench = bridge.receive_wrench(state["time"])
            if wrench is None:
                raise TimeoutError("Local bridge has no input for this simulation tick")
            csim.step(model, data, **wrench.as_control())
        print(csim.get_state(model, data))
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
