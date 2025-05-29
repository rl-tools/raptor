from enum import Enum
from copy import copy
import asyncio

class DroneState(Enum):
    DISARMED = "disarmed"
    FLYING = "flying"
    LANDING = "landing"

class Drone:
    def __init__(self):
        self.state = DroneState.DISARMED
    def odometry_callback(self, position, velocity):
        pass
    def command(self, position, velocity):
        self._forward_command(position, velocity)

class SimulatedDrone(Drone):
    def __init__(self):
        super().__init__()
    
    def forward_command(self, position, velocity):
        pass
    async def run(self):
        while True:
            await asyncio.sleep(1)
            self.odometry_callback([0, 0, 0], [0, 0, 0])
    
async def main():
    drone = SimulatedDrone()
    await drone.run()
# if __name__ == "__main__":
#     asyncio.run(main())


from copy import copy
import numpy as np
import asyncio, websockets, json
import l2f
from l2f import vector8 as vector
from foundation_model import QuadrotorPolicy
class Simulator:

    def __init__(self):
        self.policy = QuadrotorPolicy()
        self.device = l2f.Device()
        self.rng = vector.VectorRng()
        self.env = vector.VectorEnvironment()
        self.ui = l2f.UI()
        self.params = vector.VectorParameters()
        self.state = vector.VectorState()
        self.observation = np.zeros((self.env.N_ENVIRONMENTS, self.env.OBSERVATION_DIM), dtype=np.float32)
        self.next_state = vector.VectorState()

        vector.initialize_rng(self.device, self.rng, 0)
        vector.initialize_environment(self.device, self.env)
        vector.sample_initial_parameters(self.device, self.env, self.params, self.rng)
        vector.sample_initial_state(self.device, self.env, self.params, self.state, self.rng)

    async def run(self):
        uri = "ws://localhost:13337/backend" # connection to the UI server
        async with websockets.connect(uri) as websocket:
            handshake = json.loads(await websocket.recv(uri))
            assert(handshake["channel"] == "handshake")
            namespace = handshake["data"]["namespace"]
            self.ui.ns = namespace
            ui_message = vector.set_ui_message(self.device, self.env, self.ui)
            parameters_message = vector.set_parameters_message(self.device, self.env, self.params, self.ui)
            await websocket.send(ui_message)
            await websocket.send(parameters_message)
            self.policy.reset()
            while True:
                vector.observe(self.device, self.env, self.params, self.state, self.observation, self.rng)
                action = self.policy.evaluate_step(self.observation[:, :22])
                dts = vector.step(self.device, self.env, self.params, self.state, action, self.next_state, self.rng)
                self.state.assign(self.next_state)
                ui_state = copy(self.state)
                for i, s in enumerate(ui_state.states):
                    s.position[0] += i * 0.1 # Spacing for visualization
                state_action_message = vector.set_state_action_message(self.device, self.env, self.params, self.ui, ui_state, action)
                await websocket.send(state_action_message)
                await asyncio.sleep(dts[-1])

if __name__ == "__main__":
    simulator = Simulator()
    asyncio.run(simulator.run())