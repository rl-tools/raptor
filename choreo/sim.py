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

    async def run(self):
        while True:
            await asyncio.sleep(0.1)

class SimulatedDrone(Drone):
    def __init__(self, simulator):
        super().__init__()
        self.simulator = simulator
        self.command_sink = None
        simulator.register_client(self)

    def set_command_sink(self, command_sink):
        self.command_sink = command_sink
    
    def forward_command(self, position, velocity):
        if self.command_sink:
            self.command_sink(position, velocity)
    

from copy import copy
import numpy as np
import asyncio, websockets, json
import l2f
from l2f import vector8 as vector
from foundation_model import QuadrotorPolicy
class Simulator:

    def __init__(self, MAX_POSITION_ERROR=0.5, MAX_VELOCITY_ERROR=0.5):
        self.MAX_POSITION_ERROR = MAX_POSITION_ERROR
        self.MAX_VELOCITY_ERROR = MAX_VELOCITY_ERROR
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
        self.setpoints = np.zeros((self.env.N_ENVIRONMENTS, 6), dtype=np.float32)
        self.clients = []
    def num_drones(self):
        return self.env.N_ENVIRONMENTS
    
    def register_client(self, client):
        if len(self.clients) >= self.num_drones():
            raise ValueError("Maximum number of drones reached")
        drone_id = len(self.clients)
        self.clients.append(client)
        client.set_command_sink(lambda position, velocity: self.command_sink(drone_id, position, velocity))
    
    def command_sink(self, drone_id, position, velocity):
        self.setpoints[drone_id, :3] = position
        self.setpoints[drone_id, 3:6] = velocity

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
                self.observation[:, :3] -= self.setpoints[:, :3]
                self.observation[:, :3] = np.clip(self.observation[:, :3], -self.MAX_POSITION_ERROR, self.MAX_POSITION_ERROR)
                self.observation[:, 3+4:3+4+3] -= self.setpoints[:, 3:3+3]
                self.observation[:, 3+4:3+4+3] = np.clip(self.observation[:, 3+4:3+4+3], -self.MAX_VELOCITY_ERROR, self.MAX_VELOCITY_ERROR)
                action = self.policy.evaluate_step(self.observation[:, :22])
                dts = vector.step(self.device, self.env, self.params, self.state, action, self.next_state, self.rng)
                self.state.assign(self.next_state)
                ui_state = copy(self.state)
                for i, s in enumerate(ui_state.states):
                    s.position[0] += i * 0.1 # Spacing for visualization
                state_action_message = vector.set_state_action_message(self.device, self.env, self.params, self.ui, ui_state, action)
                for i, client in enumerate(self.clients):
                    client.odometry_callback(ui_state.states[i].position, ui_state.states[i].linear_velocity)
                await websocket.send(state_action_message)
                await asyncio.sleep(dts[-1])

async def main():
    simulator = Simulator()
    simulator.setpoints[:, :3] = np.random.randn(simulator.num_drones(), 3).astype(np.float32) / 10
    clients = [SimulatedDrone(simulator) for _ in range(simulator.num_drones())]
    simulator_promise = simulator.run()
    tasks = [
        asyncio.create_task(simulator.run()),
        *[asyncio.create_task(c.run()) for c in clients],
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())