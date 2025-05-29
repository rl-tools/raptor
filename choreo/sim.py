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
        self.position = None
        self.velocity = None
    def odometry_callback(self, position, velocity):
        self.position = position
        self.velocity = velocity
    def change_state(self, new_state):
        old_state = self.state
        self.state_transition(old_state, new_state)
        self.state = new_state
    def state_transition(self, old_state, new_state):
        if old_state == DroneState.DISARMED and new_state == DroneState.FLYING:
            self._arm()
        elif old_state == DroneState.FLYING and new_state == DroneState.LANDING:
            self._land()
        elif new_state == DroneState.DISARMED:
            self._disarm()
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
        self.arm_sink = None
        self.disarm_sink = None

        simulator.register_client(self)

    def set_command_sink(self, command_sink):
        self.command_sink = command_sink
    def set_arm_sink(self, arm_sink):
        self.arm_sink = arm_sink
    def set_disarm_sink(self, disarm_sink):
        self.disarm_sink = disarm_sink

    def _arm(self):
        if self.arm_sink is not None:
            self.arm_sink()
        else:
            raise ValueError("Arm sink not set")
    def _disarm(self):
        if self.disarm_sink is not None:
            self.disarm_sink()
        else:
            raise ValueError("Disarm sink not set")
    def _disarm_callback(self):
        self.change_state(DroneState.DISARMED)

    def _odometry_callback(self, position, velocity):
        self.odometry_callback(position, velocity)
    
    def _forward_command(self, position, velocity):
        if self.command_sink:
            self.command_sink(position, velocity)
    

from copy import copy
import numpy as np
import asyncio, websockets, json
import l2f
from l2f import vector256 as vector
from foundation_model import QuadrotorPolicy
import time
class Simulator:

    def __init__(self, MAX_POSITION_ERROR=0.5, MAX_VELOCITY_ERROR=0.5, ARMING_TIMEOUT=5):
        self.MAX_POSITION_ERROR = MAX_POSITION_ERROR
        self.MAX_VELOCITY_ERROR = MAX_VELOCITY_ERROR
        self.ARMING_TIMEOUT = ARMING_TIMEOUT
        self.policy = QuadrotorPolicy()
        self.device = l2f.Device()
        self.rng = vector.VectorRng()
        self.env = vector.VectorEnvironment()
        self.ui = l2f.UI()
        self.params = vector.VectorParameters()
        self.state = vector.VectorState()
        self.observation = np.zeros((self.env.N_ENVIRONMENTS, self.env.OBSERVATION_DIM), dtype=np.float32)
        self.next_state = vector.VectorState()
        self.armed = np.zeros(self.env.N_ENVIRONMENTS, dtype=np.bool_)
        self.arming_times = [None for _ in range(self.env.N_ENVIRONMENTS)]

        vector.initialize_rng(self.device, self.rng, 0)
        vector.initialize_environment(self.device, self.env)
        vector.sample_initial_parameters(self.device, self.env, self.params, self.rng)
        vector.sample_initial_state(self.device, self.env, self.params, self.state, self.rng)
        for state in self.state.states:
            state.position[2] = 0.0
        self.setpoints = np.zeros((self.env.N_ENVIRONMENTS, 6), dtype=np.float32)
        self.clients = []
    def num_drones(self):
        return self.env.N_ENVIRONMENTS
    
    def register_client(self, client):
        if len(self.clients) >= self.num_drones():
            raise ValueError("Maximum number of drones reached")
        drone_id = len(self.clients)
        client._odometry_callback(self.state.states[drone_id].position, self.state.states[drone_id].linear_velocity)
        self.clients.append(client)
        client.set_command_sink(lambda position, velocity: self.command_sink(drone_id, position, velocity))
        client.set_arm_sink(lambda: self.arm_sink(drone_id))
        client.set_disarm_sink(lambda: self.disarm_sink(drone_id))
    
    def command_sink(self, drone_id, position, velocity):
        self.setpoints[drone_id, :3] = position
        self.setpoints[drone_id, 3:6] = velocity
    
    def arm_sink(self, drone_id):
        print(f"Arming drone {drone_id}")
        self.arming_times[drone_id] = time.time()
        self.armed[drone_id] = True
    def disarm_sink(self, drone_id):
        print(f"Disrming drone {drone_id}")
        self.armed[drone_id] = False
        self.arming_times[drone_id] = None


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
                for i in range(self.num_drones()):
                    if not self.armed[i]:
                        action[i, :] = 0
                dts = vector.step(self.device, self.env, self.params, self.state, action, self.next_state, self.rng)
                for i in range(self.num_drones()):
                    if self.next_state.states[i].position[2] < 0:
                        self.next_state.states[i].position[2] = 0
                        self.next_state.states[i].orientation[0] = 1
                        self.next_state.states[i].orientation[1:] = 0
                        now = time.time()
                        if  self.arming_times[i] is None or self.arming_times[i] + self.ARMING_TIMEOUT < now:
                            self.next_state.states[i].linear_velocity[2] = 0 if self.next_state.states[i].linear_velocity[2] < 0 else self.next_state.states[i].linear_velocity[2]
                            self.next_state.states[i].linear_velocity[:2] *= 0.95
                            self.next_state.states[i].angular_velocity[:] *= 0.95
                            if self.arming_times[i] is not None and self.arming_times[i] + self.ARMING_TIMEOUT < now and i < len(self.clients):
                                self.armed[i] = False
                                self.clients[i]._disarm_callback()
                self.state.assign(self.next_state)
                state_action_message = vector.set_state_action_message(self.device, self.env, self.params, self.ui, self.state, action)
                for i, client in enumerate(self.clients):
                    client._odometry_callback(self.state.states[i].position, self.state.states[i].linear_velocity)
                await websocket.send(state_action_message)
                await asyncio.sleep(dts[-1])

async def main():
    simulator = Simulator()
    # simulator.setpoints[:, :3] = np.random.randn(simulator.num_drones(), 3).astype(np.float32) / 10 
    # simulator.setpoints[:, 2] += 1
    clients = [SimulatedDrone(simulator) for _ in range(simulator.num_drones())]
    tasks = [
        asyncio.create_task(simulator.run()),
        *[asyncio.create_task(c.run()) for c in clients],
    ]
    await asyncio.sleep(10)
    for i, client in enumerate(clients):
        while client.position is None or client.velocity is None:
            await asyncio.sleep(0.1)
        client.change_state(DroneState.FLYING)
        client.command([*client.position[:2], 1], [0, 0, 0])
        await asyncio.sleep(0.05)
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())