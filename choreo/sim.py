from copy import copy
from muxify import Muxify
import asyncio
import sys
import numpy as np
from simulator import Simulator, SimulatedDrone
import matplotlib.pyplot as plt
import json
import l2f
import os
import deadman
from trajectories.lissajous_uniform import lissajous_uniform, plot_lissajous
from crazyflie import Crazyflie, swarm_factory
from px4 import PX4
from betaflight import Betaflight
from m5stampfly import M5StampFly
from mocap import Vicon
import cflib
import signal
np.random.seed(42)

class Behavior:
    def __init__(self, clients, lissajous_parameters={}, spacing=None, height=0.3):
        self.clients = clients
        self.initial_positions = []
        self.position_offsets = []
        self.lissajous_parameters = lissajous_parameters
        assert len(self.clients) > 0, "At least one client is required"
        self.spacing = spacing if spacing is not None else np.array([0, *np.ones(len(clients)-1)])
        self.height = height

    async def run(self):
        print("Waiting for deadman trigger")
        while not deadman.trigger:
            await asyncio.sleep(0.1)
        for i, client in enumerate(self.clients):
            while client.position is None or client.velocity is None:
                await asyncio.sleep(0.1)
            self.initial_positions.append(copy(client.position))
        self.initial_positions = np.array(self.initial_positions)
        self.target_positions = self.initial_positions.copy()
        self.target_positions[:, 2] = self.height
        self.initial_target_positions = self.target_positions.copy()
        self.target_velocities = np.zeros_like(self.target_positions)
        for client in self.clients:
            await client.arm()
            self.send_commands()
            await asyncio.sleep(0.10)
        print("Clients armed")
        tick = 0
        EPSILON = 0.20
        print("Waiting for clients to reach initial positions")
        while not all([np.linalg.norm(client.position - self.target_positions[i]) < EPSILON for i, client in enumerate(self.clients)]):
            self.send_commands()
            await asyncio.sleep(0.1)
            tick += 1
        print("Starting behavior")
        t = 0
        dt = 0.1
        cumsum_spacing = np.cumsum(self.spacing)
        while True:
            in_trajectory = np.sum(t > cumsum_spacing)
            print(f"In trajectory: {in_trajectory}, t: {t:.2f}, tick: {tick}")
            self.target_positions[in_trajectory:] = self.initial_target_positions[:-in_trajectory] if in_trajectory > 0 else self.initial_target_positions
            for i, client in enumerate(self.clients):
                if t > cumsum_spacing[i]:
                    self.target_positions[i], self.target_velocities[i] = lissajous_uniform(t - cumsum_spacing[i], **self.lissajous_parameters, z=self.height)
            self.send_commands()
            await asyncio.sleep(dt)
            tick += 1
            t += dt
    
    def avoid_collisions(self, target_positions, target_velocities):
        new_target_positions = []
        new_target_velocities = []
        drones_in_avoidance = 0
        for client_i, client in enumerate(self.clients):
            target_position, target_velocity = target_positions[client_i], target_velocities[client_i]
            min_distance = None
            for second_client in filter(lambda c: c != client, self.clients):
                if client.position is not None and second_client.position is not None:
                    distance = np.linalg.norm(np.array(client.position) - np.array(second_client.position))
                    safety_distance = max(client.safety_distance, second_client.safety_distance)
                    if distance < safety_distance and (min_distance is None or distance < min_distance):
                        min_distance = distance
                        displacement = second_client.position - client.position
                        while np.linalg.norm(displacement) < 1e-6:
                            displacement = np.random.uniform(-1, 1, size=3)
                        target_position = client.position - (safety_distance*1.2 - distance) / 2 * displacement / np.linalg.norm(displacement)
                        target_position[2] = self.height
                        # print(f"Collision avoidance: {client} and {second_client}")
            if min_distance is not None:
                print(f"Min dist: {min_distance:.2f}")
                self.target_positions[client_i] = target_position + np.random.uniform(-0.01, 0.01, size=3)
                drones_in_avoidance += 1
            new_target_positions.append(target_position)
            new_target_velocities.append(target_velocity)
        return drones_in_avoidance, (new_target_positions, new_target_velocities)
    
    def send_commands(self):
        # drones_in_avoidance, (target_positions, target_velocities) = self.avoid_collisions(self.target_positions, self.target_velocities)
        target_positions, target_velocities = self.target_positions, self.target_velocities
        for client, target_position, target_velocity in zip(self.clients, target_positions, target_velocities):
            client.command(target_position, target_velocity)


VICON_IP = "192.168.1.3"
async def main():
    cflib.crtp.init_drivers()
    mocap = Vicon(VICON_IP, VELOCITY_CLIP=5, EXPECTED_FRAMERATE=100)
    global simulator
    RANDOM_CLOSE_CALLS = False
    scale = 1.5
    lissajous_parameters = dict(A=1.5*scale, B=0.5*scale, duration=20)
    initial_positions = np.array([
        [0, 0., 0],
        [0, -1.25, 0],
        [-0.5, -1.5, 0],
        [-1.0, -1.5, 0],
        [-1.5, -1.5, 0],
    ])
    spacing = np.array([2, 2, 1.5, 0.75, 1.25])
    PLOT = False
    # PLOT = True
    if PLOT:
        plt.figure(figsize=(6, 6))
        t_vals = np.linspace(0, lissajous_parameters["duration"], 1000)
        coords = np.array([lissajous_uniform(t, **lissajous_parameters)[0] for t in t_vals])
        vels = np.array([lissajous_uniform(t, **lissajous_parameters)[1] for t in t_vals])
        plt.plot(t_vals, coords[:, 0], label="x")
        plt.plot(t_vals, vels[:, 0], label="x")
        plt.plot(t_vals, coords[:, 1], label="y")
        plt.plot(t_vals, vels[:, 1], label="y")
        plt.show()
        plt.figure(figsize=(6, 6))
        plot_lissajous(just_add=True, **lissajous_parameters)
        plt.scatter(initial_positions[:, 0], initial_positions[:, 1], label="Initial Positions", color='red')
        plt.show()
    simulator = Simulator(N_DRONES=len(initial_positions))
    for state, initial_position in zip(simulator.state.states, initial_positions):
        state.position = initial_position
    MUX = False
    mux = Muxify(simulator.num_drones()+1) if MUX else [sys.stdout for _ in range(simulator.num_drones()+1)]
    sys.stdout = mux[0]
    simulator_clients = [SimulatedDrone(simulator, name=f"{i}", stdout=mux[i+1]) for i in range(simulator.num_drones())]

    def set_parameters(id, name):
        parameters = json.loads(l2f.parameters_to_json(simulator.device, simulator.env.environments[id], simulator.params.parameters[id]))
        with open(os.path.join(os.path.dirname(__file__), "parameters", f"{name}.json"), "r") as f:
            parameters["dynamics"] = json.load(f)["dynamics"]
        l2f.parameters_from_json(simulator.device, simulator.env.environments[0], json.dumps(parameters), simulator.params.parameters[id])
    
    set_parameters(0, "x500")
    set_parameters(1, "race")
    set_parameters(2, "crazyflie")
    set_parameters(3, "crazyflie")

    crazyflie_configs = [
        {
            "name": "crazyflie_bl",
            "type": Crazyflie,
            "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E9"},
            "mocap": "crazyflie_bl",
        },
        {
            "name": "crazyflie",
            "type": Crazyflie,
            "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E7"},
            "mocap": "crazyflie",
        },
    ]
    USE_PX4 = True
    # USE_PX4 = False
    USE_CRAZYFLIES = True
    # USE_CRAZYFLIES = False
    USE_BETAFLIGHT = True
    # USE_BETAFLIGHT = False
    # USE_M5STAMPFLY = True
    USE_M5STAMPFLY = False
    crazyflies = []
    if USE_CRAZYFLIES:
        crazyflies = swarm_factory(crazyflie_configs) #[cfg["type"](**cfg["kwargs"]) for cfg in crazyflie_configs]
        for cfg, crazyflie in zip(crazyflie_configs, crazyflies):
            mocap.add(cfg["mocap"], crazyflie._mocap_callback)
            crazyflie.learned_controller = True
    px4_configs = [
        {
            "name": "race",
            "type": PX4,
            "kwargs": {"uri": "tcp:192.168.8.4:5760"},
            "mocap": "race_jonas",
        },
    ]
    px4s = []
    if USE_PX4:
        for cfg in px4_configs:
            px4 = PX4(name=cfg["name"], **cfg["kwargs"])
            px4s.append(px4)
            mocap.add(cfg["mocap"], px4._mocap_callback)
    
    betaflight_configs = [
        {
            "name": "hummingbird",
            "type": Betaflight,
            "kwargs": {"uri": "/dev/serial/by-name/elrs-transmitter2", "BAUD": 921600, "rate": 50, "odometry_source": "mocap"},
            "mocap": "hummingbird",
        },
        {
            "name": "savagebee_pusher",
            "type": Betaflight,
            "kwargs": {"uri": "/dev/serial/by-name/elrs-transmitter1", "BAUD": 921600, "rate": 50, "odometry_source": "mocap"},
            "mocap": "savagebee_pusher",
        },
    ]
    betaflights = []
    if USE_BETAFLIGHT:
        betaflights = [Betaflight(**cfg["kwargs"]) for cfg in betaflight_configs]
        for cfg, betaflight in zip(betaflight_configs, betaflights):
            mocap.add(cfg["mocap"], betaflight._mocap_callback)

    m5stampfly_configs = [
        {
            "name": "m5stampfly",
            "type": Betaflight,
            "kwargs": {"uri": "/dev/serial/by-name/m5stamp-forwarder", "BAUD": 115200, "rate": 50, "odometry_source": "mocap"},
            "mocap": "m5stampfly",
        },
    ]
    m5stampflies = []
    if USE_M5STAMPFLY:
        m5stampflies = [M5StampFly(**cfg["kwargs"]) for cfg in m5stampfly_configs]
        for cfg, m5stampfly in zip(m5stampfly_configs, m5stampflies):
            mocap.add(cfg["mocap"], m5stampfly._mocap_callback)

    # clients = [*px4s, *crazyflies]
    # clients = [*simulator_clients[:-len(clients)], *clients]
    # clients = simulator_clients

    # clients = [crazyflies[0], m5stampflies[0], crazyflies[1], betaflights[0]]
    # clients = [m5stampflies[0], simulator_clients[1], simulator_clients[2], simulator_clients[3]]
    # clients = [px4s[0], simulator_clients[1], simulator_clients[2], simulator_clients[3]]
    # clients = [simulator_clients[0], simulator_clients[1], simulator_clients[2], simulator_clients[3], simulator_clients[4], simulator_clients[5]]
    # clients = [px4s[0], simulator_clients[1], simulator_clients[2], simulator_clients[3], simulator_clients[4], simulator_clients[5]]
    clients = [px4s[0], betaflights[1], crazyflies[0], crazyflies[1], betaflights[0]]
    
    behavior = Behavior(clients, lissajous_parameters=lissajous_parameters, spacing=spacing, height=0.5)
    async def loop():
        tick = 0
        dt = 0.01
        while True:
            for i, client in enumerate(clients):
                if client.simulated or client.position is None or client.velocity is None:
                    continue
                simulator.state.states[i].position = client.position
                simulator.state.states[i].linear_velocity = client.velocity
                simulator.state.states[i].orientation = client.orientation
                simulator.state.states[i].angular_velocity = np.array([0, 0, 0])
                
            await asyncio.sleep(dt)
            tick += 1
    tasks = [
        asyncio.create_task(deadman.monitor(type="foot-pedal")),
        asyncio.create_task(behavior.run()),
        asyncio.create_task(simulator.run()),
        *[asyncio.create_task(c.run()) for c in clients],
        asyncio.create_task(loop())
    ]
    await asyncio.sleep(1)
    await asyncio.gather(*tasks)

class MocapDummy:
    def __init__(self):
        pass
    def _mocap_callback(self, timestamp, position, orientation, velocity, reset_counter):
        pass


async def main_mocap_test():
    mocap = Vicon(VICON_IP, VELOCITY_CLIP=5, EXPECTED_FRAMERATE=100)
    for name in ["hummingbird", "m5stampfly", "crazyflie", "crazyflie_bl", "race_jonas"]:
        mocap.add(name, MocapDummy()._mocap_callback)
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
    # asyncio.run(main_mocap_test())


