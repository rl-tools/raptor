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
from trajectories.lissajous_uniform import lissajous_uniform, plot_lissajous
np.random.seed(42)

class Behavior:
    def __init__(self, clients, lissajous_parameters={}, spacing=None):
        self.clients = clients
        self.initial_positions = []
        self.position_offsets = []
        self.lissajous_parameters = lissajous_parameters
        assert len(self.clients) > 0, "At least one client is required"
        self.spacing = spacing if spacing is not None else np.array([0, *np.ones(len(clients)-1)])

    async def run(self):
        for i, client in enumerate(self.clients):
            while client.position is None or client.velocity is None:
                await asyncio.sleep(0.1)
            self.initial_positions.append(copy(client.position))
        self.initial_positions = np.array(self.initial_positions)
        self.target_positions = self.initial_positions.copy()
        self.target_positions[:, 2] += 1
        self.initial_target_positions = self.target_positions.copy()
        self.target_velocities = np.zeros_like(self.target_positions)
        for client in self.clients:
            await client.arm()
            self.send_commands()
            await asyncio.sleep(0.10)
        tick = 0
        EPSILON = 0.15
        while not all([np.linalg.norm(client.position - self.target_positions[i]) < EPSILON for i, client in enumerate(self.clients)]):
            self.send_commands()
            await asyncio.sleep(0.1)
            tick += 1
        t = 0
        dt = 0.1
        cumsum_spacing = np.cumsum(self.spacing)
        while True:
            in_trajectory = np.sum(t > cumsum_spacing)
            self.target_positions[in_trajectory:] = self.initial_target_positions[:-in_trajectory] if in_trajectory > 0 else self.initial_target_positions
            for i, client in enumerate(self.clients):
                if t > cumsum_spacing[i]:
                    self.target_positions[i], self.target_velocities[i] = lissajous_uniform(t - cumsum_spacing[i], **self.lissajous_parameters)
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
                        target_position[2] = 1
                        # print(f"Collision avoidance: {client} and {second_client}")
            if min_distance is not None:
                print(f"Min dist: {min_distance:.2f}", file=client.stdout)
                self.target_positions[client_i] = target_position + np.random.uniform(-0.01, 0.01, size=3)
                drones_in_avoidance += 1
            new_target_positions.append(target_position)
            new_target_velocities.append(target_velocity)
        return drones_in_avoidance, (new_target_positions, new_target_velocities)
    
    def send_commands(self):
        drones_in_avoidance, (target_positions, target_velocities) = self.avoid_collisions(self.target_positions, self.target_velocities)
        for client, target_position, target_velocity in zip(self.clients, target_positions, target_velocities):
            client.command(target_position, target_velocity)



async def main():
    global simulator
    RANDOM_CLOSE_CALLS = False
    scale = 1.5
    lissajous_parameters = dict(A=1.5*scale, B=0.5*scale, duration=20)
    initial_positions = np.array([
        [0, 0., 0],
        [0, -1.25, 0],
        [-0.5, -1.5, 0],
        [-1.0, -1.5, 0]
    ])
    spacing = np.array([0, 2.5, 2, 1])
    # PLOT = False
    plt.figure(figsize=(6, 6))
    t_vals = np.linspace(0, lissajous_parameters["duration"], 1000)
    coords = np.array([lissajous_uniform(t, **lissajous_parameters)[0] for t in t_vals])
    vels = np.array([lissajous_uniform(t, **lissajous_parameters)[1] for t in t_vals])
    plt.plot(t_vals, coords[:, 0], label="x")
    plt.plot(t_vals, vels[:, 0], label="x")
    plt.plot(t_vals, coords[:, 1], label="y")
    plt.plot(t_vals, vels[:, 1], label="y")
    plt.show()
    PLOT = True
    if PLOT:
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
    clients = [SimulatedDrone(simulator, name=f"{i}", stdout=mux[i+1]) for i in range(simulator.num_drones())]
    def set_parameters(id, name):
        parameters = json.loads(l2f.parameters_to_json(simulator.device, simulator.env.environments[id], simulator.params.parameters[id]))
        with open(os.path.join(os.path.dirname(__file__), "parameters", f"{name}.json"), "r") as f:
            parameters["dynamics"] = json.load(f)["dynamics"]
        l2f.parameters_from_json(simulator.device, simulator.env.environments[0], json.dumps(parameters), simulator.params.parameters[id])
    
    set_parameters(0, "x500")
    set_parameters(1, "race")
    set_parameters(2, "crazyflie")
    set_parameters(3, "crazyflie")
    
    # clients[0].safety_distance = 0.5
    behavior = Behavior(clients, lissajous_parameters=lissajous_parameters, spacing=spacing)
    async def loop():
        tick = 0
        dt = 0.01
        while True:
            if RANDOM_CLOSE_CALLS and tick % 100 == 0:
                min1 = np.random.randint(0, len(clients))
                min2 = np.random.choice([i for i in range(len(clients)) if i != min1])
                if tick == 100 * 3:
                    positions = np.array([s.position for s in simulator.state.states])
                    distances = np.linalg.norm(positions[:, None] - positions, axis=2)
                    distances[np.arange(len(positions)), np.arange(len(positions))] = np.inf
                    flat_index = np.argmin(distances)
                    min1, min2 = np.unravel_index(flat_index, distances.shape)
                pos1, pos2 = np.array(simulator.state.states[min1].position), np.array(simulator.state.states[min2].position)
                diff = pos2 - pos1
                simulator.state.states[min1].position = pos1 + diff * 0.25
                simulator.state.states[min2].position = pos2 - diff * 0.25
                
            await asyncio.sleep(dt)
            tick += 1
    tasks = [
        asyncio.create_task(behavior.run()),
        asyncio.create_task(simulator.run()),
        *[asyncio.create_task(c.run()) for c in clients],
        asyncio.create_task(loop())
    ]
    await asyncio.sleep(1)
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())