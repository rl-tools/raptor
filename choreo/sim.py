from copy import copy
from muxify import Muxify
import asyncio
import sys
import numpy as np
from drone import DroneState
from simulator import Simulator, SimulatedDrone
np.random.seed(42)

class Behavior:
    def __init__(self, clients):
        self.clients = clients
        self.initial_positions = []
        self.position_offsets = []

    async def run(self):
        for i, client in enumerate(self.clients):
            while client.position is None or client.velocity is None:
                await asyncio.sleep(0.1)
            self.initial_positions.append(copy(client.position))
        self.initial_positions = np.array(self.initial_positions)
        self.target_positions = self.initial_positions.copy()
        self.target_positions[:, 2] += 1
        for client in self.clients:
            client.change_state(DroneState.FLYING)
            self.distribute_commands()
            await asyncio.sleep(0.10)
        tick = 0
        dt = 0.1
        while True:
            self.distribute_commands()
            await asyncio.sleep(dt)
            tick += 1
    
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
                    safety_distance = max(client.safety_distance(), second_client.safety_distance())
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
    
    def distribute_commands(self):
        target_velocities = np.zeros_like(self.target_positions)
        drones_in_avoidance, (target_positions, target_velocities) = self.avoid_collisions(self.target_positions, target_velocities)
        for client, target_position, target_velocity in zip(self.clients, target_positions, target_velocities):
            client.command(target_position, target_velocity)



async def main():
    global simulator
    simulator = Simulator()
    MUX = False
    mux = Muxify(simulator.num_drones()+1) if MUX else [sys.stdout for _ in range(simulator.num_drones()+1)]
    sys.stdout = mux[0]
    clients = [SimulatedDrone(simulator, name=f"{i}", stdout=mux[i+1]) for i in range(simulator.num_drones())]
    behavior = Behavior(clients)
    async def loop():
        tick = 0
        dt = 0.01
        while True:
            if tick % 100 == 0:
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