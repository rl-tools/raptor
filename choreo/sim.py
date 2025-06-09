from copy import copy
from muxify import Muxify
import asyncio
import sys
import numpy as np
from drone import DroneState
from simulator import Simulator, SimulatedDrone

class Behavior:
    def __init__(self, clients):
        self.clients = clients
        self.initial_positions = []

    async def run(self):
        for i, client in enumerate(self.clients):
            while client.position is None or client.velocity is None:
                await asyncio.sleep(0.1)
            self.initial_positions.append(copy(client.position))
        for client in self.clients:
            client.change_state(DroneState.FLYING)
            self.distribute_commands()
            await asyncio.sleep(0.10)
        while True:
            self.distribute_commands()
            await asyncio.sleep(0.1)
    
    def distribute_commands(self):
        for client_i, client in enumerate(self.clients):
            target_position = [*self.initial_positions[client_i][:2], 1]
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
                self.initial_positions[client_i] = target_position + np.random.uniform(-0.01, 0.01, size=3)
            client.command(target_position, [0, 0, 0])


async def main():
    global simulator
    simulator = Simulator()
    mux = Muxify(simulator.num_drones()+1)
    sys.stdout = mux[0]
    clients = [SimulatedDrone(simulator, name=f"{i}", stdout=mux[i+1]) for i in range(simulator.num_drones())]
    behavior = Behavior(clients)
    tasks = [
        asyncio.create_task(behavior.run()),
        asyncio.create_task(simulator.run()),
        *[asyncio.create_task(c.run()) for c in clients],
    ]
    await asyncio.sleep(1)
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())