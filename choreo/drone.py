import asyncio
from enum import Enum
import numpy as np
import time

class Drone:
    def __init__(self, name=None, stdout=None):
        self.stdout = stdout
        self.position = None
        self.velocity = None
        self.simulated = False
        print(f"Client {name if name else '{unnamed}'} initialized", file=self.stdout)
        self.name = name
    def _odometry_callback(self, position, velocity):
        self.position = position
        self.velocity = velocity
    async def arm(self):
        await self._arm()
    async def disarm(self):
        await self._disarm()
    def command(self, position, velocity):
        # print(f"cmd {'  '.join([f'{float(p):.2}' for p in position])}", file=self.stdout)
        self._forward_command(position, velocity)

    async def goto(self, target_input, distance_threshold=0.15, timeout=None, relative=True):
        print(f"Going to {target_input}")
        distance = None
        start = time.time()
        while distance is None or distance > distance_threshold or (timeout is not None and time.time() - start < timeout) and not self.disarmed:
            if self.position is not None:
                target = target_input if relative else target_input
                distance = np.linalg.norm(target - self.position)
                # print(f"Distance to target: {distance:.2f} m", file=mux[4])
                self.command(target, [0, 0, 0])
            else:
                print("Position not available yet")
            await asyncio.sleep(0.1)

    async def land(self):
        while self.position is None:
            print("Land: Position not available yet")
            await asyncio.sleep(0.1)
        target_position = self.position.copy()
        target_position[2] = 0.0
        await self.goto(target_position, distance_threshold=0.1)

    async def run(self):
        while True:
            await asyncio.sleep(0.1)