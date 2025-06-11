import asyncio
from enum import Enum

class Drone:
    def __init__(self, name=None, stdout=None):
        self.stdout = stdout
        self.position = None
        self.velocity = None
        print(f"Client {name if name else '{unnamed}'} initialized", file=self.stdout)
    def odometry_callback(self, position, velocity):
        self.position = position
        self.velocity = velocity
    async def arm(self):
        await self._arm()
    async def disarm(self):
        await self._arm()
    def command(self, position, velocity):
        # print(f"cmd {'  '.join([f'{float(p):.2}' for p in position])}", file=self.stdout)
        self._forward_command(position, velocity)

    async def run(self):
        while True:
            await asyncio.sleep(0.1)