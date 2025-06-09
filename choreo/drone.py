import asyncio

from enum import Enum
class DroneState(Enum):
    DISARMED = "disarmed"
    FLYING = "flying"
    LANDING = "landing"

class Drone:
    def __init__(self, name=None, stdout=None):
        self.stdout = stdout
        self.state = DroneState.DISARMED
        self.position = None
        self.velocity = None
        print(f"Client {name if name else '{unnamed}'} initialized", file=self.stdout)
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
        print(f"cmd {'  '.join([f'{float(p):.2}' for p in position])}", file=self.stdout)
        self._forward_command(position, velocity)

    async def run(self):
        while True:
            await asyncio.sleep(0.1)