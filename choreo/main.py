import argparse
import struct
import cflib.crtp
from cflib.crazyflie import Crazyflie as CrazyflieCFLib
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from cflib.crazyflie.commander import SET_SETPOINT_CHANNEL, META_COMMAND_CHANNEL, TYPE_HOVER 
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
import numpy as np
import time
import asyncio

def send_learned_policy_packet(cf):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.channel = META_COMMAND_CHANNEL
    pk.data = struct.pack('<B', 1)
    cf.send_packet(pk)


class Crazyflie:
    def __init__(self, uri='radio://0/80/2M/E7E7E7E7E7'):
        cflib.crtp.init_drivers()
        self.scf = SyncCrazyflie(uri, cf=CrazyflieCFLib())
        self.scf.open_link()
        self.position = None
        logconf = LogConfig(name="Choreo", period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        self.scf.cf.log.add_config(logconf)

        def log_callback(timestamp, data, logconf):
            x = data['stateEstimate.x']
            y = data['stateEstimate.y']
            z = data['stateEstimate.z']
            self.position = np.array([x, y, z])
            print(f"Position: {self.position}")
        logconf.data_received_cb.add_callback(log_callback)
        logconf.start()
        self.main()
    
    async def arm(self):
        print("Requesting arming")
        self.scf.cf.platform.send_crash_recovery_request()
        asyncio.sleep(1.0)
        self.scf.cf.platform.send_arming_request(True)
        asyncio.sleep(1.0)
    
    async def main(self):
        while True:
            await asyncio.sleep(0.1)
            # self.position = self.scf.cf.position_estimator._position
            # print(f"Position: {self.position}")


    async def goto(self, target, distance_threshold=0.05):
        distance = None
        while distance is None or distance > distance_threshold:
            if self.position is not None:
                distance = np.linalg.norm(target - self.position)
                position = target
                orientation = np.array([0, 0, 0, 1])
                linear_velocity = np.zeros(3)
                angular_velocity = np.zeros(3)
                linear_acceleration = np.zeros(3)
                self.scf.cf.commander.send_full_state_setpoint(position, linear_velocity, linear_acceleration, orientation, *angular_velocity)
                # diff = (target - self.position) / 2
                # print(f"Diff: {diff}")
                # self.scf.cf.commander.send_velocity_world_setpoint(*diff, 0)
            else:
                print("Position not available yet")
            await asyncio.sleep(0.1)

    async def disarm(self):
        print("Requesting disarming")
        self.scf.cf.commander.send_stop_setpoint()
        self.scf.cf.commander.send_notify_setpoint_stop()
        self.scf.cf.platform.send_arming_request(False)



async def main():
    cf = Crazyflie()
    loop = asyncio.get_event_loop()
    await cf.arm()
    await cf.goto(np.array([0.0, 0.0, 0.4]))
    await cf.goto(np.array([0.0, 0.0, 0.0]))
    await cf.disarm()
    asyncio.sleep(1000)

if __name__ == '__main__':
    asyncio.run(main())