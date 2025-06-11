import struct
import asyncio
import time
import numpy as np

from cflib.crazyflie import Crazyflie as CrazyflieCFLib
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
import cflib.crtp
from cflib.crazyflie.commander import SET_SETPOINT_CHANNEL, META_COMMAND_CHANNEL, TYPE_HOVER 
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.console import Console

import deadman
from mux import mux

from drone import Drone



def send_learned_policy_packet(cf):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.channel = META_COMMAND_CHANNEL
    pk.data = struct.pack('<B', 1)
    cf.send_packet(pk)

class Crazyflie(Drone):
    def __init__(self, uri='radio://0/80/2M/E7E7E7E7E7'):
        cflib.crtp.init_drivers()
        self.scf = SyncCrazyflie(uri, cf=CrazyflieCFLib())
        self.scf.open_link()
        self.position = None
        logconf = LogConfig(name="Choreo", period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stateEstimate.vx', 'float')
        logconf.add_variable('stateEstimate.vy', 'float')
        logconf.add_variable('stateEstimate.vz', 'float')
        self.scf.cf.log.add_config(logconf)

        self.position_number = 0
        self.safety_distance = 0.15
        def log_callback(timestamp, data, logconf):
            x, y, z, vx, vy, vz = [data[f"stateEstimate.{f}"] for f in ['x', 'y', 'z', 'vx', 'vy', 'vz']]
            position = np.array([x, y, z])
            velocity = np.array([vx, vy, vz])
            self._odometry_callback(position, velocity)

            if self.position_number % 10 == 0:
                print(f"log  pos: {self.position[0]:.2f} {self.position[1]:.2f} {self.position[2]:.2f} {1/np.mean(self.pose_callback_dts):.2f} Hz", file=mux[2])
            self.position_number += 1
            # self.position = np.array([x, y, z])
            # print(f"Position: {self.position}")
            # print(f"State: {sm}")
        logconf.data_received_cb.add_callback(log_callback)
        logconf.start()
        console = Console(self.scf.cf)
        logfile = open("console.log", "a")
        def console_callback(text):
            logfile.write(text)
            logfile.flush()
        console.receivedChar.add_callback(console_callback)
        self.learned_controller = False
        loop = asyncio.get_event_loop()
        loop.create_task(self.main())
        self.disarmed = False
        self.last_pose_callback = None
        self.pose_callback_dts = []
        self.POSITION_ERROR_CLIP = 0.3
    def _mocap_callback(self, msg):
        now = time.time()
        if self.last_pose_callback is not None and (now - self.last_pose_callback < 0.1):
            return
        if self.last_pose_callback is not None:
            dt = now - self.last_pose_callback
            self.pose_callback_dts.append(dt)
            self.pose_callback_dts = self.pose_callback_dts[-100:]
        self.last_pose_callback = now
        pose = msg['pose']
        position = [pose['position']['x'], pose['position']['y'], pose['position']['z']]
        self.orientation = [pose['orientation']['w'], pose['orientation']['x'], pose['orientation']['y'], pose['orientation']['z']]
        # print(f"vicon pos: {position[0]:.2f} {position[1]:.2f} {position[2]:.2f} {1/np.mean(self.pose_callback_dts):.2f} Hz", file=mux[1])
        self.scf.cf.extpos.send_extpose(
            pose['position']['x'],
            pose['position']['y'],
            pose['position']['z'],
            pose['orientation']['x'],
            pose['orientation']['y'],
            pose['orientation']['z'],
            pose['orientation']['w']
        )
    
    async def arm(self):
        print("Requesting arming")
        await asyncio.sleep(1.0)
        self.scf.cf.platform.send_crash_recovery_request()
        await asyncio.sleep(1.0)
        print("arming", file=mux[2])
        self.scf.cf.platform.send_arming_request(True)
        await asyncio.sleep(1.0)
    
    async def main(self):
        while True:
            if not deadman.trigger:
                # print("disarming", file=mux[3])
                self.scf.cf.platform.send_arming_request(False)
            elif self.learned_controller:
                send_learned_policy_packet(self.scf.cf)
            await asyncio.sleep(0.05)
            # self.position = self.scf.cf.position_estimator._position
            # print(f"Position: {self.position}")
    def _forward_command(self, position, velocity):
            orientation = np.array([0, 0, 0, 1])
            angular_velocity = np.zeros(3)
            linear_acceleration = np.zeros(3)
            if self.position is not None:
                diff = position - self.position
                diff = np.clip(diff, -self.POSITION_ERROR_CLIP, self.POSITION_ERROR_CLIP)
                position = self.position + diff
            print(f"cmd {'  '.join([f'{float(p):.2}' for p in position])}", file=mux[3])
            self.scf.cf.commander.send_full_state_setpoint(position, velocity, linear_acceleration, orientation, *angular_velocity)

    async def goto(self, target_input, distance_threshold=0.15, timeout=None, relative=True):
        print(f"Going to {target_input}")
        distance = None
        start = time.time()
        while distance is None or distance > distance_threshold or (timeout is not None and time.time() - start < timeout) and not self.disarmed:
            if self.position is not None:
                target = target_input if relative else target_input
                distance = np.linalg.norm(target - self.position)
                print(f"Distance to target: {distance:.2f} m", file=mux[4])
                position = target
                orientation = np.array([0, 0, 0, 1])
                linear_velocity = np.zeros(3)
                angular_velocity = np.zeros(3)
                linear_acceleration = np.zeros(3)
                self.scf.cf.commander.send_full_state_setpoint(position, linear_velocity, linear_acceleration, orientation, *angular_velocity)
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

    async def disarm(self):
        print("Requesting disarming")
        self.scf.cf.commander.send_stop_setpoint()
        self.scf.cf.commander.send_notify_setpoint_stop()
        print("disarming", file=mux[2])
        self.scf.cf.platform.send_arming_request(False)