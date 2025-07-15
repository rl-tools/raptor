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
from cflib.utils import uri_helper

import deadman
from mux import mux

from drone import Drone

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


def swarm_factory(configs):
    factory = CachedCfFactory(rw_cache='./cache')
    swarm = Swarm([cfg["kwargs"]["uri"] for cfg in configs], factory=factory)
    print(f"Connecting to {len(configs)} Crazyflies")
    swarm.open_links()
    print(f"Connected to {len(configs)} Crazyflies")
    return [Crazyflie(**cfg["kwargs"], cf=swarm._cfs[cfg['kwargs']['uri']].cf) for cfg in configs]


def send_learned_policy_packet(cf):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.channel = META_COMMAND_CHANNEL
    pk.data = struct.pack('<B', 1)
    cf.send_packet(pk)


crazyflie_registry = {
    "crazyflie": "radio://0/80/2M/E7E7E7E7E7",
    "crazyflie_bl": "radio://0/80/2M/E7E7E7E7E9",
}

class Crazyflie(Drone):
    def __init__(self, uri='radio://0/80/2M/E7E7E7E7E7', cf=None, odometry_source="mocap", **kwargs):
        super().__init__(**kwargs)
        # odometry_source: "mocap" or "log" (feedback from drone)
        if cf is None:
            cflib.crtp.init_drivers()
            self.scf = SyncCrazyflie(uri, cf=CrazyflieCFLib())
            self.scf.open_link()
            self.cf = self.scf.cf
        else:
            self.cf = cf
        self.odometry_source = odometry_source
        self.position = None
        # logconf = LogConfig(name="Choreo", period_in_ms=500)
        # logconf.add_variable('stateEstimate.x', 'float')
        # logconf.add_variable('stateEstimate.y', 'float')
        # logconf.add_variable('stateEstimate.z', 'float')
        # logconf.add_variable('stateEstimate.vx', 'float')
        # logconf.add_variable('stateEstimate.vy', 'float')
        # logconf.add_variable('stateEstimate.vz', 'float')
        # self.cf.log.add_config(logconf)

        self.position_number = 0
        self.safety_distance = 0.15
        def log_callback(timestamp, data, logconf):
            x, y, z, vx, vy, vz = [data[f"stateEstimate.{f}"] for f in ['x', 'y', 'z', 'vx', 'vy', 'vz']]
            position = np.array([x, y, z])
            velocity = np.array([vx, vy, vz])
            if self.odometry_source == "log":
                self._odometry_callback(position, velocity)

            if self.position_number % 1 == 0:
                print(f"log  pos: {position[0]:.2f} {position[1]:.2f} {position[2]:.2f}", file=mux[2])
            self.position_number += 1
            # self.position = np.array([x, y, z])
            # print(f"Position: {self.position}")
            # print(f"State: {sm}")
        # logconf.data_received_cb.add_callback(log_callback)
        # logconf.start()
        # console = Console(self.cf)
        # logfile = open("console.log", "a")
        # def console_callback(text):
        #     logfile.write(text)
        #     logfile.flush()
        # console.receivedChar.add_callback(console_callback)
        self.learned_controller = True
        loop = asyncio.get_event_loop()
        loop.create_task(self.main())
        self.disarmed = False
        self.last_pose_callback = None
        self.pose_callback_dts = []
        self.POSITION_ERROR_CLIP = 0.5
    def _mocap_callback(self, timestamp, position, orientation, velocity, reset_counter):
        self.orientation = orientation
        if self.odometry_source == "mocap":
            self._odometry_callback(position, velocity)
        now = time.time()
        if self.last_pose_callback is not None and (now - self.last_pose_callback < 0.1):
            return
        if self.last_pose_callback is not None:
            dt = now - self.last_pose_callback
            self.pose_callback_dts.append(dt)
            self.pose_callback_dts = self.pose_callback_dts[-100:]
        self.last_pose_callback = now
        # print(f"vicon pos: {position[0]:.2f} {position[1]:.2f} {position[2]:.2f} {1/np.mean(self.pose_callback_dts):.2f} Hz", file=mux[1])
    
    async def arm(self):
        print("Requesting arming")
        await asyncio.sleep(1.0)
        self.cf.platform.send_crash_recovery_request()
        await asyncio.sleep(1.0)
        print("arming", file=mux[2])
        self.cf.platform.send_arming_request(True)
        await asyncio.sleep(1.0)
    
    async def main(self):
        while True:
            if not deadman.trigger:
                # print("disarming", file=mux[3])
                self.cf.platform.send_arming_request(False)
            elif self.learned_controller:
                send_learned_policy_packet(self.cf)
            if self.position is not None and self.orientation is not None:
                self.cf.extpos.send_extpose(
                    self.position[0],
                    self.position[1],
                    self.position[2],
                    self.orientation[1],
                    self.orientation[2],
                    self.orientation[3],
                    self.orientation[0]
                )
            await asyncio.sleep(0.1)
            # self.position = self.cf.position_estimator._position
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
            self.cf.commander.send_full_state_setpoint(position, velocity, linear_acceleration, orientation, *angular_velocity)

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
                self.cf.commander.send_full_state_setpoint(position, linear_velocity, linear_acceleration, orientation, *angular_velocity)
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
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        print("disarming", file=mux[2])
        self.cf.platform.send_arming_request(False)

async def main():
    # Initialize CFLIB drivers for single Crazyflie connections
    cflib.crtp.init_drivers()
    
    # time.sleep(10)
    VICON_IP = "192.168.1.3"
    from mocap import Vicon
    target_position = [0, 0, 0.2]
    mocap = Vicon(VICON_TRACKER_IP=VICON_IP)
    instance = "crazyflie_bl"
    crazyflie = Crazyflie(uri=crazyflie_registry[instance], odometry_source="mocap")
    crazyflie._forward_command(target_position, [0, 0, 0])
    # asyncio.create_task(deadman.monitor(type="foot-pedal")),
    asyncio.create_task(deadman.monitor(type="foot-pedal")),
    crazyflie_main_task = asyncio.create_task(crazyflie.main())
    mocap.add(instance, crazyflie._mocap_callback)
    while crazyflie.position is None:
        await asyncio.sleep(0.1)
    initial_position = crazyflie.position.copy()
    target_position = initial_position + np.array([0, 0, 0.2])
    print(f"Initial position: {initial_position}")
    print(f"Target position: {target_position}")

    while True:
        crazyflie._forward_command(target_position, [0, 0, 0])
        while not deadman.trigger:
            await asyncio.sleep(0.1)
        crazyflie._forward_command(target_position, [0, 0, 0])
        timeout = asyncio.create_task(asyncio.sleep(15))
        while not timeout.done():
            distance = crazyflie.position - target_position
            print(f"Distance to target: {distance[0]:.2f} {distance[1]:.2f} {distance[2]:.2f} m")
            await asyncio.sleep(0.1)
        crazyflie._forward_command(initial_position + np.array([0, 0, -0.2]), [0, 0, 0])
        await asyncio.sleep(5)
        while deadman.trigger:
            await asyncio.sleep(0.1)
    # await betaflight.goto(initial_position, distance_threshold=0.0)
    await crazyflie_main_task # DO NOT TERMINATE, IF TERMINATED, THE DRONE DOES NOT RECEIVE FEEDBACK AND LIKELY SHOOTS INTO THE SKY

if __name__ == "__main__":
    asyncio.run(main())