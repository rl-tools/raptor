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
from cflib.crazyflie.console import Console
import numpy as np
import time
import asyncio
import sdl2
import sdl2.ext

def send_learned_policy_packet(cf):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.channel = META_COMMAND_CHANNEL
    pk.data = struct.pack('<B', 1)
    cf.send_packet(pk)

deadman_trigger = False

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
        logconf.add_variable('rltrp.sm', "uint8_t")
        self.scf.cf.log.add_config(logconf)

        def log_callback(timestamp, data, logconf):
            x = data['stateEstimate.x']
            y = data['stateEstimate.y']
            z = data['stateEstimate.z']
            sm = data['rltrp.sm']
            self.position = np.array([x, y, z])
            # print(f"Position: {self.position}")
            # print(f"State: {sm}")
        logconf.data_received_cb.add_callback(log_callback)
        logconf.start()
        console = Console(self.scf.cf)
        def console_callback(text):
            # pass
            print(f"Console output: {text}")
        console.receivedChar.add_callback(console_callback)
        self.learned_controller = False
        loop = asyncio.get_event_loop()
        loop.create_task(self.main())
        self.disarmed = False
    
    async def arm(self):
        print("Requesting arming")
        self.scf.cf.platform.send_crash_recovery_request()
        await asyncio.sleep(1.0)
        self.scf.cf.platform.send_arming_request(True)
        await asyncio.sleep(1.0)
    
    async def main(self):
        while True:
            if not deadman_trigger:
                while True:
                    self.scf.cf.platform.send_arming_request(False)
                    await asyncio.sleep(0.02)
            elif self.learned_controller:
                send_learned_policy_packet(self.scf.cf)
            await asyncio.sleep(0.05)
            # self.position = self.scf.cf.position_estimator._position
            # print(f"Position: {self.position}")


    async def goto(self, target, distance_threshold=0.05, timeout=None):
        distance = None
        start = time.time()
        while distance is None or distance > distance_threshold or (timeout is not None and time.time() - start < timeout) and not self.disarmed:
            if self.position is not None:
                distance = np.linalg.norm(target - self.position)
                position = target
                orientation = np.array([0, 0, 0, 1])
                linear_velocity = np.zeros(3)
                angular_velocity = np.zeros(3)
                linear_acceleration = np.zeros(3)
                self.scf.cf.commander.send_full_state_setpoint(position, linear_velocity, linear_acceleration, orientation, *angular_velocity)
            else:
                print("Position not available yet")
            await asyncio.sleep(0.1)

    async def disarm(self):
        print("Requesting disarming")
        self.scf.cf.commander.send_stop_setpoint()
        self.scf.cf.commander.send_notify_setpoint_stop()
        self.scf.cf.platform.send_arming_request(False)
    


async def deadman_monitor():
    global deadman_trigger
    sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)
    sdl2.SDL_JoystickEventState(sdl2.SDL_ENABLE)
    njoy = sdl2.SDL_NumJoysticks()
    print(f"Found {njoy} joystick(s)")

    if njoy == 0:
        print("No joystick found")
        exit(1)

    joy = sdl2.SDL_JoystickOpen(0)

    event = sdl2.SDL_Event()
    while True:
        deadman_trigger_now = deadman_trigger 
        while sdl2.SDL_PollEvent(event) != 0:
            if event.type == sdl2.SDL_JOYBUTTONDOWN:
                deadman_trigger_now = True
            elif event.type == sdl2.SDL_JOYBUTTONUP:
                deadman_trigger_now = False
                break
        if deadman_trigger != deadman_trigger_now:
            deadman_trigger = deadman_trigger_now
            if deadman_trigger:
                print("Deadman trigger pressed")
            else:
                print("Deadman trigger released")
        await asyncio.sleep(0.01)


async def main():
    global deadman_trigger
    loop = asyncio.get_event_loop()
    loop.create_task(deadman_monitor())
    print("Waiting for deadman trigger")
    while not deadman_trigger:
        await asyncio.sleep(0.1)
    cf = Crazyflie()
    loop = asyncio.get_event_loop()
    await cf.arm()
    cf.learned_controller = True
    await cf.goto(np.array([0.0, 0.0, 0.4]))
    await cf.goto(np.array([0.0, 0.0, 0.4]), timeout=2)
    cf.learned_controller = False
    await cf.goto(np.array([0.0, 0.0, 0.0]))
    await cf.disarm()
    await asyncio.sleep(1000)

if __name__ == '__main__':
    asyncio.run(main())