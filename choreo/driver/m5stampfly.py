import struct
import asyncio
import time
import numpy as np
import serial

import deadman
from mux import mux

from drone import Drone

# import asyncio
# from datetime import datetime

# PORT = "/dev/ttyUSB0"
# BAUD = 921600

# async def main() -> None:

#     def callback(ftype, decoded):
#         ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
#         print(f"[{ts}] {ftype:02X} {decoded}")


#     asyncio.create_task(elrs.start())

#     value = 1000
#     while True:
#         channels = [value] * 16
#         elrs.set_channels(channels)
#         value = (value + 1) % 2048
#         await asyncio.sleep(0.1)

# if __name__ == "__main__":
#     asyncio.run(main())


class M5StampFly(Drone):
    def __init__(self, uri='/dev/serial/by-name/m5stamp-forwarder', BAUD=115200, rate=50, odometry_source="mocap", verbose=False,**kwargs):
        super().__init__(**kwargs)
        self.odometry_source = odometry_source
        self.orientation = None
        self.position = None
        self.velocity = None
        self.target_position = None
        self.target_velocity = None
        self.serial = serial.Serial(uri, BAUD, timeout=1)


        self.position_number = 0
        self.safety_distance = 0.15
        loop = asyncio.get_event_loop()
        loop.create_task(self.main())
        self.disarmed = False
        self.last_pose_callback = None
        self.pose_callback_dts = []
        self.POSITION_ERROR_CLIP = 0.5
    def _telemetry_callback(self, frame_type, data):
        pass
    def _mocap_callback(self, timestamp, position, orientation, velocity, reset_counter):
        self.velocity = velocity
        self.orientation = orientation
        self.position = position
        if self.target_position is None:
            self.target_position = position
            self.target_velocity = np.zeros(3)
        if self.odometry_source == "mocap":
            self._odometry_callback(position, velocity)
        now = time.time()
        if self.last_pose_callback is not None and (now - self.last_pose_callback < 0.01):
            return
        if self.last_pose_callback is not None:
            dt = now - self.last_pose_callback
            self.pose_callback_dts.append(dt)
            self.pose_callback_dts = self.pose_callback_dts[-100:]
        self.last_pose_callback = now
        # print(f"vicon pos: {position[0]:.2f} {position[1]:.2f} {position[2]:.2f} {1/np.mean(self.pose_callback_dts):.2f} Hz", file=mux[1])
    
    async def arm(self):
        pass
    
    async def main(self):
        tick = 0
        previous_deadman_trigger = None
        landing_start = None
        landing_position = None
        while True:
            if self.orientation is None or self.position is None or self.velocity is None or self.target_position is None or self.target_velocity is None:
                await asyncio.sleep(0.01)
                relative_position = [0, 0, 1]
                angle_transmission = 0
                relative_velocity = [0, 0, 0]
                armed = False
            else:
                w_m, z_m = self.orientation[0], self.orientation[3]
                if w_m < 0:
                    w_m = -w_m
                    z_m = -z_m
                d_m = np.sqrt(w_m*w_m + z_m*z_m)
                if d_m < 1e-6:
                    continue
                angle_transmission = 2 * np.arctan2(z_m / d_m, w_m / d_m) / np.pi

                if previous_deadman_trigger is not None and previous_deadman_trigger != deadman.trigger:
                    if not deadman.trigger and landing_start is None:
                        print("Landing")
                        landing_start = time.time()
                        landing_position = self.position.copy()
                        landing_position[2] = -0.2

                if landing_start is not None:
                    self.target_position = landing_position
                    self.target_velocity = np.zeros(3)
                    if time.time() - landing_start > 1:
                        landing_start = None
                        armed = False
                    else:
                        armed = True
                else:
                    armed = deadman.trigger
                
                previous_deadman_trigger = deadman.trigger

                relative_position = np.array(self.position) - np.array(self.target_position)
                relative_velocity = np.array(self.velocity) - np.array(self.target_velocity)
                if tick % 100 == 0:
                    # print(f"angle transmission: {angle_transmission:.2f}")
                    print(f"relative position: {relative_position[0]:.2f} {relative_position[1]:.2f} {relative_position[2]:.2f} velocity: {relative_velocity[0]:.2f} {relative_velocity[1]:.2f} {relative_velocity[2]:.2f}")
                    # print(f"orientation: {self.orientation[0]:.2f} {self.orientation[1]:.2f} {self.orientation[2]:.2f} {self.orientation[3]:.2f}")
            data_to_send = f"{relative_position[0]:0.3f},{relative_position[1]:0.3f},{relative_position[2]:0.3f},{angle_transmission:0.3f},{relative_velocity[0]:0.3f},{relative_velocity[1]:0.3f},{relative_velocity[2]:0.3f},{int(armed)}\n"
            self.serial.write(data_to_send.encode())
            # if(self.serial.in_waiting > 0):
            #     print(self.serial.readline().decode())
            await asyncio.sleep(0.01)
            tick += 1
    def _forward_command(self, position, velocity):
            self.target_position = position
            self.target_velocity = velocity

    async def disarm(self):
        pass

async def main():
    VICON_IP = "192.168.1.3"
    from mocap import Vicon
    mocap = Vicon(VICON_TRACKER_IP=VICON_IP)
    target_position = [0, 0, 0.2]
    fly = M5StampFly(uri='/dev/serial/by-name/m5stamp-forwarder', rate=50, odometry_source="mocap", verbose=True)
    fly.command(target_position, [0, 0, 0])
    asyncio.create_task(deadman.monitor(type="foot-pedal")),
    fly_main_task = asyncio.create_task(fly.main())
    mocap.add("m5stampfly", fly._mocap_callback)
    while fly.position is None:
        await asyncio.sleep(0.1)
    initial_position = fly.position.copy()
    target_position = initial_position + np.array([0, 0, 0.5])
    print(f"Initial position: {initial_position}")
    print(f"Target position: {target_position}")


    while True:
        fly.command(target_position, [0, 0, 0])
        while not deadman.trigger:
            await asyncio.sleep(0.1)
        fly.command(target_position, [0, 0, 0])
        await asyncio.sleep(30)
        fly.command(initial_position + np.array([0, 0, -0.2]), [0, 0, 0])
        await asyncio.sleep(5)
        while deadman.trigger:
            await asyncio.sleep(0.1)
    # await fly.goto(initial_position, distance_threshold=0.0)
    await fly_main_task # DO NOT TERMINATE, IF TERMINATED, THE DRONE DOES NOT RECEIVE FEEDBACK AND LIKELY SHOOTS INTO THE SKY

if __name__ == "__main__":
    asyncio.run(main())