import struct
import asyncio
import time
import numpy as np
from elrs import ELRS
from elrs.elrs import RC_CHANNEL_MIN, RC_CHANNEL_MAX


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


class Betaflight(Drone):
    def __init__(self, uri='/dev/ttyUSB0', BAUD=921600, rate=50, odometry_source="mocap", verbose=False,**kwargs):
        super().__init__(**kwargs)
        self.odometry_source = odometry_source
        self.orientation = None
        self.position = None
        self.velocity = None
        self.target_position = None
        self.target_velocity = None
        self.elrs = ELRS(uri, baud=BAUD, rate=rate, telemetry_callback=self._telemetry_callback, verbose=verbose)
        asyncio.create_task(self.elrs.start())


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
        while True:
            if self.orientation is None or self.position is None or self.velocity is None or self.target_position is None or self.target_velocity is None:
                await asyncio.sleep(0.01)
                continue
            w_m, z_m = self.orientation[0], self.orientation[3]
            if w_m < 0:
                w_m = -w_m
                z_m = -z_m
            d_m = np.sqrt(w_m*w_m + z_m*z_m)
            if d_m < 1e-6:
                continue
            angle_transmission = 2 * np.arctan2(z_m / d_m, w_m / d_m) / np.pi

            relative_position = np.array(self.position) - np.array(self.target_position)
            relative_velocity = np.array(self.velocity) - np.array(self.target_velocity)
            if tick % 100 == 0:
                print(f"angle transmission: {angle_transmission:.2f}")
                print(f"relative position: {relative_position[0]:.2f} {relative_position[1]:.2f} {relative_position[2]:.2f} velocity: {relative_velocity[0]:.2f} {relative_velocity[1]:.2f} {relative_velocity[2]:.2f}")
            def to_channel(value):
                return np.clip(value, -1, 1) * (RC_CHANNEL_MAX - RC_CHANNEL_MIN) / 2 + (RC_CHANNEL_MAX + RC_CHANNEL_MIN) / 2
            output = [
                to_channel(relative_position), # AET
                to_channel([angle_transmission]), # R
                to_channel([-1 if not deadman.trigger else 1]), # AUX1
                to_channel(relative_velocity) # AUX2-4
            ]
            self.elrs.set_channels(np.concatenate(output).astype(int).tolist())
            await asyncio.sleep(0.01)
            tick += 1
    def _forward_command(self, position, velocity):
            self.target_position = position
            self.target_velocity = velocity

    async def goto(self, target_input, distance_threshold=0.15, timeout=None, relative=True):
        print(f"Going to {target_input}")
        distance = None
        start = time.time()
        while distance is None or distance > distance_threshold or (timeout is not None and time.time() - start < timeout) and not self.disarmed:
            if self.position is not None:
                target = target_input if relative else target_input
                distance = np.linalg.norm(target - self.position)
                # print(f"Distance to target: {distance:.2f} m", file=mux[4])
                self._forward_command(target, [0, 0, 0])
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
        pass

async def main():
    time.sleep(10)
    VICON_IP = "192.168.1.3"
    from mocap import Vicon
    target_position = [0, 0, 0.2]
    mocap = Vicon(VICON_TRACKER_IP=VICON_IP)
    betaflight = Betaflight(uri='/dev/serial/by-name/elrs-transmitter2', BAUD=921600, rate=50, odometry_source="mocap", verbose=True)
    betaflight._forward_command(target_position, [0, 0, 0])
    # asyncio.create_task(deadman.monitor(type="foot-pedal")),
    asyncio.create_task(deadman.monitor(type="gamepad")),
    betaflight_main_task = asyncio.create_task(betaflight.main())
    mocap.add("hummingbird", betaflight._mocap_callback)
    while betaflight.position is None:
        await asyncio.sleep(0.1)
    initial_position = betaflight.position.copy()
    target_position = initial_position + np.array([0, 0, 0.2])
    print(f"Initial position: {initial_position}")
    print(f"Target position: {target_position}")

    async def timer():
        tick = 0
        dt = 0.01
        while True:
            if tick*dt > 5:
                betaflight.elrs.verbose = False
            await asyncio.sleep(dt)
            tick += 1
    asyncio.create_task(timer())
    while True:
        betaflight._forward_command(target_position, [0, 0, 0])
        while not deadman.trigger:
            await asyncio.sleep(0.1)
        betaflight._forward_command(target_position, [0, 0, 0])
        await asyncio.sleep(5)
        betaflight._forward_command(initial_position + np.array([0, 0, -0.2]), [0, 0, 0])
        await asyncio.sleep(5)
        while deadman.trigger:
            await asyncio.sleep(0.1)
    # await betaflight.goto(initial_position, distance_threshold=0.0)
    await betaflight_main_task # DO NOT TERMINATE, IF TERMINATED, THE DRONE DOES NOT RECEIVE FEEDBACK AND LIKELY SHOOTS INTO THE SKY

if __name__ == "__main__":
    asyncio.run(main())