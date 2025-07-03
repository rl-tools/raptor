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
    def __init__(self, uri='/dev/ttyUSB0', BAUD=921600, rate=50, odometry_source="mocap", **kwargs):
        super().__init__(**kwargs)
        self.odometry_source = odometry_source
        self.orientation = None
        self.position = None
        self.velocity = None
        self.target_position = None
        self.target_velocity = None
        self.elrs = ELRS(uri, baud=BAUD, rate=rate, telemetry_callback=self._telemetry_callback)
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
    def _mocap_callback(self, msg):
        pose = msg['pose']["pose"]
        position = [pose['position']['x'], pose['position']['y'], pose['position']['z']]
        twist = msg['twist']["twist"]
        self.velocity = [twist['linear']['x'], twist['linear']['y'], twist['linear']['z']]
        self.orientation = [pose['orientation']['w'], pose['orientation']['x'], pose['orientation']['y'], pose['orientation']['z']]
        if self.odometry_source == "mocap":
            self._odometry_callback(position, self.velocity if self.velocity is not None else np.zeros(3))
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
        pass
    
    async def main(self):
        while True:
            if self.orientation is None or self.position is None or self.velocity is None or self.target_position is None or self.target_velocity is None:
                continue
            w_m, z_m = self.orientation[0], self.orientation[3]
            if w_m < 0:
                w_m = -w_m
                z_m = -z_m
            d_m = np.sqrt(w_m*w_m + z_m*z_m)
            if d_m < 1e-6:
                continue
            angle_transmission = 2 * np.arctan2(z_m / d_m, w_m / d_m) / np.pi
            print(f"angle transmission: {angle_transmission:.2f}")
            relative_position = np.array(self.position) - np.array(self.target_position)
            relative_velocity = np.array(self.velocity) - np.array(self.target_velocity)
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
                print(f"Distance to target: {distance:.2f} m", file=mux[4])
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
    from mocap import Vicon
    target_position = [0, 0, 0.2]
    betaflight = Betaflight(uri='/dev/ttyUSB0', BAUD=921600, rate=50, odometry_source="mocap")
    betaflight._forward_command(target_position, [0, 0, 0])
    asyncio.create_task(deadman.monitor()),
    asyncio.create_task(betaflight.main())
    mocap = Vicon()
    mocap.add(betaflight, "/vicon/hummingbird/odom")
    while True:
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())