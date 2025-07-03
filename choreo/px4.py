import struct
import os
import asyncio
import time
import numpy as np

import deadman
from mux import mux

from drone import Drone

from pymavlink import mavutil



class PX4(Drone):
    def __init__(self, name, uri="tcp:localhost:5760", odometry_source="mocap", **kwargs):
        super().__init__(**kwargs)
        self.odometry_source = odometry_source

        self.safety_distance = 0.15

        self.connection = mavutil.mavlink_connection(uri)

        print(f"Waiting for PX4 heartbeat {name}")
        self.connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.connection.target_system, self.connection.target_component))
        # ask for LOCAL_POSITION_NED at 50 Hz  (20 000 µs period)
        # self.connection.mav.command_long_send(
        #     self.connection.target_system,
        #     self.connection.target_component,
        #     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # 511
        #     0,                                             # confirmation
        #     mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,  # param1: msg id
        #     40000,     # param2: µs between messages  → 50 Hz
        #     0, 0, 0, 0, 0)

        self.disarmed = False
        self.last_mocap_callback = None
        self.mocap_callback_dts = []
        self.POSITION_ERROR_CLIP = 0.5
        self.POSITION_STD = 0.01 # m
        self.VELOCITY_STD = 0.01 # m/s
        self.ORIENTATION_STD = 5.0 # degrees
        self.MOCAP_INTERVAL = 0.005
        self.latest_command = None

        loop = asyncio.get_event_loop()
        loop.create_task(self.main())
    def _mocap_callback(self, timestamp, position, orientation, velocity):
        usec = int(timestamp * 1e6)

        x,  y,  z  =  position[0], -position[1], -position[2]
        qw, qx, qy, qz = orientation
        q = [qw, qx, qy, qz]
        self.orientation = q
        q_mav = [qw,  qx, -qy, -qz]

        vx, vy, vz = velocity[0], -velocity[1], -velocity[2]

        if self.odometry_source == "mocap":
            self._odometry_callback(position, velocity)

        pose_cov = np.full(21, np.nan,  dtype=np.float32)
        vel_cov  = np.full(21, np.nan,  dtype=np.float32)
        pose_cov[0] = pose_cov[6] = pose_cov[11] = self.POSITION_STD**2
        pose_cov[15] = pose_cov[18] = pose_cov[20] = (np.deg2rad(self.ORIENTATION_STD))**2
        vel_cov[0]  = vel_cov[6]  = vel_cov[11]  = self.VELOCITY_STD**2
        now = time.time()
        if self.last_mocap_callback is None or self.MOCAP_INTERVAL is None or now - self.last_mocap_callback > self.MOCAP_INTERVAL:
            self.connection.mav.odometry_send(
                usec,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # pose frame
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,     # twist frame
                x, y, z,
                q_mav,
                # vx_body, vy_body, vz_body,
                vx, vy, vz,
                float('nan'), float('nan'), float('nan'),   # angular rates
                pose_cov,
                vel_cov,
                0,                                         # reset_counter
                mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
                100 # quality (100% confidence, 0-100)
            )
            if self.last_mocap_callback is not None:
                dt = now - self.last_mocap_callback
                self.mocap_callback_dts.append(dt)
                self.mocap_callback_dts = self.mocap_callback_dts[-100:]
            self.last_mocap_callback = now
            # print(f"Forwarding position (FRD) to MAVLink: {x:.2f}, {y:.2f}, {z:.2f}, q: {qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f} {1/np.mean(self.mocap_callback_dts):.2f} Hz")
    
    async def arm(self):
        pass
    
    async def main(self):
        while True:
            msg = self.connection.recv_match(
                type=['LOCAL_POSITION_NED',
                    'VEHICLE_LOCAL_POSITION',
                    'ODOMETRY'],
                blocking=False)

            if msg:
                typ = msg.get_type()

                if typ == 'LOCAL_POSITION_NED':
                    position = np.array([ msg.x, -msg.y, -msg.z ])
                    velocity = np.array([ msg.vx, -msg.vy, -msg.vz ])
                    print(f"Mavlink Position: {position[0]:.2f}  {position[1]:.2f}  {position[2]:.2f}  Velocity: {velocity[0]:.2f}  {velocity[1]:.2f}  {velocity[2]:.2f}")
                    if self.odometry_source != "mocap":
                        self._odometry_callback(position, velocity)
            if self.latest_command is not None:
                position, velocity = self.latest_command
                # if self.position is not None:
                #     diff = position - self.position
                #     diff = np.clip(diff, -self.POSITION_ERROR_CLIP, self.POSITION_ERROR_CLIP)
                #     position = self.position + diff
                # if self.velocity is not None:
                #     diff = velocity - self.velocity
                #     diff = np.clip(diff, -self.POSITION_ERROR_CLIP, self.POSITION_ERROR_CLIP)
                #     velocity = self.velocity + diff
                # print(f"cmd {'  '.join([f'{float(p):.2}' for p in position])} {'  '.join(f'{float(p):.2}' for p in velocity)}", file=mux[3])
                position_ned = [position[0], -position[1], -position[2]]
                # position_ned = [0, 0.8, -0.3]
                velocity_ned = [velocity[0], -velocity[1], -velocity[2]]
                # velocity_ned = np.zeros(3)
                self.connection.mav.set_position_target_local_ned_send(
                    0,
                    1, 
                    1,
                    # connection.target_system,
                    # connection.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b1111111111000000,
                    *position_ned,  # x, y, z
                    *velocity_ned,  # vx, vy, vz
                    0, 0, 0,  # afx, afy, afz
                    0, 0,  # yaw, yaw_rate
                )

            await asyncio.sleep(0.1)
    def _forward_command(self, position, velocity):
        self.latest_command = position, velocity

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
