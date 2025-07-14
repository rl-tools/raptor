import asyncio
from pyvicon_datastream.tools import ObjectTracker
from scipy.spatial.transform import Rotation as R
import time
import numpy as np
import csv

class Vicon:
    def __init__(self, VICON_TRACKER_IP, **kwargs):
        self.kwargs = kwargs
        self.VICON_TRACKER_IP = VICON_TRACKER_IP
        self.tracker = ObjectTracker(VICON_TRACKER_IP)
        assert self.tracker.is_connected, "Tracker must be connected to Vicon system"
        self.tracker.vicon_client.get_frame()
        subject_count = self.tracker.vicon_client.get_subject_count()
        names = [self.tracker.vicon_client.get_subject_name(i) for i in range(subject_count)]
        print("Currently observed object names:", names)
        self.tracker.vicon_client.disconnect()
        self.objects = []
    
    def add(self, object_name, callback):
        self.objects.append(ViconObject(object_name, ObjectTracker(self.VICON_TRACKER_IP), callback, **self.kwargs))


class ViconObject:
    def __init__(self, object_name, tracker, callback, VELOCITY_CLIP=None, EXPECTED_FRAMERATE=100):
        self.object_name = object_name
        self.tracker = tracker
        self.callback = callback
        self.NUM_FRAMES = 100
        self.NUM_FRAMES_CSV = 100000
        self.frames = []
        self.frame_times = []
        self.tick = 0
        self.dt = 0.001
        self.start_time = time.time()
        self.last_second = None
        self.data_records = []
        self.last_position = None
        self.last_position_time = None
        self.framerate = None
        self.VELOCITY_CLIP = VELOCITY_CLIP
        self.EXPECTED_FRAMERATE = EXPECTED_FRAMERATE
        self.poll_interval = 1/(EXPECTED_FRAMERATE * 3)
        self.reset_counter = 0
        asyncio.create_task(self.main())

    async def main(self):
        with open(f"vicon_data_{self.object_name}.csv", 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'frame', 'frame_dt', 'x', 'y', 'z', 'euler_x', 'euler_y', 'euler_z', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz'])
            while True:
                result = self.tracker.get_position(self.object_name)
                if result:
                    latency, frame, data = result
                    if len(self.frames) == 0 or frame != self.frames[-1]:
                        now_ns = time.time_ns()
                        now = now_ns / 1e9
                        data = list(filter(lambda x: x[0] == self.object_name, data))
                        if len(data) > 0:
                            self.frames.append(frame)
                            self.frame_times.append(now)
                            self.frames = self.frames[-self.NUM_FRAMES:]
                            self.frame_times = self.frame_times[-self.NUM_FRAMES:]
                            if  len(self.frame_times) > 1:
                                _, _, x, y, z, euler_x, euler_y, euler_z = data[0]
                                position = np.array([x, y, z]) / 1000
                                r = R.from_euler('XYZ', [euler_x, euler_y, euler_z])
                                orientation_xyzw = r.as_quat()  # Returns [x, y, z, w]
                                orientation = [orientation_xyzw[3], orientation_xyzw[0], orientation_xyzw[1], orientation_xyzw[2]]
                                # print(f"x: {position[0]:.2f}, y: {position[1]:.2f}, z: {position[2]:.2f}, roll: {euler_x:.2f}, pitch: {euler_y:.2f}, yaw: {euler_z:.2f}")
                                if self.last_position is not None:
                                    frame_dt = now - self.last_position_time
                                    if frame_dt > 1.5/self.EXPECTED_FRAMERATE:
                                        print(f"High frame latency: {frame_dt}")
                                        self.reset_counter += 1
                                    velocity = (position - self.last_position) / frame_dt
                                    if self.VELOCITY_CLIP is not None:
                                        velocity = np.clip(velocity, -self.VELOCITY_CLIP, self.VELOCITY_CLIP)
                                else:
                                    frame_dt = 0 
                                    velocity = np.zeros(3)
                                self.data_records.append((now, frame, frame_dt, position, euler_x, euler_y, euler_z, *orientation, velocity))
                                writer.writerow([now, frame, frame_dt, *position, euler_x, euler_y, euler_z, *orientation, *velocity])
                                self.data_records = self.data_records[-self.NUM_FRAMES_CSV:]
                                self.last_position = position
                                self.last_position_time = now
                                self.callback(now_ns, position, orientation, velocity, self.reset_counter)
                if len(self.frames) > 1:
                    if(np.any(np.diff(self.frames) > 1)):
                        if np.argmax(np.diff(self.frames)) == len(self.frames) - 1:
                            print(f"Mocap: {self.object_name} frame Jump detected: {np.argmax(np.diff(self.frames))} {self.frames}")

                current_second = int(time.time())
                tick_now = False
                if self.last_second is None or current_second != self.last_second:
                    self.last_second = int(time.time())
                    self.tick += 1
                    tick_now = True
                if tick_now and self.tick % 10 == 0 and len(self.frames) > 1:
                    self.framerate = len(self.frames) / (self.frame_times[-1] - self.frame_times[0])
                    print(f"Mocap: {self.object_name} Framerate: {self.framerate:.2f}")

                await asyncio.sleep(self.poll_interval)

