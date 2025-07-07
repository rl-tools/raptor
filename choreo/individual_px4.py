from px4 import PX4
from mocap import Vicon
import asyncio
import signal
import numpy as np
import sys

# VICON_IP = "128.238.39.121"
VICON_IP = "192.168.1.3"

async def main():
    mocap = Vicon(VICON_IP, VELOCITY_CLIP=5, EXPECTED_FRAMERATE=100)
    def sig_handler(signum, frame):
        print("Saving mocap data")
        mocap.save_to_csv()
        sys.exit(0)
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    cfg = {
        "name": "race",
        "type": PX4,
        "kwargs": {"uri": "tcp:192.168.8.4:5760"},
        # "kwargs": {"uri": "udp:192.168.8.4:14550"},
        # "kwargs": {"uri": "/dev/ttyACM0"},
        "mocap": "race_jonas",
    }
    px4 = PX4(name=cfg["name"], **cfg["kwargs"])
    mocap.add(cfg["mocap"], px4._mocap_callback)
    print("Waiting for px4 position")
    while px4.position is None:
        await asyncio.sleep(0.1)
    target_position = px4.position + np.array([0, 0, 0.2])
    print(f"Target position: {target_position}")
    while True:
        await px4.goto(target_position, distance_threshold=0.05)
        await asyncio.sleep(0.01)

if __name__ == "__main__":
    asyncio.run(main())