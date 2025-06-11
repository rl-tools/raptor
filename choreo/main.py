import argparse
import numpy as np
import time
import asyncio
import roslibpy
import deadman
from mux import mux
from mocap import Vicon

from crazyflie import Crazyflie
    



vehicle_configs = [
    {
        "name": "crazyflie",
        "type": Crazyflie,
        "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E7"},
        # "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E8"},
        "mocap": "/vicon/crazyflie/pose",
        # "mocap_topic": None,
    }
]

async def main():
    deadman.run_deadman_monitor()
    mocap = Vicon()
    ros = roslibpy.Ros(host='localhost', port=9090)
    ros.run(timeout=5)

    vehicles = []
    for config in vehicle_configs:
        vehicle = config["type"](**config["kwargs"])
        if "mocap" in config and config["mocap"] is not None:
            mocap.add(vehicle, config["mocap"])
        vehicles.append(vehicle)
    print("Waiting for vehicles to be located")
    while not all([v.position is not None for v in vehicles]):
        await asyncio.sleep(0.1)
    vehicle = vehicles[0]
    print("Waiting for deadman trigger")
    while not deadman.trigger:
        await asyncio.sleep(0.1)
    initial_position = np.array(vehicle.position)
    print(f"Initial position: {initial_position[0]:.2f} {initial_position[1]:.2f} {initial_position[2]:.2f}")
    await vehicle.arm()
    vehicle.learned_controller = True
    # vehicle.learned_controller = False
    await vehicle.goto(initial_position + np.array([0.0, 0.0, 0.4]))
    await vehicle.goto(initial_position + np.array([0.0, 0.0, 0.4]), timeout=2)
    vehicle.learned_controller = False
    await vehicle.goto(initial_position + np.array([0.0, 0.0, 0.0]))
    await vehicle.disarm()
    await asyncio.sleep(1000)

if __name__ == '__main__':
    asyncio.run(main())