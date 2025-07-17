import numpy as np
import asyncio


VICON_IP = "192.168.1.3"
configs = {
    "simulator_race": {
        "type": "simulator",
        "kwargs": {
            "parameter_path": "choreo/parameters/race.json",
            "initial_position": [0, 0, 0]
        },
    },
    "crazyflie_bl": {
        "type": "crazyflie",
        "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E9"},
        "mocap": "crazyflie_bl"
    },
    "crazyflie":{
        "type": "crazyflie",
        "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E7"},
        "mocap": "crazyflie",
    },
    "race": {
        "type": "px4",
        "kwargs": {"uri": "tcp:192.168.1.2:5760"},
        "mocap": "race_jonas",
    },
    "hummingbird": {
        "type": "betaflight",
        "kwargs": {"uri": "/dev/serial/by-name/elrs-transmitter2", "BAUD": 921600, "rate": 50, "odometry_source": "mocap"},
        "mocap": "hummingbird",
    },
    "savagebee_pusher": {
        "type": "betaflight",
        "kwargs": {"uri": "/dev/serial/by-name/elrs-transmitter1", "BAUD": 921600, "rate": 50, "odometry_source": "mocap"},
        "mocap": "savagebee_pusher",
    },
    "m5stampfly": {
        "type": "m5stampfly",
        "kwargs": {"uri": "/dev/serial/by-name/m5stamp-forwarder", "BAUD": 115200, "rate": 50, "odometry_source": "mocap"},
        "mocap": "m5stampfly",
    },
}


def make_clients(configs):
    simulator_keys = [k for k in configs.keys() if configs[k]["type"] == "simulator"]
    crazyflie_keys = [k for k in configs.keys() if configs[k]["type"] == "crazyflie"]

    clients = {}
    if len(simulator_keys) > 0:
        from simulator import Simulator, SimulatedDrone
        simulator = Simulator(N_DRONES=(np.log2(len(simulator_keys))//1 + 1) ** 2)
        for key in simulator_keys:
            client = SimulatedDrone(simulator, name=key, **configs[key]["kwargs"])
            asyncio.create_task(client.run()),
            clients[key] = client
        asyncio.create_task(simulator.run())
    
    return {k: clients[k] if k in clients else None for k in configs.keys()}
    
    # elif backend == "px4":
    #     from px4 import PX4
    #     from mocap import Vicon
    #     VICON_IP = "192.168.1.3"
    #     mocap = Vicon(VICON_IP, VELOCITY_CLIP=5, EXPECTED_FRAMERATE=100)
    #     cfg = {
    #         "name": "race",
    #         "type": PX4,
    #         "kwargs": {"uri": "tcp:192.168.1.2:5760"},
    #         "mocap": "race_jonas",
    #     }
    #     px4 = PX4(name=cfg["name"], **cfg["kwargs"])
    #     mocap.add(cfg["mocap"], px4._mocap_callback)
    #     asyncio.create_task(client.run()),
    #     print("Waiting for px4 position")
    #     while px4.position is None:
    #         await asyncio.sleep(0.1)
    #     initial_position = px4.position
    # elif backend == "crazyflie":
    #     from crazyflie import Crazyflie
    #     crazyflie_configs = [
    #         {
    #             "name": "crazyflie_bl",
    #             "type": Crazyflie,
    #             "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E9"},
    #             "mocap": "crazyflie_bl",
    #         },
    #         {
    #             "name": "crazyflie",
    #             "type": Crazyflie,
    #             "kwargs": {"uri": "radio://0/80/2M/E7E7E7E7E7"},
    #             "mocap": "crazyflie",
    #         },
    #     ]