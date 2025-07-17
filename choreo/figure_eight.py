import numpy as np
import matplotlib.pyplot as plt
from trajectories.lissajous import lissajous
import asyncio
import time
from registry import make_clients, configs
import copy


backend = "simulator"



async def main():
    scale = 1
    lissajous_parameters = dict(A=1.0*scale, B=0.5*scale, duration=20, ramp_duration=2)
    target_height = 1
    # plt.figure(figsize=(6, 6))
    # t_vals = np.linspace(0, lissajous_parameters["duration"], 1000)
    # coords = np.array([lissajous(t, **lissajous_parameters)[0] for t in t_vals])
    # vels = np.array([lissajous(t, **lissajous_parameters)[1] for t in t_vals])
    # plt.plot(t_vals, coords[:, 0], label="x")
    # plt.plot(t_vals, vels[:, 0], label="vx")
    # plt.plot(t_vals, coords[:, 1], label="y")
    # plt.plot(t_vals, vels[:, 1], label="vy")
    # plt.legend()
    # plt.show()

    machine = "simulator_race"
    config = copy.deepcopy(configs[machine])
    config["kwargs"]["initial_position"] = [0, 0, 0]
    clients = make_clients({machine: config})
    client = clients[machine]


    while client.position is None:
        await asyncio.sleep(0.01)
    
    initial_position = client.position

    await client.arm()
    await client.goto(initial_position + np.array([0, 0, target_height - initial_position[2]]), distance_threshold=0.15)
    await asyncio.sleep(1)
    start_time = time.time()
    while True:
        target_position, target_velocity = lissajous(time.time() - start_time, **lissajous_parameters)
        client.command(target_position, target_velocity)
        await asyncio.sleep(0.01)


    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())