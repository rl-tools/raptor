import numpy as np
def lissajous(t, A=1, B=0.5, a=1, b=2, z=1, scale=1, duration=10, ramp_duration=0):
    time_velocity = min(t, ramp_duration) / ramp_duration if ramp_duration > 0 else 1
    ramp_time = time_velocity * min(t, ramp_duration) / 2 # time deficit
    progress = (ramp_time + max(0, t - ramp_duration)) * 2 * np.pi / duration
    d_progress = 2 * np.pi * time_velocity / duration
    x = scale * A * np.sin(a * progress)
    y = scale * B * np.sin(b * progress)
    vx = scale * A * np.cos(a * progress) * a * d_progress
    vy = scale * B * np.cos(b * progress) * b * d_progress
    return np.array([x, y, z]), np.array([vx, vy, 0])