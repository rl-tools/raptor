import numpy as np
def lissajous(t, A=1, B=0.5, a=1, b=2, z=1, scale=1, duration=10):
    progress = t * 2 * np.pi / duration
    d_progress = 2 * np.pi / duration
    x = scale * A * np.sin(a * progress)
    y = scale * B * np.sin(b * progress)
    vx = scale * A * np.cos(a * progress) * a * d_progress
    vy = scale * B * np.cos(b * progress) * b * d_progress
    return np.array([x, y, z]), np.array([vx, vy, 0])