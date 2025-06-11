import numpy as np
import matplotlib.pyplot as plt
import json

def build_lookup(A=1, B=0.5, a=1, b=2, scale=1, N=10000):
    theta = np.linspace(0, 2*np.pi, N)
    x = scale*A*np.sin(a*theta)
    y = scale*B*np.sin(b*theta)
    s = np.concatenate(([0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
    return theta, s/s[-1], s[-1]

theta_tab, s_norm_tab, total_len = build_lookup()

def lissajous_uniform(t, A=1, B=0.5, a=1, b=2, z=1, scale=1, duration=10,
                      theta_tab=theta_tab, s_norm_tab=s_norm_tab, total_len=total_len):
    s_norm = (t % duration) / duration
    theta = np.interp(s_norm, s_norm_tab, theta_tab)
    x = scale*A*np.sin(a*theta)
    y = scale*B*np.sin(b*theta)
    dx_dth = scale*A*a*np.cos(a*theta)
    dy_dth = scale*B*b*np.cos(b*theta)
    denom = np.hypot(dx_dth, dy_dth)
    v = total_len / duration
    vx = v * dx_dth / denom
    vy = v * dy_dth / denom
    return np.array([x, y, z]), np.array([vx, vy, 0.0])

def plot_lissajous(just_add=False, **kwargs):
    t_vals = np.linspace(0, kwargs["duration"], 1000)
    coords = np.array([lissajous_uniform(t, **kwargs)[0] for t in t_vals])

    if not just_add:
        plt.figure(figsize=(6, 6))
    plt.plot(coords[:, 0], coords[:, 1])
    plt.title(f"Lissajous: {json.dumps(kwargs)}")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis('equal')
    plt.grid(True)
    if not just_add:
        plt.show()