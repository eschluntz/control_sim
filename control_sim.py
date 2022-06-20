#!/usr/bin/env python

"""
Simulates a simple pendulum to demonstrate different control laws.

\
 \
  \
   O

Theta is the angle of the pendulum.
Positive is counterclockwise.
Torque positive is counterclockwise.

theta_accel = - m * g * sin(theta) + torque
"""

import math
import time
import matplotlib.pyplot as plt
import numpy as np
import imageio
import control

dt = 0.05
MAX_F = 30.0
GOAL = math.pi / 2

class Pendulum(object):

    def __init__(self):
        self.t_end = 10.0

        # for rendering
        self._fig, self._axs = plt.subplots(1, 2, figsize=(15, 5), gridspec_kw={'width_ratios': [2, 1]})
        self._fig.tight_layout()
        self._frame_n = 0

    def render_frame(self, ts, thetas, torques, save):
        # draw graphs
        _axs = self._axs
        _axs[0].clear()
        _axs[0].set_xlim(0, self.t_end)
        _axs[0].set_ylim(-.5, 3.5)

        _axs[0].plot(ts, thetas, label="theta")
        _axs[0].plot([0, self.t_end], [GOAL, GOAL], "g--", label="goal")
        _axs[0].plot(ts, torques, label="torque")
        _axs[0].legend(loc="lower right")

        # draw pendulum
        _axs[1].clear()
        # remove axis labels
        _axs[1].set_xticks([])
        _axs[1].set_yticks([])
        _axs[1].set_xlim(-1.1, 1.1)
        _axs[1].set_ylim(-1.1, 1.1)
        _axs[1].set_aspect('equal')

        _axs[1].plot([0, math.sin(GOAL)], [0, -math.cos(GOAL)], "--", label="goal", color="green")
        _axs[1].plot([0, math.sin(thetas[-1])], [0, -math.cos(thetas[-1])], label="pendulum", marker="o")
        
        # save as frame to render into gif
        if save:
            plt.savefig(f'frames/frame_{self._frame_n:04d}.png')
            self._frame_n += 1
        
        plt.pause(0.001)
    
    def run(self, name : str, control_law : callable, init_theta=0, init_theta_vel=0.0, save=False):

        # store data
        t = 0.0
        theta = init_theta
        theta_vel = init_theta_vel
        self._frame_n = 0

        ts = []
        thetas = []
        theta_vels = []
        torques = []

        t0 = time.time()
        while t < self.t_end:
            # calculate new values
            torque = control_law(theta, theta_vel)
            theta_accel = - 1 * math.sin(theta) - 0.4 * theta_vel + torque
            theta_vel += theta_accel * dt
            theta += theta_vel * dt
            t += dt

            # store data
            ts.append(t)
            thetas.append(theta)
            theta_vels.append(theta_vel)
            torques.append(torque)

            # render
            if len(ts) % 5 == 0:
                self.render_frame(ts, thetas, torques, save)
            
        # render gif using imageio
        if save:
            imageio.mimsave(f'gifs/pendulum_{name}.gif', [imageio.imread(f'frames/frame_{i:04d}.png') for i in range(0, self._frame_n)])

        t1 = time.time()
        print(f"{t1 - t0:.2f}s")





def control_law_null(theta : float, theta_vel : float) -> float:
    return 0.0


def control_law_bang_bang(theta : float, theta_vel : float) -> float:
    if theta < GOAL:
        return .05 * MAX_F
    else:
        return -.05 * MAX_F


class ControlLawPID(object):
    def __init__(self, p, i, d):
        self.k_p = p
        self.k_i = i
        self.k_d = d
        self.integral = 0.0
        self.last_error = 0.0

    def __call__(self, theta : float, theta_vel : float) -> float:
        error = theta - GOAL
        self.integral += error * dt
        if self.last_error is not None:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        self.last_error = error
        return self.k_p * error + self.k_i * self.integral + self.k_d * derivative


def control_law_lqr(theta : float, theta_vel : float) -> float:
    A = np.array([[0, 1], [-1, 0]])
    B = np.array([[0], [1]])
    q = 10.0
    Q = np.array([[q, 0], [0, q]])
    R = np.array([[1]])

    K, _, _ = control.lqr(A, B, Q, R)
    return K[0, 0] * (GOAL - theta) + K[0, 1] * (0.0 - theta_vel) + 1.0


if __name__ == "__main__":
    p = Pendulum()
    # p.run("null", control_law_null, init_theta=.75, save=True)
    # p.run("bang", control_law_bang_bang, init_theta=.75,save=True)
    # p.run("p", ControlLawPID(-5.5, 0, -.5), init_theta=.75, save=True)
    # p.run("pi", ControlLawPID(-3.5, -1.40, -.5), init_theta=.75, save=True)
    # p.run("pd", ControlLawPID(-5.5, 0, -1.5), init_theta=.75, save=True)
    # p.run("pid", ControlLawPID(-5.5, -3.0, -1.5), init_theta=.75, save=True)
    # p.run("pid2", ControlLawPID(-5.5, -5.60, -5.5), init_theta=.75, save=True)
    p.run("lqr", control_law_lqr, init_theta=.75, save=True)
