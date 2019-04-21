#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np


def mjtg(current, setpoint, frequency, move_time):
    trajectory = []
    trajectory_derivative = []
    trajectory_derivative_two = []
    timefreq = int(move_time * frequency)

    for time in range(1, timefreq):
        trajectory.append(
            current + (setpoint - current) *
            (10.0 * (time/timefreq)**3
             - 15.0 * (time/timefreq)**4
             + 6.0 * (time/timefreq)**5)
        )

        trajectory_derivative.append(
            frequency * (setpoint - current) *
            (30.0 * (time)**2.0 * (1.0/timefreq)**3
             - 60.0 * (time)**3.0 * (1.0/timefreq)**4
             + 30.0 * (time)**4.0 * (1.0/timefreq)**5)
        )

        trajectory_derivative_two.append(
            frequency * (setpoint - current) *
            (60.0 * (time) * (1.0/timefreq)**3
             - 180.0 * (time)**2 * (1.0/timefreq)**4
             + 120.0 * (time)**3 * (1.0/timefreq)**5)
        )

    return trajectory, trajectory_derivative, trajectory_derivative_two


# Set up and calculate trajectory.
average_velocity = 0.5
current = 0.0
setpoint = 1.0
frequency = 20
time = (setpoint - current) / average_velocity

traj, traj_vel, traj_acc = mjtg(current, setpoint, frequency, time)

# Create plot.
xaxis = [i / frequency for i in range(1, int(time * frequency))]

plt.plot(xaxis, traj)
plt.plot(xaxis, traj_vel)
plt.plot(xaxis, traj_acc)
plt.title("Minimum jerk trajectory")
plt.xlabel("Time [s]")
plt.ylabel("Distance [m], Velocity [m/s] and Acceleration [m/s2]")
plt.legend(['pos', 'vel', 'acc'])
plt.show()
