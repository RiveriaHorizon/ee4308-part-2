#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np


def path_plot():
    file_name = "../build/io/path.txt"

    x_pose, y_pose, z_pose, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, heading, ang_vel = np.loadtxt(
        file_name, delimiter=' ', unpack=True)

    x_axis = [i for i in range(1, len(x_pose) + 1)]

    plt.title('Path Text File Plot')
    plt.plot(x_axis, x_pose)
    plt.plot(x_axis, y_pose)
    plt.plot(x_axis, z_pose)
    plt.plot(x_axis, x_vel)
    plt.plot(x_axis, y_vel)
    plt.plot(x_axis, x_acc)
    plt.plot(x_axis, y_acc)
    plt.xlabel('Time [s]')
    plt.ylabel('Distance [m], Velocity [m/s] and Acceleration [m/s2]')
    plt.legend(['pos_x', 'pos_y', 'pos_z', 'vel_x',
                'vel_y', 'acc_x', 'acc_y'])
    plt.savefig('../build/graph_plots/path_txt_plot.png', dpi=700)


if __name__ == "__main__":
    path_plot()
