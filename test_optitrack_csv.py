# MIT License
#
# Copyright (c) 2021 Julien Dupeyroux
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# @author Julien Dupeyroux

import numpy as np 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import sys
import csv

def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

if __name__ == '__main__':

    if len(sys.argv) != 2:
        sys.exit('Usage: python test_optitrack_csv.py ID_OF_TRIAL')

    ID = sys.argv[1]

    # Get obstacle location
    OBST_X = list()
    OBST_Y = list()
    with open("dataset/trial_overview.csv", newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0] == ID:
                OBST_X.append(float(row[1]))
                OBST_Y.append(float(row[3]))

    # Position (in m)
    x = list()
    y = list()
    z = list()

    # Orientation (quaternions) (in rad)
    a = list()
    b = list()
    c = list()
    d = list()

    # Time (in ns)
    t = list()
    t0 = -1

    print("Loading and plotting OptiTrack data for sample " + str(ID) + "...")

    with open("dataset/" + str(ID) + "/optitrack.csv") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if t0 < 0:
                t0 = int(row[0])
            t.append((int(row[0]) - t0)*1e-9)
            x.append(float(row[1]))
            y.append(float(row[3]))
            z.append(float(row[2]))
            a.append(float(row[4]))
            b.append(float(row[5]))
            c.append(float(row[6]))
            d.append(float(row[7]))

    plt.suptitle("Visualization for OptiTrack data - Sample #" + str(ID), size=16)

    # Plot positions
    plt.subplot(4,3,1)
    plt.plot(t, x)
    plt.legend("x")
    plt.title("Cartesian coordinates (meters)")
    plt.subplot(4,3,4)
    plt.plot(t, y)
    plt.legend("y")
    plt.subplot(4,3,7)
    plt.plot(t, z)
    plt.xlabel("t [s]")
    plt.legend("z")

    # Plot orientation (quaternions)
    plt.subplot(4,3,2)
    plt.plot(t, a)
    plt.legend("a", loc=3)
    plt.title("Quaternions")
    plt.subplot(4,3,5)
    plt.plot(t, b)
    plt.legend("b", loc=3)
    plt.subplot(4,3,8)
    plt.plot(t, c)
    plt.legend("c", loc=3)
    plt.subplot(4,3,11)
    plt.plot(t, d)
    plt.xlabel("t [s]")
    plt.legend("d", loc=3)

    # Plot 2D trajectory
    ax = plt.subplot(5,3,(3,6))
    plt.plot(x,y)
    for i in range(len(OBST_X)):
        c = plt.Circle((OBST_X[i],OBST_Y[i]), 0.2, color='r')
        ax.add_patch(c)
    plt.title("2D Trajectory")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.xlim([-4.5, 4.5])
    plt.ylim([-4.5, 4.5])
    plt.gca().set_aspect('equal', adjustable='box')

    # Plot 3D trajectory
    ax = plt.subplot(4,3,(9,12),projection='3d')
    for i in range(len(OBST_Y)):
        Xc,Yc,Zc = data_for_cylinder_along_z(OBST_X[i],OBST_Y[i],0.2,2)
        ax.plot_surface(Xc, Yc, Zc, alpha=0.5)
    plt.plot(x,y,z)
    plt.title("3D Trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_xlim3d(-4, 4)
    ax.set_ylim3d(-4, 4)
    ax.set_zlim3d(0, 2)

    plt.show()