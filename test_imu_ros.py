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
import sys
import rosbag
import rospy
import math

if __name__ == '__main__':

    if len(sys.argv) != 2:
        sys.exit('Usage: python test_imu_ros.py ID_OF_TRIAL')

    ID = sys.argv[1]

    radToDeg = 57.2957786

    path = "dataset/" + str(ID) + "/"
    myBag = rosbag.Bag(path + str(ID) + ".bag")

    myTopic = '/dvs/imu'
    
    # linear accelerations (m/s^2)
    x = list()
    filt_x = list()
    y = list()
    filt_y = list()
    z = list()
    filt_z = list()

    # Angular velocities (rad/s)
    p = list()
    filt_p = list()
    q = list()
    filt_q = list()
    r = list()
    filt_r = list()

    # Time (in ns)
    t = list()
    t0 = -1

    print("Loading and plotting IMU data for sample " + str(ID) + "...")

    for topic, msg, tt in myBag.read_messages():
        if topic == myTopic:
            if t0 < 0:
                t0 = msg.header.stamp.to_nsec()
            t.append((msg.header.stamp.to_nsec() - t0)*1e-9)
            x.append(msg.linear_acceleration.x)
            y.append(msg.linear_acceleration.y)
            z.append(msg.linear_acceleration.z)
            p.append(radToDeg*msg.angular_velocity.x)
            q.append(radToDeg*msg.angular_velocity.y)
            r.append(radToDeg*msg.angular_velocity.z)

    myBag.close()
    
    W = 15
    
    for i in range(len(x) - W + 1):
        filt_x.append(np.mean(x[i:(i+W)]))
        filt_y.append(np.mean(y[i:(i+W)]))
        filt_z.append(np.mean(z[i:(i+W)]))
    
    for i in range(W-1):
        filt_x.append(np.mean(x[(len(x)+i-2*W):(len(x)+i-1-W)]))
        filt_y.append(np.mean(y[(len(x)+i-2*W):(len(x)+i-1-W)]))
        filt_z.append(np.mean(z[(len(x)+i-2*W):(len(x)+i-1-W)]))

    for i in range(len(x) - W + 1):
        filt_p.append(np.mean(p[i:(i+W)]))
        filt_q.append(np.mean(q[i:(i+W)]))
        filt_r.append(np.mean(r[i:(i+W)]))
    
    for i in range(W-1):
        filt_p.append(np.mean(p[(len(x)+i-2*W):(len(x)+i-1-W)]))
        filt_q.append(np.mean(q[(len(x)+i-2*W):(len(x)+i-1-W)]))
        filt_r.append(np.mean(r[(len(x)+i-2*W):(len(x)+i-1-W)]))

    plt.suptitle("Visualization for 6-axes IMU data - Sample #" + str(ID), size=16)

    # Plot linear accelerations
    plt.subplot(3,2,1)
    plt.plot(t, x)
    plt.plot(t, filt_x)
    plt.legend(['raw_x', 'filt_x'], loc=3)
    plt.title("Linear accelerations (m/s^2)")
    plt.subplot(3,2,3)
    plt.plot(t, y)
    plt.plot(t, filt_y)
    plt.legend(['raw_y', 'filt_y'], loc=3)
    plt.subplot(3,2,5)
    plt.plot(t, z)
    plt.plot(t, filt_z)
    plt.xlabel("t [s]")
    plt.legend(['raw_z', 'filt_z'], loc=3)

    # Plot angular velocities
    plt.subplot(3,2,2)
    plt.plot(t, p)
    plt.plot(t, filt_p)
    plt.legend(['raw_p', 'filt_p'], loc=3)
    plt.title("Angular velocities (deg/s)")
    plt.subplot(3,2,4)
    plt.plot(t, q)
    plt.plot(t, filt_q)
    plt.legend(['raw_q', 'filt_q'], loc=3)
    plt.subplot(3,2,6)
    plt.plot(t, r)
    plt.plot(t, filt_r)
    plt.legend(['raw_r', 'filt_r'], loc=3)
    plt.xlabel("t [s]")

    plt.show()