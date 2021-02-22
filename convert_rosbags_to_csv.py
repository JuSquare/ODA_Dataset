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
import os
import csv
import rosbag
import rospy

topics = ['/optitrack/pose', '/dvs/imu', '/dvs/events', '/radar/data']

if __name__ == '__main__':
    for i in range(1345,1370):
        if i == 1306 or i == 1321 or i == 1344:
            continue
        path = "dataset/" + str(i) + "/"
        myBag = rosbag.Bag(path + str(i) + ".bag")
        optitrack_t0 = -1
        imu_t0 = -1
        dvs_t0 = -1
        radar_t0 = -1
        
        optitrackFile = open(path + "optitrack.csv", mode="w")
        optitrack_writer = csv.writer(optitrackFile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        optitrack_writer.writerow(['time [ns]', 'x [m]', 'y [m]', 'z [m]', 'a', 'b', 'c', 'd'])

        imuFile = open(path + "imu.csv", mode="w")
        imu_writer = csv.writer(imuFile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        imu_writer.writerow(['time [ns]', 'ax [m s-2]', 'ay [m s-2]', 'az [m s-2]', 'p [rad/s]', 'q [rad/s]', 'r [rad/s]'])

        dvsFile = open(path + "dvs.csv", mode="w")
        dvs_writer = csv.writer(dvsFile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        dvs_writer.writerow(['time [ns]', 'x', 'y', 'p'])

        radarFile = open(path + "radar.csv", mode="w")
        radar_writer = csv.writer(radarFile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        radar_writer.writerow(['time [ns]'])

        for topic, msg, t in myBag.read_messages():
            if topic == topics[0]:
                if optitrack_t0 < 0:
                    optitrack_t0 = msg.header.stamp.to_nsec()
                optitrack_writer.writerow([msg.header.stamp.to_nsec()-optitrack_t0,msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
            if topic == topics[1]:
                if imu_t0 < 0:
                    imu_t0 = msg.header.stamp.to_nsec()
                imu_writer.writerow([msg.header.stamp.to_nsec()-imu_t0,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
            if topic == topics[2]:
                if dvs_t0 < 0:
                    dvs_t0 = msg.events[0].ts.to_nsec()
                for i in range(len(msg.events)):
                    if msg.events[i].polarity == True:
                        dvs_writer.writerow([msg.events[i].ts.to_nsec()-dvs_t0,msg.events[i].x,msg.events[i].y,1])
                    else:
                        dvs_writer.writerow([msg.events[i].ts.to_nsec()-dvs_t0,msg.events[i].x,msg.events[i].y,-1])
            if topic == topics[3]:
                if radar_t0 < 0:
                    radar_t0 = msg.ts.to_nsec()
                L = len(msg.data_rx1_re)
                data = np.zeros(4*L+1)
                data[0] = msg.ts.to_nsec()-radar_t0
                for i in range(L):
                    data[i+1] = msg.data_rx1_re[i]
                    data[i+1+L] = msg.data_rx1_im[i]
                    data[i+1+2*L] = msg.data_rx2_re[i]
                    data[i+1+3*L] = msg.data_rx2_im[i]
                radar_writer.writerow(data)



