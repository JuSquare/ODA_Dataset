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
from PIL import Image
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import math
import cmath
from scipy.fft import fft, fftfreq, fftshift
import sys
import csv
import rosbag
import rospy
import cv2

def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

def fft_radar(re, im):
    s1 = fftshift(fft(re))
    s2 = fftshift(fft(im))
    real = s1.real - s2.imag
    imag = s1.imag + s2.real
    mag = (real**2 + imag**2)**0.5
    angle = np.zeros(len(real))
    for i in range(len(angle)):
        angle[i] = math.atan2(real[i], imag[i])
    return mag, angle

if __name__ == '__main__':

    topics = ['/optitrack/pose', '/dvs/imu', '/dvs/events', '/radar/data']

    max_frames = 5

    if len(sys.argv) != 2:
        sys.exit('Usage: python sample_visualization_ros.py ID_OF_TRIAL')

    ID = sys.argv[1]

    print("Launching visualization for sample " + str(ID) + "...")

    path = "dataset/" + str(ID) + "/"
    myBag = rosbag.Bag(path + str(ID) + ".bag")

    # Get RGB camera data (collect one frame every 30 frames, within a limit of max_frames frames)
    myVideo = cv2.VideoCapture(path + str(ID) + ".avi") # fps = 29.97
    vidFrames = list()
    count = 1
    while(len(vidFrames)!=max_frames):
        success, sampleFrame = myVideo.read()
        count += 1
        if count%30 == 0:
            vidFrames.append(sampleFrame)

    # Get obstacle location (OptiTrack data)
    OBST_X = list()
    OBST_Y = list()
    with open("dataset/trial_overview.csv", newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0] == ID:
                OBST_X.append(float(row[1]))
                OBST_Y.append(float(row[3]))

    # DVS parameters
    DIMX = 240
    DIMY = 180
    FPS = 24

    # DVS Events
    dvs_x = list()
    dvs_y = list()
    dvs_p = list()

    # DVS Frames
    dvs_frames = list()
    dvs_myFrame = 128*np.ones((DIMY,DIMX,3), 'uint8')

    # DVS Time (in ns)
    dvs_t = list()
    dvs_t0 = -1
    dvs_previousTime = 0
    dvs_currentTime = 0

    # OptiTrack Position (in m)
    optitrack_x = list()
    optitrack_y = list()
    optitrack_z = list()

    # OptiTrack Orientation (quaternions) 
    optitrack_a = list()
    optitrack_b = list()
    optitrack_c = list()
    optitrack_d = list()

    # OptiTrack Time (in ns)
    optitrack_t = list()
    optitrack_t0 = -1

    # IMU Linear accelerations (m/s^2)
    imu_x = list()
    imu_filt_x = list()
    imu_y = list()
    imu_filt_y = list()
    imu_z = list()
    imu_filt_z = list()

    # IMU Angular velocities (rad/s)
    imu_p = list()
    imu_filt_p = list()
    imu_q = list()
    imu_filt_q = list()
    imu_r = list()
    imu_filt_r = list()

    # IMU Time (in ns)
    imu_t = list()
    imu_t0 = -1

    radToDeg = 57.2957786

    # Radar First antenna
    rx1_re = list()
    rx1_im = list()

    # Radar Second antenna
    rx2_re = list()
    rx2_im = list()

    # Radar Time (in ns)
    radar_t = list()
    radar_t0 = -1

    # Collect data from ROS bags
    for topic, msg, tt in myBag.read_messages():
        # OptiTrack (Ground Truth data)
        if topic == topics[0]:
            if optitrack_t0 < 0:
                optitrack_t0 = msg.header.stamp.to_nsec()
            optitrack_t.append((msg.header.stamp.to_nsec() - optitrack_t0)*1e-9)
            optitrack_x.append(msg.pose.position.x)
            optitrack_y.append(msg.pose.position.z)
            optitrack_z.append(msg.pose.position.y)
            optitrack_a.append(msg.pose.orientation.x)
            optitrack_b.append(msg.pose.orientation.y)
            optitrack_c.append(msg.pose.orientation.z)
            optitrack_d.append(msg.pose.orientation.w)
        # IMU data
        if topic == topics[1]:
            if imu_t0 < 0:
                imu_t0 = msg.header.stamp.to_nsec()
            imu_t.append((msg.header.stamp.to_nsec() - imu_t0)*1e-9)
            imu_x.append(msg.linear_acceleration.x)
            imu_y.append(msg.linear_acceleration.y)
            imu_z.append(msg.linear_acceleration.z)
            imu_p.append(radToDeg*msg.angular_velocity.x)
            imu_q.append(radToDeg*msg.angular_velocity.y)
            imu_r.append(radToDeg*msg.angular_velocity.z)
        # DVS data
        if topic == topics[2]:
            if dvs_t0 < 0:
                dvs_t0 = msg.events[0].ts.to_nsec()
            for i in range(len(msg.events)):
                dvs_currentTime = int(msg.events[i].ts.to_nsec()-dvs_t0)*1e-9
                dvs_t.append(dvs_currentTime)
                xx = DIMX - 1 - int(msg.events[i].x)
                dvs_x.append(xx)
                yy = DIMY - 1 - int(msg.events[i].y)
                dvs_y.append(yy)
                if msg.events[i].polarity == True:
                    pp=1
                else:
                    pp=-1
                dvs_p.append(pp)
                if dvs_currentTime - dvs_previousTime < 1/FPS:
                    if pp == 1:
                        dvs_myFrame[yy,xx,2] = 255
                    else:
                        dvs_myFrame[yy,xx,0] = 255
                else:
                    dvs_frames.append(dvs_myFrame)
                    dvs_previousTime = dvs_currentTime
                    dvs_myFrame = 128*np.zeros((DIMY,DIMX,3), 'uint8')
                    if pp == 1:
                        dvs_myFrame[yy,xx,2] = 255
                    else:
                        dvs_myFrame[yy,xx,0] = 255
        # Radar data
        if topic == topics[3]:
            if radar_t0 < 0:
                radar_t0 = msg.ts.to_nsec()
            radar_t.append((msg.ts.to_nsec() - radar_t0)*1e-9)
            rx1_re.append(msg.data_rx1_re)
            rx1_im.append(msg.data_rx1_im)
            rx2_re.append(msg.data_rx2_re)
            rx2_im.append(msg.data_rx2_im)

    dvs_frames.append(dvs_myFrame)

    myBag.close()

    # Apply basic filter to IMU data (moving average filter)
    W = 15
    for i in range(len(imu_x) - W + 1):
        imu_filt_x.append(np.mean(imu_x[i:(i+W)]))
        imu_filt_y.append(np.mean(imu_y[i:(i+W)]))
        imu_filt_z.append(np.mean(imu_z[i:(i+W)]))
    for i in range(W-1):
        imu_filt_x.append(np.mean(imu_x[(len(imu_x)+i-2*W):(len(imu_x)+i-1-W)]))
        imu_filt_y.append(np.mean(imu_y[(len(imu_x)+i-2*W):(len(imu_x)+i-1-W)]))
        imu_filt_z.append(np.mean(imu_z[(len(imu_x)+i-2*W):(len(imu_x)+i-1-W)]))
    for i in range(len(imu_x) - W + 1):
        imu_filt_p.append(np.mean(imu_p[i:(i+W)]))
        imu_filt_q.append(np.mean(imu_q[i:(i+W)]))
        imu_filt_r.append(np.mean(imu_r[i:(i+W)]))
    for i in range(W-1):
        imu_filt_p.append(np.mean(imu_p[(len(imu_x)+i-2*W):(len(imu_x)+i-1-W)]))
        imu_filt_q.append(np.mean(imu_q[(len(imu_x)+i-2*W):(len(imu_x)+i-1-W)]))
        imu_filt_r.append(np.mean(imu_r[(len(imu_x)+i-2*W):(len(imu_x)+i-1-W)]))

    # Apply FFT transform to radar data (first chirp only)
    id = 60
    # Get the first chirp and apply 0-padding for better FFT performance
    chirp_length = int(len(rx1_re[0])/16)
    re1 = np.zeros(2*chirp_length)
    re2 = np.zeros(2*chirp_length)
    im1 = np.zeros(2*chirp_length)
    im2 = np.zeros(2*chirp_length)
    re1[0:chirp_length-1] = rx1_re[id][0:chirp_length-1]
    re2[0:chirp_length-1] = rx2_re[id][0:chirp_length-1]
    im1[0:chirp_length-1] = rx1_im[id][0:chirp_length-1]
    im2[0:chirp_length-1] = rx2_im[id][0:chirp_length-1]
    # Process FFT
    mag1, angle1 = fft_radar(re1, im1)
    mag2, angle2 = fft_radar(re2, im2)

    plt.suptitle("Visualization for Radar data - Sample #" + str(ID), size=16)

    # Create normalized frequency axis
    D = 2
    N = len(mag1)
    xf = fftshift(fftfreq(N, 1))[:N]

    plt.suptitle("Visualization for dataset sample #" + str(ID), size=16)
    plt.subplots_adjust(left=0.035, right=0.973, top=0.913, bottom=0.057, wspace=0.16, hspace=0.26)

    # Plot visual information (DVS + RGB Camera)
    for i in range(5):
        # Plot DVS buffered data (every 1 sec up to max_frames sec)
        plt.subplot(9,7,i+1)
        plt.imshow(Image.fromarray(dvs_frames[(i+1)*FPS]))
        plt.title("t = " + str(i+1) + " sec")
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        # Plot RGB video frames (every 1 sec up to max_frames sec)
        plt.subplot(9,7,8+i)
        plt.imshow(Image.fromarray(vidFrames[i]))
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)

    # Plot positions OptiTrack
    plt.subplot(9,3,7)
    plt.plot(optitrack_t, optitrack_x)
    plt.legend("x", loc=3)
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,10)
    plt.plot(optitrack_t, optitrack_y)
    plt.legend("y", loc=3)
    plt.ylabel("OptiTrack Position [m]")
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,13)
    plt.plot(optitrack_t, optitrack_z)
    plt.legend("z", loc=3)
    plt.gca().axes.get_xaxis().set_visible(False)

    # Plot linear accelerations IMU
    plt.subplot(9,3,16)
    plt.plot(imu_t, imu_x)
    plt.plot(imu_t, imu_filt_x)
    plt.legend(['accX', 'filt_accX'], loc=3)
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,19)
    plt.plot(imu_t, imu_y)
    plt.plot(imu_t, imu_filt_y)
    plt.legend(['accY', 'filt_accY'], loc=3)
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.ylabel("IMU Linear Accelerations [m s^-2]")
    plt.subplot(9,3,22)
    plt.plot(imu_t, imu_z)
    plt.plot(imu_t, imu_filt_z)
    plt.legend(['accZ', 'filt_accZ'], loc=3)
    plt.xlabel("t [s]")

    # Plot orientation (quaternions) OptiTrack
    plt.subplot(9,3,8)
    plt.plot(optitrack_t, optitrack_a)
    plt.legend("a", loc=3)
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,11)
    plt.plot(optitrack_t, optitrack_b)
    plt.legend("b", loc=3)
    plt.ylabel("OptiTrack Quaternions")
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,14)
    plt.plot(optitrack_t, optitrack_c)
    plt.legend("c", loc=3)
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,17)
    plt.plot(optitrack_t, optitrack_d)
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.legend("d", loc=3)

    # Plot angular velocities
    plt.subplot(9,3,20)
    plt.plot(imu_t, imu_p)
    plt.plot(imu_t, imu_filt_p)
    plt.legend(['p', 'filt_p'], loc=3)
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,23)
    plt.plot(imu_t, imu_q)
    plt.plot(imu_t, imu_filt_q)
    plt.legend(['q', 'filt_q'], loc=3)
    plt.ylabel("IMU Angular Velocities [deg/s]")
    plt.gca().axes.get_xaxis().set_visible(False)
    plt.subplot(9,3,26)
    plt.plot(imu_t, imu_r)
    plt.plot(imu_t, imu_filt_r)
    plt.legend(['r', 'filt_r'], loc=3)
    plt.xlabel("t [s]")

    # Plot 2D trajectory OptiTrack
    ax = plt.subplot(9,7,(6,13))
    plt.plot(optitrack_x,optitrack_y)
    for i in range(len(OBST_X)):
        c = plt.Circle((OBST_X[i],OBST_Y[i]), 0.2, color='r')
        ax.add_patch(c)
    plt.title("2D Trajectory (OptiTrack)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.xlim([-4.5, 4.5])
    plt.ylim([-4.5, 4.5])
    plt.gca().set_aspect('equal', adjustable='box')

    # Plot 3D trajectory OptiTrack
    ax = plt.subplot(9,7,(7,14),projection='3d')
    for i in range(len(OBST_Y)):
        Xc,Yc,Zc = data_for_cylinder_along_z(OBST_X[i],OBST_Y[i],0.2,2)
        ax.plot_surface(Xc, Yc, Zc, alpha=0.5)
    plt.plot(optitrack_x,optitrack_y,optitrack_z)
    plt.title("3D Trajectory (OptiTrack)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_xlim3d(-4, 4)
    ax.set_ylim3d(-4, 4)
    ax.set_zlim3d(0, 2)

    # Plot the radar magnitude of FFTs for the 1st chirp (normalized frequency)
    # plt.subplot(3,7,(13,14))
    plt.subplot(3,3,6)
    plt.plot(xf,mag1)
    plt.plot(xf,mag2)
    plt.title("Radar - Magnitude of the FFT (1st chirp), t = " + "{:.2f}".format(radar_t[id]) + " sec")
    plt.ylabel("Power spectrum")
    plt.legend(['RX_1','RX_2'], loc=1)
    plt.xlim([0, 0.5])

    # Plot the radar phase of FFTs for the 1st chirp (normalized frequency)
    # plt.subplot(3,7,(20,21))
    plt.subplot(3,3,9)
    plt.plot(xf,angle1)
    plt.plot(xf,angle2)
    plt.title("Radar - Phase of the FFT (1st chirp), t = " + "{:.2f}".format(radar_t[id]) + " sec")
    plt.ylabel("unwrap(phase(fft(RX)))")
    plt.xlabel("Normalized frequency")
    plt.legend(['RX_1','RX_2'], loc=1)
    plt.xlim([0, 0.5])

    plt.show()

