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
# @author Julien Dupeyroux, Nikhil Wessendorp

import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
from scipy.fft import fft, fftfreq, fftshift
import subprocess
import sys
import csv
import math
import cmath

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

    if len(sys.argv) != 2:
        sys.exit('Usage: python test_radar_csv.py ID_OF_TRIAL')

    ID = sys.argv[1]

    # First antenna
    rx1_re = list()
    rx1_im = list()

    # Second antenna
    rx2_re = list()
    rx2_im = list()

    # Time (in ns)
    t = list()

    print("Loading and plotting Radar data for sample " + str(ID) + "...")

    with open("dataset/" + str(ID) + "/radar.csv") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            t.append(float(row[0])*1e-9)
            L = int((len(row)-1)/4)
            rx1_re.append(np.float_(row[1:L+1]))
            rx1_im.append(np.float_(row[L+1:2*L+1]))
            rx2_re.append(np.float_(row[2*L+1:3*L+1]))
            rx2_im.append(np.float_(row[3*L+1:4*L+1]))
    
    id = 2

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

    # Plot both real and imaginary parts of RX signals for the 1st chirp
    plt.subplot(4,2,1)
    plt.plot(re1[0:N//2])
    plt.plot(re2[0:N//2])
    plt.title("Real (1st chirp)")
    plt.legend(['RX_1','RX_2'], loc=1)
    plt.subplot(4,2,2)
    plt.plot(im1[0:N//2])
    plt.plot(im2[0:N//2])
    plt.title("Imaginary (1st chirp)")
    plt.legend(['RX_1','RX_2'], loc=1)

    # Plot the magnitude of FFTs for the 1st chirp (normalized frequency)
    plt.subplot(3,1,2)
    plt.plot(xf,mag1)
    plt.plot(xf,mag2)
    plt.title("Magnitude of the FFT (1st chirp)")
    plt.ylabel("Power spectrum")
    plt.legend(['RX_1','RX_2'], loc=1)
    plt.xlim([0, 0.5])

    # Plot the phase of FFTs for the 1st chirp (normalized frequency)
    plt.subplot(3,1,3)
    plt.plot(xf,angle1)
    plt.plot(xf,angle2)
    plt.title("Phase of the FFT (1st chirp)")
    plt.ylabel("unwrap(phase(fft(RX)))")
    plt.xlabel("Normalized frequency")
    plt.legend(['RX_1','RX_2'], loc=1)
    plt.xlim([0, 0.5])

    plt.show()

    plt.cla()
    plt.clf()

    f = plt.figure(frameon=False, figsize=(6, 5), dpi=300)
    canvas_width, canvas_height = f.canvas.get_width_height()

    # Open an ffmpeg process
    outf = "radar_" + str(ID) + ".mp4"
    cmdstring = ('ffmpeg', 
        '-y', '-r', '15', # overwrite, 15fps (radar frequency = 15.7 Hz)
        '-s', '%dx%d' % (canvas_width, canvas_height), # size of image string
        '-pix_fmt', 'argb', # format
        '-f', 'rawvideo',  '-i', '-', # tell ffmpeg to expect raw video from the pipe
        '-vcodec', 'mpeg4', outf) # output encoding
    p = subprocess.Popen(cmdstring, stdin=subprocess.PIPE)

    for id in range(len(rx1_re)):

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

        plt.suptitle("Magnitude of the FFT (1st chirp) - Sample #" + str(ID), size=16)

        # Create normalized frequency axis
        D = 2
        N = len(mag1)
        xf = fftshift(fftfreq(N, 1))[:N]

        # Plot the magnitude of FFTs for the 1st chirp (normalized frequency)
        plt.subplot(1,2,1)
        plt.plot(xf,mag1)
        plt.plot(xf,mag2)
        plt.legend(['RX_1','RX_2'], loc=1)
        plt.xlabel("Normalized frequency")
        plt.ylim([0, 2500])
        plt.xlim([0, 0.5])

        # Plot the magnitude of FFTs for the 1st chirp (normalized frequency) (semilog)
        plt.subplot(1,2,2)
        plt.semilogy(xf,mag1)
        plt.semilogy(xf,mag2)
        plt.legend(['RX_1','RX_2'], loc=1)
        plt.xlabel("Normalized frequency")
        plt.ylim([1, 1e4])
        plt.xlim([0, 0.5])

        f.canvas.draw()
        string = f.canvas.tostring_argb()
        p.stdin.write(string)

        plt.cla()
        plt.clf()

    p.communicate()
