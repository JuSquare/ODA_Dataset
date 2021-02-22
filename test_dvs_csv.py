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
import sys
import csv
import subprocess

if __name__ == '__main__':

    if len(sys.argv) != 2:
        sys.exit('Usage: python test_dvs_csv.py ID_OF_TRIAL')

    ID = sys.argv[1]

    DIMX = 240
    DIMY = 180

    FPS = 24

    # Events
    x = list()
    y = list()
    p = list()

    # Frames
    frames = list()
    myFrame = 128*np.ones((DIMY,DIMX,3), 'uint8')

    # Time (in ns)
    t = list()
    t0 = -1
    previousTime = 0
    currentTime = 0

    with open("dataset/" + str(ID) + "/dvs.csv") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if t0 < 0:
                t0 = int(row[0])
            currentTime = (int(row[0]) - t0)*1e-9
            t.append(currentTime)
            xx = DIMX - 1 - int(row[1])
            x.append(xx)
            yy = DIMY - 1 - int(row[2])
            y.append(yy)
            pp = int(row[3])
            p.append(pp)

            if currentTime - previousTime < 1/FPS:
                if pp == 1:
                    myFrame[yy,xx,2] = 255
                else:
                    myFrame[yy,xx,0] = 255
            else:
                frames.append(myFrame)
                previousTime = currentTime
                myFrame = 128*np.zeros((DIMY,DIMX,3), 'uint8')
                if pp == 1:
                    myFrame[yy,xx,2] = 255
                else:
                    myFrame[yy,xx,0] = 255

    frames.append(myFrame)

    L = min(5,int(len(frames)/FPS))

    plt.figure(figsize=(20,3.5))

    plt.suptitle("Visualization for DVS data - Sample #" + str(ID), size=16)

    for i in range(L):
        plt.subplot(1,L,i+1)
        plt.imshow(Image.fromarray(frames[(i+1)*FPS]))
        plt.title("t = " + str(i+1) + " sec")
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)

    plt.show()

    plt.cla()
    plt.clf()


    f = plt.figure(frameon=False, figsize=(6, 6*DIMY/DIMX), dpi=300)
    canvas_width, canvas_height = f.canvas.get_width_height()

    # Open an ffmpeg process
    outf = "dvs_" + str(ID) + ".mp4"
    cmdstring = ('ffmpeg', 
        '-y', '-r', '%d' % FPS, # overwrite, 100fps
        '-s', '%dx%d' % (canvas_width, canvas_height), # size of image string
        '-pix_fmt', 'argb', # format
        '-f', 'rawvideo',  '-i', '-', # tell ffmpeg to expect raw video from the pipe
        '-vcodec', 'mpeg4', outf) # output encoding
    p = subprocess.Popen(cmdstring, stdin=subprocess.PIPE)

    for i in range(len(frames)):

        plt.imshow(Image.fromarray(frames[i]))

        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        
        f.canvas.draw()
        string = f.canvas.tostring_argb()
        p.stdin.write(string)

        plt.cla()
        plt.clf()

    p.communicate()


