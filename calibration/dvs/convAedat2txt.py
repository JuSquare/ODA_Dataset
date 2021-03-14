import sys
from PIL import Image
import serial
import numpy as np
import math
from dv import AedatFile

DVS_T = list() # timestamps given in microseconds
DVS_X = list() # X address
DVS_Y = list() # Y address
DVS_P = list() # polarity (1: ON, 0: OFF)

if len(sys.argv)!=3:
        sys.exit("Use: python3 read.py <name_of_file>.aedat4 <name_of_file>.txt")

with AedatFile(sys.argv[1]) as f:
    # list all the names of streams in the file
        print("Reading the .aedat4 file... (This may take time)")
    # getting the events
        for e in f['events']:
                DVS_X.append(e.x)
                DVS_Y.append(e.y)
                DVS_T.append(e.timestamp)
                DVS_P.append(e.polarity)
        print("Reading finished. ")

with open(sys.argv[2],"w+") as f:
    print("Writing the data to " + sys.argv[2])
    for i in range(0,len(DVS_T)-1):
        if DVS_P[i] == True:
            f.write(str(DVS_T[i]) + "," + str(DVS_X[i]) + "," + str(DVS_Y[i]) + "," + "1" + "\n") 
        else:
            f.write(str(DVS_T[i]) + "," + str(DVS_X[i]) + "," + str(DVS_Y[i]) + "," + "0" + "\n") 

