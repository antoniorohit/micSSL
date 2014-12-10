#!/usr/bin/env /usr/bin/arch -i386 python

import numpy as np
import scipy.signal as sp
import pylab as plt
import serial
import time
import sys
import math
import scipy.optimize as opt

SPEED_SOUND = 340.0             # m/s
SAMPLE_RATE = 30000.0           # sps
DISTANCE_MIC = 0.20             # mt
r = 0.5773502*DISTANCE_MIC     # r = l/sqrt(3)
R = 0.8                         #    mt (Radius of Table)
K1 = r*r + R*R
K2 = 2*r*R
THRESHOLD = 0.05
ERR_THRESH =  0.05                 # test
D = 0

def movingaverage(interval, window_size):
    window = np.ones(int(window_size))/float(window_size)
    return np.convolve(interval, window, 'valid')

def f(phi):
    y = D - np.sqrt(K1 - K2*np.cos(phi + 2*np.pi/3)) + np.sqrt(K1 - K2*np.cos(phi))
    return y
 
def approxTheta(D):
    theta = 0
    err_old = D - np.sqrt(K1 - K2*np.cos(theta + 2*np.pi/3)) + np.sqrt(K1 - K2*np.cos(theta))
    theta = 0.03
    err = D - np.sqrt(K1 - K2*np.cos(theta + 2*np.pi/3)) + np.sqrt(K1 - K2*np.cos(theta))
    if(err<err_old):
        correction = 0.03
    else:
        correction = -0.03
    while(abs(err) > ERR_THRESH):
        err = D - np.sqrt(K1 - K2*np.cos(theta + 2*np.pi/3)) + np.sqrt(K1 - K2*np.cos(theta))
        theta += correction

#     print "Error:" +"%.3f"%err, "Theta:"+"%.3f"%(theta*180/math.pi)
        
    return theta


color1 = 0
color2 = 0
color3 = 0


ser = serial.Serial('/dev/tty.usbmodem1452', 921600, timeout=0.1)

ser.close()
ser.open()

array1 = np.ndarray(shape=(250), dtype=int)
array2 = np.ndarray(shape=(250), dtype=int)
array3 = np.ndarray(shape=(250), dtype=int)
i = 0
elem = []
startTime = endTime = 0
print_index = 0

while(1):
    if ser.inWaiting():
        elem = ser.readline().strip().split(',')
        if(elem[0] != '' and elem != ['Go']):
            try:
                array1[i] = int(elem[0])
                array2[i] = int(elem[1])
                array3[i] = int(elem[2])
                if( ((array1[i]) <= 1000) and  (array2[i] <= 1000) and  (array3[i] <= 1000 )):
                    i += 1
                else:
                    print "Missed!"
                    print elem
            except:
                print elem, sys.exc_info()[0]
                pass
            
    if(i >= 250):
        startTime = time.time()
#         print "Serial (ms):", "%.3f"%((time.time() - endTime)*1000)

        max = (np.max(array1)) +(np.max(array2)) +(np.max(array3))
        
        array1 = (array1/np.linalg.norm(array1)) 
        array2 = (array2/np.linalg.norm(array2)) 
        array3 = (array3/np.linalg.norm(array3))   
        
        array1 = movingaverage(array1, 5)  
        array2 = movingaverage(array2, 5)  
        array3 = movingaverage(array3, 5)  
                    
        corr21 = sp.fftconvolve(array2-np.mean(array2), (array1-np.mean(array1))[::-1], mode='same')
        corr23 = sp.fftconvolve(array2-np.mean(array2), (array3-np.mean(array3))[::-1], mode='same')
        corr31 = sp.fftconvolve(array3-np.mean(array3), (array1-np.mean(array1))[::-1], mode='same')
        
        delayDiff21 =  (np.argmax(corr21) - 125.0)*SPEED_SOUND/SAMPLE_RATE      # metres
        delayDiff23 =  (np.argmax(corr23) - 125.0)*SPEED_SOUND/SAMPLE_RATE
        delayDiff31 =  (np.argmax(corr31) - 125.0)*SPEED_SOUND/SAMPLE_RATE
        
        D1 = (delayDiff21)
        D2 = (delayDiff31)

        D = D1

        theta = opt.newton(f, 0)
        
        bit0 = (D1 > 0)
        bit1 = (D2 > 0)
        bit2 = (abs(D1) > abs(D2))
        
        delayDiff = np.argmax([delayDiff21, delayDiff23, delayDiff31])
#       
#         plt.subplot(2, 1, 1)
#         plt.plot(corr21)
#         plt.subplot(2, 1, 2)
#         plt.plot(array1)
#         plt.plot(array2)
#         plt.plot(array3)
#         plt.grid()
#         plt.show()
        array1 = np.ndarray(shape=(250), dtype=int)
        array2 = np.ndarray(shape=(250), dtype=int)
        array3 = np.ndarray(shape=(250), dtype=int)

        i = 0
        while(ser.readline().strip()!="Go"):
            if False:
                if(bit0 == True and bit1 == True and bit2 == False):
                    print "0-60"
                    ser.write("1")
                elif(bit0 == False and bit1 == True and bit2 == False):
                    print "60-120"
                    ser.write("2")
                elif(bit0 == False and bit1 == False and bit2 == False):
                    print "120-180"
                    ser.write("3")
                elif(bit0 == False and bit1 == False and bit2 == True):
                    print "180-240"
                    ser.write("4")
                elif(bit0 == True and bit1 == False and bit2 == True):
                    print "240-300"
                    ser.write("5")
                elif(bit0 == True and bit1 == True and bit2 == True):
                    print "300-360"
                    ser.write("6")
            
            print "%3.0f"%(theta*180/math.pi)                  
            ser.write(chr(int((theta*180/math.pi + 180)/20)))      
            
            if delayDiff == 0 and delayDiff21 > 0.05:
                ser.write("G")
            elif delayDiff == 1 and delayDiff23 > 0.05:
                ser.write("B")
            elif delayDiff == 2 and delayDiff31 > 0.05:
                ser.write("R")
                
            ser.write('Y')

        ser.flushInput()  
        ser.flushOutput()   
#         print "Crunching (ms):", "%.3f"%((time.time() - startTime)*1000)
        endTime = time.time()



            