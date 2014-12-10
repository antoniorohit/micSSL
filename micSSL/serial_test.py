#!/usr/bin/env /usr/bin/arch -i386 python

import numpy as np
import scipy.signal as sp
import scipy.optimize as opt
import pylab as plt
import serial
import time
import sys
import math

SPEED_SOUND = 340.0 # m/s
SAMPLE_RATE = 30000.0 # sps
DISTANCE_MIC = 0.20 # mt
r = 0.5773502*DISTANCE_MIC     # r = l/sqrt(3)
R = 0.8                         #    mt (Radius of Table)
K1 = r*r + R*R
K2 = 2*r*R
DELAY_DIFF_THRESH = 0.1
ERR_THRESH =  0.05                 # test
sign = 1
D = 0

def movingaverage(interval, window_size):
    window = np.ones(int(window_size))/float(window_size)
    return np.convolve(interval, window, 'valid')

def f(phi):
    y = D - np.sqrt(K1 - K2*np.cos(phi + 2*np.pi/3)) + np.sqrt(K1 - K2*np.cos(phi))
    return y

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

#         print (np.max(array1)), +(np.max(array2)), +(np.max(array3))
        
        array1 = (array1/np.linalg.norm(array1)) 
        array2 = (array2/np.linalg.norm(array2)) 
        array3 = (array3/np.linalg.norm(array3))   
        
        array1 = movingaverage(array1, 5)  
        array2 = movingaverage(array2, 5)  
        array3 = movingaverage(array3, 5)  
                    
        corr12 = sp.fftconvolve(array1-np.mean(array1), (array2-np.mean(array2))[::-1], mode='same')
        corr23 = sp.fftconvolve(array2-np.mean(array2), (array3-np.mean(array3))[::-1], mode='same')
        corr31 = sp.fftconvolve(array3-np.mean(array3), (array1-np.mean(array1))[::-1], mode='same')
        
        delayDiff12 =  (np.argmax(corr12) - 125.0)*SPEED_SOUND/SAMPLE_RATE      # metres
        delayDiff23 =  (np.argmax(corr23) - 125.0)*SPEED_SOUND/SAMPLE_RATE
        delayDiff31 =  (np.argmax(corr31) - 125.0)*SPEED_SOUND/SAMPLE_RATE

         
        max_index = np.argmax([delayDiff12, delayDiff23, delayDiff31])
    
#         plt.subplot(2, 1, 1)
#         plt.plot(corr12)
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
            print_index+=1;
            try:
                if max_index == 0 and delayDiff12 > DELAY_DIFF_THRESH:
                    ser.write("G")
#                     D = delayDiff12
#                     theta = opt.newton(f, 0, tol=1.48e-03)
#                     ser.write(chr(int((theta*180/math.pi + 0)/20)))      
                    if print_index%10 == 0:
                        print "Closest to G ||", "%.3f,%.3f,%.3f"%(delayDiff12, delayDiff23, delayDiff31)

                
                elif max_index == 1 and delayDiff23 > DELAY_DIFF_THRESH:
                    ser.write("B")
#                     D = delayDiff23
#                     theta = opt.newton(f, 0, tol=1.48e-03)
#                     ser.write(chr(int((theta*180/math.pi + 120)/20)))      
                    if print_index%10 == 0:
                        print "Closest to B ||", "%.3f,%.3f,%.3f"%(delayDiff12, delayDiff23, delayDiff31)

                elif max_index == 2 and delayDiff31 > DELAY_DIFF_THRESH:
                    ser.write("R")
#                     D = delayDiff31
#                     theta = opt.newton(f, 0, tol=1.48e-03)
#                     ser.write(chr(int((theta*180/math.pi + 240)/20)))      
                    if print_index%10 == 0:
                        print "Closest to R ||", "%.3f,%.3f,%.3f"%(delayDiff12, delayDiff23, delayDiff31)
                ser.write('Y')
            
            except ValueError or RuntimeError or OverflowError:
                if print_index%10 == 0:
                    print "%.3f,%.3f,%.3f"%(delayDiff12, delayDiff23, delayDiff31)
#                     print theta
                ser.write('Y')

        ser.flushInput()  
        ser.flushOutput()   
#         print "Crunching (ms):", "%.3f"%((time.time() - startTime)*1000)
        endTime = time.time()



            