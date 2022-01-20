# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 10:00:34 2022

@author: Huiqian
"""
import os
import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import sys

font1 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size'   : 15,
}

'''
frame,  time, x, y, heading, 
v_x,  v_y,  yaw_rate, steer_angle, pedal_acc,
pedal_brake, lon_acc
'''

def get_data_reference(filename):
    with open(filename) as f:
        x = []
        y = []
        xx = []
        yy = []
        steer_angle = []

        readers = csv.reader(f,delimiter=',')
        origin_data = list(readers)
        head = origin_data.pop(0)
        data = np.array(origin_data)
        data_size = len(data)
        print filename,"data size:",data_size
        for i  in range(data_size):
            x.append(float(data[i][2]))
            y.append(float(data[i][3]))
            # steer_angle.append(float(data[i][8]))
        phi_revise = math.atan((y[-1]-y[10])/(x[-1]-x[10]))
        for i in range(len(x)):
            xx.append(x[i]*math.cos(phi_revise) + y[i]*math.sin(phi_revise))
            yy.append(y[i]*math.cos(phi_revise) - x[i]*math.sin(phi_revise))
        xx = np.array(xx)
        yy = np.array(yy)
        
        re = np.column_stack((xx,yy))
    return re,phi_revise

def get_data(filename,revise_angle):
    with open(filename) as f:
        x = []
        y = []
        xx = []
        yy = []
        steer_angle = []

        readers = csv.reader(f,delimiter=',')
        origin_data = list(readers)
        head = origin_data.pop(0)
        data = np.array(origin_data)
        data_size = len(data)
        print filename,"data size:",data_size
        for i  in range(data_size):
            x.append(float(data[i][2]))
            y.append(float(data[i][3]))
            steer_angle.append(float(data[i][8]))
        phi_revise = revise_angle
        for i in range(len(x)):
            xx.append(x[i]*math.cos(phi_revise) + y[i]*math.sin(phi_revise))
            yy.append(y[i]*math.cos(phi_revise) - x[i]*math.sin(phi_revise))
        xx = np.array(xx)
        yy = np.array(yy)
        
        re = np.column_stack((xx,yy,steer_angle))
    return re


args = sys.argv
args_num = args.__len__()

print "args number:",args_num

if(args_num == 1):
    print "No args input."
else:
    for i in range(1,args_num):
        print args[i]

    plt.figure(figsize=(7,5))
    filename0 = args[1]
    data0,r = get_data_reference(filename0)
    x0 = data0[:,0]
    y0 = data0[:,1]
    plt.plot(x0,y0,c = 'red', label = filename0)

    if(args_num >= 3):
        filename1 = args[2]
        data1 = get_data(filename1,r)
        x1 = data1[:,0]
        y1 = data1[:,1]
        plt.plot(x1,y1,c = 'blue', label = filename1)

    if(args_num >= 4):
        filename2 = args[3]
        data2 = get_data(filename2,r)
        x2 = data2[:,0]
        y2 = data2[:,1]
        plt.plot(x2,y2,c = "green", label = filename2)

    if(args_num >= 5):
        filename3 = args[4]
        data3 = get_data(filename3,r)
        x3 = data3[:,0]
        y3 = data3[:,1]
        plt.plot(x3,y3,c = "black", label = filename3)

    if(args_num >= 6):
        filename4 = args[5]
        data4 = get_data(filename4,r)
        x4 = data4[:,0]
        y4 = data4[:,1]
        plt.plot(x4,y4,c = "orange", label = filename4)

    plt.legend(numpoints=1,loc=1)
    plt.show()

# filename1 = "data_logger-0119-1.csv"  
# data1,r = get_data_reference(filename1)
# filename2 = "data_logger-0119-4.csv"

# filename3 = "data_logger-0119-2.csv"
# data3 = get_data(filename3,r)
# filename4 = "data_logger-0119-3.csv"
# data4 = get_data(filename4,r)



# x3 = data3[:,0]
# y3 = data3[:,1]
# x4 = data4[:,0]
# y4 = data4[:,1]

# steer_angle1 = data1[:,2]
# steer_angle2 = data2[:,2]
# steer_angle3 = data3[:,2]
# steer_angle4 = data4[:,2]



# plt.figure(figsize=(7,5))
# plt.plot(x1,steer_angle1,c = 'red')
# plt.plot(x2,steer_angle2,c = 'blue')
# plt.plot(x3,steer_angle3,c = 'green')
# plt.plot(x4,steer_angle4,c = 'black')
# plt.show()
