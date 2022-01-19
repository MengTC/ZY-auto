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

font1 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size'   : 15,
}


def get_data_reference(filename):
    with open(filename) as f:
        x = []
        y = []
        xx = []
        yy = []

        readers = csv.reader(f,delimiter=',')
        origin_data = list(readers)
        head = origin_data.pop(0)
        data = np.array(origin_data)
        data_size = len(data)
        for i  in range(data_size):
            x.append(float(data[i][2]))
            y.append(float(data[i][3]))
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

        readers = csv.reader(f,delimiter=',')
        origin_data = list(readers)
        head = origin_data.pop(0)
        data = np.array(origin_data)
        data_size = len(data)
        for i  in range(data_size):
            x.append(float(data[i][2]))
            y.append(float(data[i][3]))
        phi_revise = revise_angle
        for i in range(len(x)):
            xx.append(x[i]*math.cos(phi_revise) + y[i]*math.sin(phi_revise))
            yy.append(y[i]*math.cos(phi_revise) - x[i]*math.sin(phi_revise))
        xx = np.array(xx)
        yy = np.array(yy)
        
        re = np.column_stack((xx,yy))
    return re

filename1 = "reference_path.csv"  
data1,r = get_data_reference(filename1)
filename2 = "data_logger.csv"
data2 = get_data(filename2,r)

x = data1[:,0]
y = data1[:,1]
x_ = data2[:,0]
y_ = data2[:,1]

plt.figure(figsize=(7,5))
plt.plot(x,y,c = 'red', label = 'desired')
plt.plot(x_,y_,c = 'blue', label = "pure pursuit")
plt.legend(numpoints=1,loc=1)
# #plt.title('Average speed vs TTC')
# plt.xlabel("TTC[s]",font1)
# plt.ylabel("Pedestrian speed[m/s]",font1)
plt.show()
