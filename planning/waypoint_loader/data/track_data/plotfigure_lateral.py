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


def get_data(filename):
    with open(filename) as f:
        # frame = []
        x = []
        y = []
        lateral_error = []
        heading_error = []
        front_wheel_angle = []        

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
            # filter
            # if (float(data[i][2])<120):
                # continue

            # add data 
            # frame.append(float(data[i][0]))
        #     longitude.append(float(data[i][2]))
        #     latitude.append(float(data[i][1]))
        #     lateral_error.append(float(data[i][3]))     
        #     heading_error.append(float(data[i][5]))
        #     front_wheel_angle.append(float(data[i][-1]))

        # frame = np.array(frame)
        # longitude = np.array(longitude)
        # latitude = np.array(latitude)
        # lateral_error = np.array(lateral_error)
        # heading_error = np.array(heading_error)
        # front_wheel_angle = np.array(front_wheel_angle)
        phi_revise = math.atan((y[-1]-y[10])/(x[-1]-x[10]))
        for i in range(len(x)):
            xx.append(x[i]*math.cos(phi_revise) + y[i]*math.sin(phi_revise))
            yy.append(y[i]*math.cos(phi_revise) - x[i]*math.sin(phi_revise))
        xx = np.array(xx)
        yy = np.array(yy)
        
        # re = np.column_stack((frame,longitude,latitude,lateral_error,heading_error,front_wheel_angle))
        re = np.column_stack((xx,yy))
    return re

# GPS convert
def D2R(theta):
    rad = [i*math.pi/180 for i in theta]
    return rad

def Haversin(theta):
    return math.sin(theta/2.0)**2

def SphereDist2(p1,p2,R=6378137.0):
    p1 = D2R(p1)
    p2 = D2R(p2)
    h = Haversin(abs(p1[1]-p2[1]))+math.cos(p1[1])*math.cos(p2[1])*Haversin(abs(p1[0]-p2[0]))
    return 2 * R *math.asin(math.sqrt(h))

def SphereAzimuth2(p1,p2):
    p1 = D2R(p1)
    p2 = D2R(p2)
    lon1 = p1[0]
    lon2 = p2[0]
    lat1 = p1[1]
    lat2 = p2[1]
    tc1 = math.atan2(math.sin(lon2-lon1)*math.cos(lat2),
                     math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1))%(2*math.pi), 
    return tc1[0]

def con2xy(origin,azi_init,lon_set,lat_set):
    set_size = len(lat_set)
    x = []
    y = []
    xx =[]
    yy =[]
    for i in range(set_size):
        dis = SphereDist2(origin,[lon_set[i], lat_set[i]])
        azi = SphereAzimuth2(origin,[lon_set[i], lat_set[i]])
        x.append(dis*math.cos(azi_init - azi))
        y.append(dis*math.sin(azi_init - azi))
    phi_revise = math.atan((y[-1]-y[10])/(x[-1]-x[10]))
    for i in range(set_size):
        xx.append(x[i]*math.cos(phi_revise) + y[i]*math.sin(phi_revise))
        yy.append(y[i]*math.cos(phi_revise) - x[i]*math.sin(phi_revise))
    xx = np.array(xx)
    yy = np.array(yy)
    return xx,yy

filename1 = "track00.csv"  
data1 = get_data(filename1)
x = data1[:,0]
y = data1[:,1]
# Parameters 
# origin = [data1[10,1],data1[10,2]]
# azi_init = SphereAzimuth2(origin,[data1[10,1], data1[10,2]])


# Plot figures
# azi_init = data1[1,3]
# x,y = con2xy(origin,azi_init,data1[:,1], data1[:,2])


# 
# x = data1[:,1]
# y = data1[:,2]
plt.figure(figsize=(7,5))
plt.plot(x,y,c = 'red', label = 'distance')
# plt.legend(numpoints=1,loc=1)
# #plt.title('Average speed vs TTC')
# plt.xlabel("TTC[s]",font1)
# plt.ylabel("Pedestrian speed[m/s]",font1)
plt.show()
