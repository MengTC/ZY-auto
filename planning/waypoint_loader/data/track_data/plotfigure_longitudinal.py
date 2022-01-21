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

def get_data(filename):
    with open(filename) as f:
        re = []
        re = np.array(re)

        readers = csv.reader(f,delimiter=',')
        origin_data = list(readers)
        head = origin_data.pop(0)
        data = np.array(origin_data)
        data_size = len(data)
        print filename,"data size:",data_size
        var_num = len(data[0])
        print "variable number:",var_num
        for j in range(var_num):# for each variable in csv file
            y = []
            for i  in range(data_size):
                y.append(float(data[i][j]))
            x = np.array(y)
            if j == 0:
                re = x
            else:
                re = np.column_stack((re,x))
    return re, head


colors = ['red','blue','violet','pink','green','brown','orange','yellow']
args = sys.argv
args_num = args.__len__()
# args[1]: column id to plot 
#     [2]: filenames to read
print "args number:",args_num
col = int(args[1])
filenames = args[2].split(",")
plt.figure(figsize=(7,5))


for i in range(len(filenames)):
    filename = filenames[i]

    color = colors[i]
    data,header = get_data(filename)
    t = data[:,1] - data[0,1]
    if (col == 5):
        plt.plot(t,data[:,col]*3.6, c = color, label=filename)
    else:
        plt.plot(t,data[:,col], c = color, label=filename)
var_names = header
plt.title(var_names[col])
plt.legend(numpoints=1,loc=1)
plt.show()




    


# if(args_num == 1):
#     print "No args input."
# else:
#     for i in range(1,args_num):
#         print args[i]

#     plt.figure(figsize=(7,5))
#     filename0 = args[1]
#     data0,r = get_data_reference(filename0)
#     x0 = data0[:,0]
#     y0 = data0[:,1]
#     plt.plot(x0,y0,c = 'red', label = filename0)

#     if(args_num >= 3):
#         filename1 = args[2]
#         data1 = get_data(filename1,r)
#         x1 = data1[:,0]
#         y1 = data1[:,1]
#         plt.plot(x1,y1,c = 'blue', label = filename1)

#     if(args_num >= 4):
#         filename2 = args[3]
#         data2 = get_data(filename2,r)
#         x2 = data2[:,0]
#         y2 = data2[:,1]
#         plt.plot(x2,y2,c = "green", label = filename2)

#     if(args_num >= 5):
#         filename3 = args[4]
#         data3 = get_data(filename3,r)
#         x3 = data3[:,0]
#         y3 = data3[:,1]
#         plt.plot(x3,y3,c = "black", label = filename3)

#     if(args_num >= 6):
#         filename4 = args[5]
#         data4 = get_data(filename4,r)
#         x4 = data4[:,0]
#         y4 = data4[:,1]
#         plt.plot(x4,y4,c = "orange", label = filename4)

#     plt.legend(numpoints=1,loc=1)
#     plt.show()

