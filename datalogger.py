# -*- coding: utf-8 -*-
"""
Created on Tue Jun  9 22:05:57 2020

@author: Aaron
"""

#library declaration
import serial
import csv
import time
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("tkAgg")
import numpy as np
import tkinter

#serial communication
ser = serial.Serial('COM5')
ser.flushInput() #clear queue to avoid data overlap

#plotting settings
plot_window = 100 #plot window width
y_var = np.array(np.zeros([plot_window]))

plt.ion() #interactive mode on
#fig, ax = plt.subplots()
#line, = ax.plot(y_var)

#1st button
def log_data():
    plot_window = 100 #plot window width
    y_var = np.array(np.zeros([plot_window]))

    plt.ion() #interactive mode on
    fig, ax = plt.subplots()
    line, = ax.plot(y_var)
    
    while True:
        try:
            ser_bytes = ser.readline()
            
            try:
                decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
                print(decoded_bytes)
            except:
                continue
            
            #data write
            with open("test_data.csv","a") as f:
                writer = csv.writer(f,delimiter=",")
                writer.writerow([time.time(),decoded_bytes])
                
            y_var = np.append(y_var,decoded_bytes)
            y_var = y_var[1:plot_window+1]
            line.set_ydata(y_var)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()
            
        except KeyboardInterrupt: #ctrl+c to stop
            print('interrupted!')
            break

#Widgets
m = tkinter.Tk() #Main Window

#Data Logger Button Widget
startlog = tkinter.Button(m, text = 'Start logging data', command = log_data)
startlog.grid()

m.mainloop() 
