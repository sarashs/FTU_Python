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
from tkinter import *

#serial communication
ser = serial.Serial('COM5',baudrate=9600)
ser.write(b'yo')
ser.flushInput() #clear queue to avoid data overlap

#plotting settings
plot_window = 500 #plot window width
y_var = np.array(np.zeros([plot_window]))

plt.ion() #interactive mode on

#1st button
def log_data(value):
    plot_window = 100 #plot window width
    y_var = np.array(np.zeros([plot_window]))

    plt.ion() #interactive mode on
    fig, ax = plt.subplots()
    line, = ax.plot(y_var)
    
    if(value==1):
        ax.set_yscale('linear')
    else:
        ax.set_yscale('log')
    
    while True:
        try:
            ser_bytes = ser.readline()
            #time.sleep(sample_rate)
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
master = Tk() #Main Window
master.minsize(300,300) #dimensions

#entry for collection rate
e = Entry(master)
e.grid()
sample_rate = e.get()

#selects radiobutton
def func1():
    if v.get() == 1:
        log_data(1)
        return 0
    else:
        log_data(2)
        return 1

#Data Logger Button Widget
#Log
v = IntVar()
v.set(1)

a = Radiobutton(master, text = 'Linear Plot', value = 1, variable = v)
a.grid()
b = Radiobutton(master, text = 'Logarithmic Plot', value = 2, variable = v)
b.grid()

button_sel = v.get() #get current value of v

startlog = Button(master, text = 'Start logging data', command = func1)
startlog.grid()

master.mainloop() 
