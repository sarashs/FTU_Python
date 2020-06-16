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

#plotting settings
plt.ion() #interactive mode on

#1st button
#implement a flag to raise flag, if theres input 
def log_data(value,data_col):
    ser.reset_input_buffer() #clear buffer before plotting for extraneous data
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
            time.sleep(data_col)
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
master.minsize(200,200) #dimensions

#entry for collection rate
data_col = IntVar()
l1 = Label(master, text = "Enter collection rate in hz")
l1.grid()
e1 = Entry(master, textvariable = data_col)
e1.grid()
sample_rate = e1.get()

#entry for arduino control
onoff = StringVar()

l2 = Label(master, text = "Enter H/L to turn LED on/off")
l2.grid()

e2 = Entry(master, textvariable = onoff)
e2.grid()

def setCheckButtonText():
    if onoff.get() == 'H': 
        ser.write(bytes('H', 'UTF-8'))
    elif onoff.get() == 'L':
        ser.write(bytes('L', 'UTF-8'))
    else:
        ser.close()
        
#selects radiobutton
def func1():
    if v.get() == 1:
        log_data(1,data_col)

    else:
        log_data(2)

#Data Logger Button Widget
#Log
v = IntVar()

a = Radiobutton(master, text = 'Linear Plot', value = 1, variable = v)
a.grid()
b = Radiobutton(master, text = 'Logarithmic Plot', value = 2, variable = v)
b.grid()

startlog = Button(master, text = 'Start logging data', command = func1)
startlog.grid()

led_button = Button(master, text = 'Turn led on/off', command = setCheckButtonText)
led_button.grid()

master.mainloop() 
