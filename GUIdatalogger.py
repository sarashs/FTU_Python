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
from tqdm import tqdm

#serial communication
ser = serial.Serial('COM5',baudrate=9600)

#plotting settings
plt.ion() #interactive mode on

#1st button
#implement a flag to raise flag, if theres input 
def log_data(value,data_rate,data_dur):
    ser.reset_input_buffer() #clear buffer before plotting for extraneous data
    plot_window = 100 #plot window width
    y_var = np.array(np.zeros([plot_window]))

    plt.ion() #interactive mode on
    fig, ax = plt.subplots()
    line, = ax.plot(y_var)
    
    start_time = time.time() #grab the time in unix at the the time log_data is executed
    
    if(value==1):
        ax.set_yscale('linear')
    else:
        ax.set_yscale('log')

    pbar = tqdm(total = time.time() - start_time) #total is the number of expected iterations)
    while(time.time() - start_time < data_dur): #while duration is greater than the difference of current and start time
        try:
            
            ser_bytes = ser.readline()            
            time.sleep(data_rate) #collection rate controls the data.csv length

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
            pbar.update(data_rate/data_dur) #we are manually updating the progress bar since we are wrapping the progress bar in a while loop
                           #we know when to terminate the while loop but it's not wrapped under a for loop
                          #since we need the try except branch to execute the keyboard interrupt
            
        except KeyboardInterrupt: #ctrl+c to stop
            print('interrupted!')
            break

#Widgets
master = Tk() #Main Window
master.minsize(200,200) #dimensions

#entry for collection rate
data_col = IntVar()
record_data_label = Label(master, text = "Record data every n seconds")
record_data_label.grid()

entry_datacol_box = Entry(master, textvariable = data_col)
entry_datacol_box.grid()

#entry for test duration // this doesn't log the right amount of points
data_dur = IntVar()
data_duration_label = Label(master, text = "Enter the test duration in seconds")
data_duration_label.grid()

entry_datadur_box = Entry(master, textvariable = data_dur)
entry_datadur_box.grid()

#entry for test duration
max_temp = DoubleVar()
max_temp_label = Label(master, text = "Enter the maximum temperature of the test in Celsius")
max_temp_label.grid()

max_temp_box = Entry(master, textvariable = max_temp)
max_temp_box.grid()


#entry for arduino control

#onoff = StringVar()

#l2 = Label(master, text = "Enter H/L to turn LED on/off")
#l2.grid()

#e2 = Entry(master, textvariable = onoff)
#e2.grid()

#def setCheckButtonText():
#    if onoff.get() == 'H': 
#        ser.write(bytes('H', 'UTF-8'))
#    elif onoff.get() == 'L':
#        ser.write(bytes('L', 'UTF-8'))
#    else:
#        ser.close()
        
#selects radiobutton selection
def radiobutsel():
    if v.get() == 1:
        log_data(1,data_col.get(),data_dur.get())

    else:
        log_data(2,data_col.get(),data_dur.get())


#Data Logger Radio Button Widgets
#Log
v = IntVar()

a = Radiobutton(master, text = 'Linear Plot', value = 1, variable = v)
a.grid()
b = Radiobutton(master, text = 'Logarithmic Plot', value = 2, variable = v)
b.grid()


startlog = Button(master, text = 'Start logging data', command = radiobutsel)
startlog.grid()

#Forward and Reverse Voltage Test Radio Button Widgets
#send a bit to select the voltage to apply

#stop data logging
#led_button = Button(master, text = 'Turn led on/off', command = setCheckButtonText)
#led_button.grid()

master.mainloop() 
