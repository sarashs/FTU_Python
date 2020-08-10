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
import datetime
from datetime import timedelta
import json
from pymongo import MongoClient
from pymongo import ASCENDING

#serial communication
port = 'COM5'
ser = serial.Serial(port,baudrate=9600)

#start mongod on the command line then
#create a MongoClient to the running mongod instance 
client = MongoClient('mongodb://localhost:27017/')
db = client.my_logs
log_collection = db.log
log_collection.create_index([("timestamp", ASCENDING)])

#plotting settings
plt.ion() #interactive mode on

#1st button
#implement a flag to raise flag, if theres input 
def log_data(value,data_rate,data_dur_hour,data_dur_min,data_dur_sec,max_volt,min_volt):
    ser.reset_input_buffer() #clear buffer before plotting for extraneous data
    plot_window = 100 #plot window width
    y_var = np.array(np.zeros([plot_window]))

    plt.ion() #interactive mode on
    fig, ax = plt.subplots()
    line, = ax.plot(y_var)

    #take spinbox values and convert them to unix-time in seconds
    time_del_obj = timedelta(hours = int(data_dur_hour), minutes = int(data_dur_min), seconds = int(data_dur_sec))
    
    data_duration = time_del_obj.total_seconds() #duration in seconds
    start_time = time.time() #grab the time in unix at the the time log_data is executed
    
    if(value==1):
        ax.set_yscale('linear')
    elif(value == 2):
        ax.set_yscale('log')

    num_logs = 0 #number of datapoints counter
    while(time.time() - start_time < data_duration): #while duration is greater than the difference of current and start time
        try: 
            ser_bytes = ser.readline()            
            time.sleep(data_rate) #collection rate controls the data.csv length

            try:
                decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
                print(decoded_bytes)
            except:
                continue
            
            #data write
            """Log data to MongoDB log"""
            entry = {}
            entry['timestamp'] = datetime.datetime.utcnow()
            entry['data'] = decoded_bytes
            log_collection.insert_one(entry)
            
            y_var = np.append(y_var,decoded_bytes)
            y_var = y_var[1:plot_window+1]
            line.set_ydata(y_var)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()
            num_logs = num_logs + 1

            voltage = decoded_bytes*10 + 500
            print(voltage)
            
            if(voltage > max_volt): #voltage bound logic
                break
            elif(voltage < min_volt):
                break
            else:
                continue
            
        except KeyboardInterrupt: #ctrl+c to stop
            print('interrupted!')
            break
        
    print(data_rate)
    print(max_volt)
    print(min_volt)
    
    log_acc = num_logs/(data_duration/data_rate) * 100
    lost_points =  (data_duration/data_rate) - num_logs
    print("Total data points logged: ",num_logs)
    print("Lost data points: ", lost_points)
    print("Total logging accuracy: ", log_acc,"%")

def fileDialogRead():
    #prompt browse file
    filename = filedialog.askopenfilename(initialdir = "/",title = "Select file",filetypes = (("json files",".json"),("all files",".*")))
    #read parameters in that file
    with open(filename, "r") as read_file:
        data = json.load(read_file) #deserialize json file into python object

    json_string = json.dumps(data) #encode into string with dumps
   
    json_string.replace('"','\\"') #add escape chars
    
    json_bytes = json_string.encode() #convert to bytes before sending
    
    print("String sent: ")
    print(json_bytes)
    print("\n")
    ser.write(json_bytes) #write to serial
    time.sleep(2) # with the port open, the response will be buffered 
                  # so wait a bit longer for response here. without this line
                  # the arduino will not send back anything.

    # Serial read section

    msg = ser.read(ser.inWaiting()) # read everything in the input buffer
                                    # inWaiting gets the total number of bytes from the input buffer
    print(msg)
    
    read_file.close() #close file

#Widgets
master = Tk() #Main Window
master.minsize(200,200) #dimensions

#entry for collection rate
data_col = IntVar()
record_data_label = Label(master, text = "Record data every n seconds")
record_data_label.grid(row=0,column=0)

entry_datacol_box = Entry(master, textvariable = data_col)
entry_datacol_box.grid(row=0,column=1)

#entry for test duration // this doesn't log the right amount of points
label_1 = Label(master, text="Enter the desired test duration.")
label_1.grid()

label_2 = Label(master, text="Hours")
label_2.grid(row=2,column=0)
data_dur_hour = Spinbox(master, from_ = 0, to = 1000)
data_dur_hour.grid(row=2,column=1)

label_3 = Label(master, text="Minutes")
label_3.grid(row=3,column=0)
data_dur_min = Spinbox(master, from_ = 0, to = 59)
data_dur_min.grid(row=3,column=1)

label_4 = Label(master, text="Seconds")
label_4.grid(row=4,column=0)
data_dur_sec = Spinbox(master, from_ = 0, to = 59)
data_dur_sec.grid(row=4,column=1)


#entry for test duration
max_temp = DoubleVar()
max_temp_label = Label(master, text = "Enter the maximum temperature of the test in Celsius.")
max_temp_label.grid(row=5,column=0)

max_temp_box = Entry(master, textvariable = max_temp)
max_temp_box.grid(row=5,column=1)

#selects radiobutton selection
def radiobutsel():
    if v.get() == 1:
        log_data(1,data_col.get(),data_dur_hour.get(),data_dur_min.get(),data_dur_sec.get(),max_volt.get(),min_volt.get())
        
    elif v.get() == 2:
        log_data(2,data_col.get(),data_dur_hour.get(),data_dur_min.get(),data_dur_sec.get(),max_volt.get(),min_volt.get())


#Data Logger Radio Button Widgets
#Log
v = IntVar()

rad_label = Label(master, text="Choose the plot format.")
rad_label.grid(row=6,column=0)

a = Radiobutton(master, text = 'Linear Plot', value = 1, variable = v)
a.grid(row=6,column=1)
b = Radiobutton(master, text = 'Logarithmic Plot', value = 2, variable = v)
b.grid(row=7,column=1)

#Forward and Reverse Voltage Test Widgets
#entry for collection rate
max_volt = DoubleVar()
max_volt_label = Label(master, text = "Enter maximum voltage for the test to stop in millivolts.")
max_volt_label.grid(row=8,column=0)

max_volt_box = Entry(master, textvariable = max_volt)
max_volt_box.grid(row=8,column=1)

min_volt = DoubleVar()
min_volt_label = Label(master, text = "Enter minimum voltage for the test to stop in millivolts.")
min_volt_label.grid(row=9,column=0)

min_volt_box = Entry(master, textvariable = min_volt)
min_volt_box.grid(row=9,column=1)

startlog = Button(master, text = 'Start logging data', command = radiobutsel)
startlog.grid(row = 10, column = 0)

#load data parameters
data_load_button = Button(text = "Browse a file to send test parameters", command = fileDialogRead)
data_load_button.grid(row = 11, column = 0)

##send_instruction_set = Button(text = "Browse a file to send over the serial", command = sendInstructionList)
##send_instruction_set.grid(row = 12, column = 0)

master.mainloop() 
