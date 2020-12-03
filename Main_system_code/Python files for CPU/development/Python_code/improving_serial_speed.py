# -*- coding: utf-8 -*-
"""
FTU testing script d

Read the read me file

@author: Valentine
"""
import serial
from pprint import pprint
import sys
import json
import csv
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import smtplib 
from email.message import EmailMessage

EMAILADDRESS = "IvanovFTU2020@gmail.com"
EMAILPASSWORD = "FTU2020!@#"
CONTACTSFILE = "contacts.txt"
TESTRESULTFILE = "testing.csv"
TESTPLOT = "data_plot.png"
Email_body = "Test Passed \nAttached below are the test files"
Contacts_to_email = ""
COMPORT = "COM6"
BAUDRATE = 256000 #9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000

def Display_COM_PORTS():
    """
    Parameters
    ----------
    def Display_COM_PORTS : Displays available COM Ports on computer

    Returns 
    -------
    None.

    """
    print("Available Serial Ports : \n")
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    
    for port, desc, hwid in sorted(ports):
            print("{}: {}".format(port, desc))
    
    print("")
    return

#connecting serial port
while True:
    try:
        ser = serial.Serial(COMPORT, baudrate = BAUDRATE, timeout = 1) #9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000
        ser.close()
    except Exception as e:
        print()
        print(e)
        print("Unable to connect to serial Port :" + COMPORT + " \n")
        Display_COM_PORTS()
        COMPORT = input("Enter COMPORT to connect to : ")
        continue
    finally:
        print("\nSuccessfully connected to :" + COMPORT + "\n")
        break
        

ser = serial.Serial(COMPORT, baudrate = BAUDRATE, timeout = 1) 

            
#Plotting options
plt.style.use('fivethirtyeight')
#interactive mode on, In interactive mode block defaults to False. This will ensure that all of the figures are shown and this function immediately returns
plt.ion() 

#instruction string
original_instruction = '''{"id": 0,"description": "Testing","test_values": {"temperature": 120, "v_stress": -400, "test_time": 2, "magnetic_field": 5, "Test_start": 1, "Test_stop":0, "serial_rate": 200, "High speed test":0},"measurement_params": {"temperature": {"unit": "C"},"v_stress": {"unit": "mV"},"test_time": {"unit": "seconds"},"magnetic_field": {"unit": "mT"},"serial_rate": {"unit": "milliseconds"}}}'''
instruction_set =json.loads(original_instruction) #getting the json string from the instruction set



#opening csv file to store values
#this creates a new file called "testing.csv"


#fields for csv file
fields = ['Test id','Time (s)', 'Temperature (C)', 'magnetic field (mT)','Test stop','Test error','Error message','AIN0 (mV)','AIN1 (mV)','AIN2 (mV)','AIN3 (mV)','AIN4 (mV)','AIN5 (mV)','AIN6 (mV)','AIN7 (mV)',
          'AIN8 (mV)','AIN9 (mV)','AIN10 (mV)','AIN11 (mV)','AIN12 (mV)','AIN13 (mV)','AIN14 (mV)','AIN15 (mV)','ADC OFFSET','ADC VCC (V)','ADC Temperature (C)','ADC Gain',
          'ADC Reference (V)']

with open(TESTRESULTFILE,'w', newline='') as new_file:
    csv_writer = csv.writer(new_file)
    #write headers to the csv file
    csv_writer.writerow(fields)


def animate(i): #this function reads data from the csv file and plots it onto the figure
    data1 = pd.read_csv(TESTRESULTFILE) #get data from csv
    x = data1[x_axis_input]
    y1 = data1[y_axis_input]
    #y2 = data['data_2']

    plt.cla() #clear axes

    plt.plot(x, y1, label= y_axis_input)
    plt.title("A graph of " + y_axis_input + " vs " + x_axis_input)
    #plt.plot(x, y2, label='Channel 2')
    
    plt.legend(loc='upper left') #to be used if there is a label eg plt.plot(x, y1, label='data_0')
    plt.tight_layout()
        
print("Hi Welcome to the FUT python file: ")

print("__________________________TEST SETTINGS__________________________")
testID = input("Enter the Test ID: ")

testDescription = input("Enter the description: ")

while True:
    try: 
        high_speed_test = int(input("Enter '1' for a high speed test \n Enter '0' for a normal test"))
    except ValueError:
        print("Please enter an integer")
        #start loop again incase of incorrect data
        continue
    else:
        #exit if all is successful
        break

while True:
    try: 
        temperature = int(input("Enter desired temperature in C: "))
    except ValueError:
        print("Please enter an integer")
        #start loop again incase of incorrect data
        continue
    else:
        #exit if all is successful
        break
    
while True:
    try: 
        v_stress = int(input("Enter desired voltage stress in mV: "))
    except ValueError:
        print("Please enter an integer")
        #start loop again incase of incorrect data
        continue
    else:
        #exit if all is successful
        break
    
while True:
    try: 
        if (high_speed_test == 0):
            test_time = float(input("Enter desired test time in minutes: "))
        else :
            test_time = float(input("Enter desired test time in minutes: "))/60.00  #for high speed test, we want them to enter duration in seconds
        
    except ValueError:
        print("Please enter a float") 
        #start loop again incase of incorrect data
        continue
    else:
        #exit if all is successful
        break
    
while True:
    try: 
        magnetic_field = int(input("Enter desired magnetic field in mT: "))
    except ValueError:
        print("Please enter an integer")
        #start loop again incase of incorrect data
        continue
    else:
        #exit if all is successful
        break
    
while True:
    try: 
        if (high_speed_test == 0):
            serial_rate = float(input("Enter the full data samples returned per second (max 20): "))
        else :
            serial_rate = 20 #for high speed test, we want the fastest rate back
    except ValueError:
        print("Please enter a float")
        #start loop again incase of incorrect data
        continue
    else:
        #exit if all is successful
        break

    #apply inputs to the json file 
instruction_set['id'] = testID
instruction_set['description'] = testDescription
instruction_set['test_values']['High speed test']=high_speed_test
instruction_set['test_values']['temperature']=temperature
instruction_set['test_values']['v_stress']=v_stress
instruction_set['test_values']['test_time']=test_time
instruction_set['test_values']['magnetic_field']=magnetic_field
instruction_set['test_values']['serial_rate']= (int) (1000 / serial_rate) # multiply by 1000 to set to milliseconds


instruction_str = json.dumps(instruction_set)
print("Test parameters : \n")
pprint(instruction_str)  #show instruction set to user12

#get info from User about plotting the graph
print("\n__________________________Graph Plotting settings__________________________ \n\n")

#user interaction
while True:
    try: 
        user_in = str(input("Would you like to plot a graph \n Options (y/n)  :"))
        
        if ('y' in user_in or 'n'  in user_in):
            if ('y' in user_in ):
                plot_option = True #plotting flag
            else: plot_option = False
            
            break
        else:
            raise Exception
            
    except Exception as e:
        print("\n Please enter either y or n ")
        print(e)
        #start loop again incase of incorrect data
        continue

if (plot_option):
    print("Options for axes are : " )
    print( fields)

    while True :
        try:
            x_axis_input = str(input("Enter value on x_axis : "))
            y_axis_input = str(input("Enter value on y_axis : "))

            if (x_axis_input not in fields or y_axis_input not in fields):
                raise Exception
        except Exception as e:
            print(" \n Option entered not valid")
            print (e)
            #start loop again incase of incorrect data
            continue
        else: break



def getValues(): #this function reads values from the Arduino
    arduinoData = ser.readline().decode('ascii')
    return arduinoData

#userInput = input('Send data to Arduino ? (y/n) : ')

while True:
    try: 
        userInput = input('Send data to Arduino ? (y/n) : ')
        
        if ('y' in userInput or 'n'  in userInput):
            break
        else:
            raise Exception
            
    except Exception as e:
        print("Please enter either y or n ")
        print(e)
        #start loop again incase of incorrect data
        continue

if userInput == 'y':
        instruction = 'instruction'
        ready_signal = 'ready'
        
        ser.reset_input_buffer() #clear serial buffer
        ser.write(instruction.encode())
        
        #check if arduino has sent ready signal
        #wait for data
        while (ser.inWaiting() < 1):
            pass
    
        if (ready_signal in getValues()): #wait for ready signal
            ser.write(instruction_str.encode())
            ser.reset_input_buffer()
        else :
            print(getValues())
            ser.close()

            raise Exception("no ready signal from arduino")
        
        #check if the instruction matches
        #wait for data
        while (ser.inWaiting() < 1) :
            pass
        #wait for ready signal
        if (instruction_str in getValues()): 
            print("Values match")
        else :
            print(getValues())
            ser.close()
            raise Exception("instructions do not match the one in arduino") 
            
elif userInput == 'n':
    raise Exception("Ending test")
else :
    raise Exception("Invalid Input") 


#================================================PROGRESS BAR ===========================================================
if (high_speed_test == 1):
    #place a progress bar to wait for the MCU to finish the test 
    pass

#run while loop as test goes on
#==============================================TEST RUNNING============================================================== 
try :
    while(1):
        
        #wait for arduino data and store values in a csv file
    
        #userInput = input('Get data point?')
    
        #if userInput == 'y':
            
            #ser.write(b'g' )
            
            #wait for data
            while (ser.inWaiting() < 1):
                # ser.reset_output_buffer()
                # ser.write(b'g' )
                pass
            
            data =json.loads(getValues()) #getting the json string
            
            #Json format
            #{"test id":"Test52","test run":0,"test error":0,
            #"ADC data":[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28],
            #"test data":[135,50,2]}
            
            #write to csv file
            with open(TESTRESULTFILE,'a', newline='') as new_file: #a+ is for appending files
                csv_writer = csv.writer(new_file)
                
                loops = data['number of adc arrays']
                for i in range(0,loops,1) :
                    test_data = "test data"+ str(i)
                    adc_data = "ADC data"+ str(i)
                    
                    
                    print ("\n Test ID: ", end="")
                    print(data['test id'])
                    
                    print ("Test stop: ", end="")  
                    print(data['test stop'])
                    
                    print ("ADC data: ", end="")
                    print(data[adc_data])
                    
                    print ("Time, s: ", end="") 
                    print(data[test_data][0])
                    
                    print ("Temperature, C: ", end="") 
                    print(data[test_data][1])
                    
                    print ("Magnetic Field, mT: ", end="") 
                    print(data[test_data][2])
                    
                    print ("Test error: ", end="") 
                    print(data['test error']) 
                    
                    csv_writer.writerow([data["test id"],data[test_data][0],
                                     data[test_data][1],data[test_data][2],data["test stop"],
                                     data["test error"],data["error message"],
                                     data[adc_data][0],data[adc_data][1],data[adc_data][2],
                                    data[adc_data][3],data[adc_data][4],data[adc_data][5],
                                    data[adc_data][6],data[adc_data][7],data[adc_data][8],
                                    data[adc_data][9],data[adc_data][10],data[adc_data][11],
                                    data[adc_data][12],data[adc_data][13],data[adc_data][14],
                                    data[adc_data][15],data[adc_data][16],data[adc_data][17],
                                    data[adc_data][18],data[adc_data][19],data[adc_data][20]])
                    
                    # csv_writer.writerow([data["test id"],data[test_data][0],
                    #                  data[test_data][1],data[test_data][2],data["test stop"],
                    #                  data["test error"],data["test error message"],
                    #                  data[adc_data][0],data[adc_data][1],data[adc_data][2],
                    #                  data[adc_data][3],data[adc_data][4],data[adc_data][5],
                    #                  data[adc_data][6],data[adc_data][7],data[adc_data][8],
                    #                  data[adc_data][9],data[adc_data][10],data[adc_data][11],
                    #                  data[adc_data][12],data[adc_data][13],data[adc_data][14],
                    #                  data[adc_data][15],data[adc_data][16],data[adc_data][17],
                    #                  data[adc_data][18],data[adc_data][19],data[adc_data][20],
                    #                  data[adc_data][21],data[adc_data][22],data[adc_data][23],
                    #                  data[adc_data][24],data[adc_data][25],data[adc_data][26],
                    #                  data[adc_data][27],data[adc_data][28]])

                  
            #read plotting values
            if (plot_option):
                plt.ion()    
                ani = FuncAnimation(plt.gcf(), animate)
                plt.tight_layout()#Adjust the padding between and around subplots.
                plt.show()  #Display all open figures.
                plt.pause(0.000001) #Run the GUI event loop for interval seconds.
                
            time.sleep(0.000001)            
                
            #check for test completion
            if (data['test stop'] == 1 and data['test error'] == 0 ):
                print("Test has sucessfully completed")
                Email_body = "Test has sucessfully completed \nAttached below are the test files"
                break
            elif (data['test stop'] == 1 and data['test error'] == 1 ):
                print("Test has stopped due to Device error, error message :")                
                print(data['error message'])
                Email_body = "Test has stopped due to Device error, \nDevice Error message :" + data['error message'] +"\nAttached below are the test files"
                break
     
            ser.reset_input_buffer()
            

                
    ser.close()
except Exception as e:
    print()
    print(e)
    print("Error in communicating with device")
    Email_body = "Test Failed : \n Reason :Error in communicating with device \nCPU Details:" + str(e) + "\nAttached below are the test files"
    pass
    
#test is done running and now e-mailing is left 
#==============================================EMAIL SECTION==============================================================       


if (plot_option):
        plt.savefig(TESTPLOT)

try:
    
    with open(CONTACTSFILE) as f:
        line = f.readline()
        while line:
            Contacts_to_email = Contacts_to_email + line.strip() + "," #storing emails in a string list
            line = f.readline()
        Contacts_to_email = Contacts_to_email[:-1] #delete last "," character
            
    print("Sending E-mail to : " + Contacts_to_email + "\n")
    
    #creating email structure
    msg = EmailMessage()
    msg['Subject'] = 'FTU Test Results'
    msg['From'] = EMAILADDRESS
    msg['To'] = Contacts_to_email
    msg.set_content(Email_body)
    
    #Hard coded method to add attachments reduce code, for smoother code
    #check https://varunver.wordpress.com/2017/08/23/django-create-csv-and-email-as-attachment/
    #attaching documents
    if (plot_option): #attach plot only if the plotting was asked for
        with open(TESTPLOT, 'rb') as f:
            file_data = f.read()
        msg.add_attachment(file_data, maintype='image', subtype='png', filename=TESTPLOT )
    with open(TESTRESULTFILE, 'rb') as f:
        file_data_2 = f.read()
    msg.add_attachment(file_data_2, maintype='doc', subtype='csv', filename=TESTRESULTFILE )
    
    
    with smtplib.SMTP_SSL('smtp.gmail.com',465) as smtp:
        #465 is the port number
        #you can use environmental addresses to hide this information
        smtp.login(user=EMAILADDRESS,password=EMAILPASSWORD)
        smtp.send_message(msg)
except Exception as e:
    print(e)
    print("Issue with E-mail sending")
    pass

else:
    print("E-mailing successful")

sys.exit()


























        


