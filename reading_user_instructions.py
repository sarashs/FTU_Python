# -*- coding: utf-8 -*-
"""
Communicating to user and placing varibales into a json file

@author: Valentine
"""
import serial
import json
import csv

#connecting serial port
ser = serial.Serial('COM3', baudrate = 115200, timeout = 1) #9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000

original_instruction = '''{"id": 0,"description": "Testing","test_values": {"temperature": 120, "v_stress": -400, "test_time": 2, "magnetic_field": 5, "Test_start": 1, "Test_stop":0, "serial_rate": 200},"measurement_params": {"temperature": {"unit": "C"},"v_stress": {"unit": "mV"},"test_time": {"unit": "seconds"},"magnetic_field": {"unit": "mT"},"serial_rate": {"unit": "milliseconds"}}}'''

instruction_set =json.loads(original_instruction) #getting the json string from the instruction set

#opening csv file to store values
#this creates a new file called "testing.csv"

#fields for csv file
fields = ['Test id','TIME(s)', 'Temperature(C)', 'magnetic field(mT)','Test stop','Test error','Error message','DIFF0','DIFF1','DIFF2',
          'DIFF3', 'DIFF4', 'DIFF5','DIFF6','DIFF7','AIN0','AIN1','AIN2','AIN3','AIN4','AIN5','AIN6','AIN7',
          'AIN8','AIN9','AIN10','AIN11','AIN12','AIN13','AIN14','AIN15','OFFSET','VCC','TEMP','GAIN',
          'REF']

with open('testing.csv','w', newline='') as new_file:
    csv_writer = csv.writer(new_file)
    #write headers to the csv file
    csv_writer.writerow(fields)
        
print("Hi Welcome to the FUT python file: ")

testID = input("Enter the Test ID: ")

testDescription = input("Enter the description: ")

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
        test_time = int(input("Enter desired test time in minutes: "))
    except ValueError:
        print("Please enter an integer")
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
        serial_rate = float(input("Enter the rate at which you want to record data in seconds: "))
    except ValueError:
        print("Please enter an integer")
        #start loop again incase of incorrect data
        continue
    else:
        #exit if all is successful
        break
 #apply to the json file 
instruction_set['id'] = testID
instruction_set['description'] = testDescription
instruction_set['test_values']['temperature']=temperature
instruction_set['test_values']['v_stress']=v_stress
instruction_set['test_values']['test_time']=test_time
instruction_set['test_values']['magnetic_field']=magnetic_field
instruction_set['test_values']['serial_rate']=serial_rate*1000


instruction_str = json.dumps(instruction_set)
print(instruction_str)

def getValues():
    arduinoData = ser.readline().decode('ascii')
    return arduinoData

userInput = input('Send data to Arduino ? (y/n) :')

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



#run while loop as test goes on
while(1):
    
    #wait for arduino data and store values in a csv file

    #userInput = input('Get data point?')

    #if userInput == 'y':
        
        ser.write(b'g' )
        
        #wait for data
        while (ser.inWaiting() < 1):
            pass
        data =json.loads(getValues()) #getting the json string
        print(type(data))
        print (data)   
        
        #Json format
        #{"test id":"Test52","test run":0,"test error":0,
        #"ADC data":[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28],
        #"test data":[135,50,2]}
        
        #write to csv file
        with open('testing.csv','a', newline='') as new_file: #a+ is for appending files
            csv_writer = csv.writer(new_file)
            csv_writer.writerow([data['test id'],data['test data'][0],
                                 data['test data'][1],data['test data'][2],data['test stop'],
                                 data['test error'],data['error message'],
                                 data['ADC data'][0],data['ADC data'][1],data['ADC data'][2],
                                 data['ADC data'][3],data['ADC data'][4],data['ADC data'][5],
                                 data['ADC data'][6],data['ADC data'][7],data['ADC data'][8],
                                 data['ADC data'][9],data['ADC data'][10],data['ADC data'][11],
                                 data['ADC data'][12],data['ADC data'][13],data['ADC data'][14],
                                 data['ADC data'][15],data['ADC data'][16],data['ADC data'][17],
                                 data['ADC data'][18],data['ADC data'][19],data['ADC data'][20],
                                 data['ADC data'][21],data['ADC data'][22],data['ADC data'][23],
                                 data['ADC data'][24],data['ADC data'][25],data['ADC data'][26],
                                 data['ADC data'][27],data['ADC data'][28]])
            
        #check for test completion
        if (data['test stop'] == 1 and data['test error'] == 0 ):
            print("Test has sucessfully completed")
            break
        elif (data['test stop'] == 1 and data['test error'] == 1 ):
            print("Test has stopped due to error, error message :")
            print(data['error message'])
            break
            
            
            
        
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(b'd')
        
         #wait for data
        while (ser.inWaiting() < 1):
            pass
    
        if ("done" in getValues()): #wait for ready signal
            ser.reset_input_buffer()
        else :
            print(getValues())
            ser.close()

            raise Exception("no ready signal from arduino")
        
ser.close()






























        


