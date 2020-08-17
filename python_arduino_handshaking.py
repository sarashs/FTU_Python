  
import serial
import json
import csv

#opening csv file to store values
#this creates a new file called "testing.csv"

#fields for csv file
fields = ['Test id','TIME(s)', 'Temperature(C)', 'magnetic field(mT)','Test run','Test error','DIFF0','DIFF1','DIFF2',
          'DIFF3', 'DIFF4', 'DIFF5','DIFF6','DIFF7','AIN0','AIN1','AIN2','AIN3','AIN4','AIN5','AIN6','AIN7',
          'AIN8','AIN9','AIN10','AIN11','AIN12','AIN13','AIN14','AIN15','OFFSET','VCC','TEMP','GAIN',
          'REF']

with open('testing.csv','w', newline='') as new_file:
    csv_writer = csv.writer(new_file)
    #write headers to the csv file
    csv_writer.writerow(fields)
    

#open serial
ser = serial.Serial('COM3', baudrate = 9600, timeout = 1)

def getValues():
    
    #ser.write(b'g')
    arduinoData = ser.readline().decode('ascii')
    return arduinoData



while(1):

    userInput = input('Get data point?')

    if userInput == 'y':
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
                                 data['test data'][1],data['test data'][2],data['test run'],
                                 data['test error'],
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
        
    if userInput == 'l':
        instruction = 'instruction'
        ready_signal = 'ready'
        instruction2 = '''{"id": 0,"description": "Apply required stress for 2 hours, etcetera","test_values": {"temperature": 120, "v_stress": -400, "test_time": 5, "magnetic_field": 5, "Test_start": 1, "Test_stop":0, "serial_rate": 1500},"measurement_params": {"temperature": {"unit": "C"},"v_stress": {"unit": "mV"},"test_time": {"unit": "seconds"},"magnetic_field": {"unit": "mT"},"serial_rate": {"unit": "milliseconds"}}}'''
        
        ser.write(instruction.encode())
        
        #check if arduino has sent ready signal
        #wait for data
        while (ser.inWaiting() < 1):
            pass
    
        if (ready_signal in getValues()): #wait for ready signal
            ser.write(instruction2.encode())
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
        if (instruction2 in getValues()): 
            print("Values match")
        else :
            print(getValues())
            ser.close()
            raise Exception("instructions do not match the one in arduino")
        
    
    if userInput == 'e':
        ser.close()
    
    if userInput == 'p':
        #print the values from Arduino
        print(getValues())
    
        