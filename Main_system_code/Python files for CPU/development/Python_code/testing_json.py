import random
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import time

import smtplib 
from email.message import EmailMessage

EMAILADDRESS = "IvanovFTU2020@gmail.com"
EMAILPASSWORD = "FTU2020!@#"
CONTACTSFILE = "contacts.txt"
TESTRESULTFILE = "data.csv"
TESTPLOT = "data_plot.png"
Email_body = "Test Passed \nAttached below are the test files"
Contacts_to_email = ""


#setting up the csv file
fieldnames = ["data_0", "data_1", "data_2", "data_3", "data_4", "data_5", "data_6", "data_7", "data_8"]
with open('data.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
    
    
(data_0, data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8) = (10,10,10,10,10,10,10,10,10)

plt.style.use('fivethirtyeight')
#interactive mode on, In interactive mode block defaults to False. This will ensure that all of the figures are shown and this function immediately returns
plt.ion() 

print("_____________Graph Plotting settings________________ \n\n")

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
    print( fieldnames)

    while True :
        try:
            x_axis_input = str(input("Enter value on x_axis : "))
            y_axis_input = str(input("Enter value on y_axis : "))

            if (x_axis_input not in fieldnames or y_axis_input not in fieldnames):
                raise Exception
        except Exception as e:
            print(" \n Option entered not valid")
            print (e)
            #start loop again incase of incorrect data
            continue
        else: break

def animate(i):
    data = pd.read_csv('data.csv') #get data from csv
    x = data[x_axis_input]
    y1 = data[y_axis_input]
    #y2 = data['data_2']

    plt.cla() #clear axes

    plt.plot(x, y1, label= y_axis_input)
    plt.title("A graph of " + y_axis_input + " vs " + x_axis_input)
    #plt.plot(x, y2, label='Channel 2')
    
    plt.legend(loc='upper left') #to be used if there is a label eg plt.plot(x, y1, label='data_0')
    plt.tight_layout()

count = 60
while (count > 0):
    #write new data into file
    with open('data.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    
        info = {
            "data_0": data_0,
            "data_1": data_1,
            "data_2": data_2,
            "data_3": data_3,
            "data_4": data_4,
            "data_5": data_5,
            "data_6": data_6,
            "data_7": data_7,
            "data_8": data_8
        }
    
        csv_writer.writerow(info)
        
        print(data_0, data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8)

    data_0 += 1
    data_1 = data_1 + random.randint(-6, 8)
    data_2 = data_2 + random.randint(-5, 9)
    data_3 = data_3 + random.randint(-4, 10)
    data_4 = data_4 + random.randint(-3, 6)
    data_5 = data_5 + random.randint(-2, 8)
    data_6 = data_6 + random.randint(-1, 6)
    data_7 = data_7 + random.randint(0, 8)
    data_8 = data_8 + random.randint(1, 6)

    if (plot_option):
        plt.ion()    
        ani = FuncAnimation(plt.gcf(), animate)
        plt.tight_layout()#Adjust the padding between and around subplots.
        plt.show()  #Display all open figures.
        plt.pause(0.00001) #Run the GUI event loop for interval seconds.
    
    count = count - 1
    time.sleep(0.01)

if (plot_option):
        plt.savefig(TESTPLOT)

try:
    
    with open(CONTACTSFILE) as f:
        line = f.readline()
        while line:
            Contacts_to_email = Contacts_to_email + line.strip() + "," #storing emails in a string list
            line = f.readline()
        Contacts_to_email = Contacts_to_email[:-1] #delete last "," character
    print("\n Test done")        
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

