import smtplib 
from email.message import EmailMessage

EMAILADDRESS = "IvanovFTU2020@gmail.com"
EMAILPASSWORD = "FTU2020!@#"
CONTACTSFILE = "contacts.txt"
TESTRESULTFILE = "testing.csv"
TESTPLOT = "plot.png"
Email_body = "Test Passed \nAttached below are the test files"
Contacts_to_email = ""

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
        
     
    
    