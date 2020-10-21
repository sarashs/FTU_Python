/*******************************************************************************
*	File Name: pin_toggle.ino
*
* File Description: This file controls the voltage toggle of pin connecting to
*                     DAC output, user enters a command and it will toggle
*
*                   PWM duty cycle is based on the reading from the mag sensor
*                      and shifted to 8 bits from 16
*
* History: Created by Valentine
*******************************************************************************/
// Includes --------------------------------------------------------------------
#include "HeaderFiles/mag_pwm.h"

// Global Variables ------------------------------------------------------------
String ser_chars = "" ; //initialize for incoming serial strings

// Functions -------------------------------------------------------------------
/*******************************************************************************
*	Function Name: setup()
*
* Function inputs: None
*	Function outputs: None
*
* Function Description: This function runs once upon execution
*
* History: Created by Valentine
*******************************************************************************/
void setup() {
  // put your setup code here, to run once:
  pinMode(togglepin,OUTPUT); //this will be the pin we will toggle
  Serial.begin(9600); //baud rate is place holder
}

/*******************************************************************************
*	Function Name: serialEvent()
*
* Function inputs: None
*	Function outputs: None
*
* Function Description: This function does something....
*
* History: Created by Valentine
*******************************************************************************/
void serialEvent(){
  if(Serial.available()){
    ser_chars = Serial.readString();
    Serial.println(ser_chars);
  }
}

/*******************************************************************************
*	Function Name: loop()
*
* Function inputs: None
*	Function outputs: None
*
* Function Description: This function runs repeatedly
*
* History: Created by Valentine
*******************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  //upon user command toggle pin
  serialEvent();

  bool pinstate = digitalRead(togglepin);

  if(ser_chars.indexOf("TOGGLEPIN") != -1){ //if this string is entered, it won't return -1


    digitalWrite(togglepin, !pinstate); //write it to the pin
    Serial.println("Current state: " );
    Serial.println(pinstate);

    ser_chars.replace("TOGGLEPIN",""); //empty the string buffer to enter a new string
  }
}
