/*******************************************************************************
*	File Name: mag_pwm.ino
*
* File Description: This file controls the PWM of the magnetic field feedback loop
*                     to maintain a constant field.
*
*                   PWM duty cycle is based on the reading from the mag sensor
*                      and shifted to 8 bits from 16
*
* History: Created by Valentine
*******************************************************************************/
// Includes --------------------------------------------------------------------
#include "HeaderFiles/mag_pwm.h"

// Global Variables ------------------------------------------------------------
uint16_t mag_value; //value read by the sensor

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
  Serial.begin(9600); //use 9600 baud rate as placeholder this can be changed later.
  pinMode(6,OUTPUT); //PWM pin
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
  mag_value = analogRead(A1); //read from the output of the pin. this value will be sampled at 16 bits
  analogWrite(pwmPin,mag_value/256); //output PWM signal to the PWM pin, map 0-65535 to 0-255
}
