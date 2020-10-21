/*******************************************************************************
*	File Name: led_control.ino
*
* File Description: This file simply controls the leds
*
* History: Created by Valentine
*******************************************************************************/
// Includes --------------------------------------------------------------------
#include "HeaderFiles/led_control.h"

// Global variables ------------------------------------------------------------
int incomingByte;

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
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
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
  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H') {
      digitalWrite(ledPin, HIGH);
    }
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == 'L') {
      digitalWrite(ledPin, LOW);
    }
  }
}
