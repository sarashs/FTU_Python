/*******************************************************************************
*	File Name: pin_toggle.ino
*
* File Description: This file controls the temperature sensor
*
* History: Created by Valentine
*******************************************************************************/
// Includes --------------------------------------------------------------------
#include "HeaderFiles/temp_sensor.h"

// Global Variables ------------------------------------------------------------
int sensorValue = 0;   // variable to store the value coming from the sensor
float temperature = 0.0;
float voltage = 0.0;

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
  Serial.begin(9600);
  analogReadResolution(10); //10 bit resolution
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
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  voltage = sensorValue * (3300/1024); // in milliVolt
  //Serial.print(" voltage = ");
  //Serial.print(voltage);
  temperature = (voltage - 500 ) / 10;
  //Serial.print(" temperature(C) = ");
  Serial.println(temperature);
  delay(1000); //adjust sampling rate
}
