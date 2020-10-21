/*******************************************************************************
*	File Name: heater_fsm.ino
*
* File Description: This describes the behaviour of the heater finite state machine
*
* History: Created by Valentine
*******************************************************************************/
// Includes --------------------------------------------------------------------
#include "HeaderFiles/heater_fsm.h"

// Global Variables ------------------------------------------------------------
const float f_thresh_temp = 50.0;
const float f_max_temp = 150.0;

int i_incomingByte;
enum State_enum {HEATER_IDLE,HEATER_HEATING_SAFE,HEATER_HEATING_UNSAFE,HEATER_MAX_IDLE,HEATER_COOLING_UNSAFE};

//state initialize
uint8_t state = HEATER_IDLE;

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
  pinMode(LEDG,OUTPUT);
  pinMode(LEDB,OUTPUT);
  pinMode(LEDR,OUTPUT);
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
  heater_fsm(read_temp()); //check_temp for fsm changes
  delay(10);
}

/*******************************************************************************
*	Function Name: heater_fsm()
*
* Function inputs: current_temp (uint16_t)
*	Function outputs: None
*
* Function Description: This function handles the state transitions
*
* History: Created by Valentine
*******************************************************************************/
void heater_fsm(uint16_t current_temp) {
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    i_incomingByte = Serial.read();
  }

  switch(state){
    case HEATER_IDLE:
      if(i_incomingByte == 'S'){                  //user input from GUI or serial
        state = HEATER_HEATING_SAFE;
      }
      else{
        state = HEATER_IDLE;
      }
      break;

    case HEATER_HEATING_SAFE:
      if(current_temp < f_thresh_temp){
        state = HEATER_HEATING_SAFE;
      }
      else{
        state = HEATER_HEATING_UNSAFE;
      }
      break;

    case HEATER_HEATING_UNSAFE:
      if(current_temp == f_max_temp){
        state = HEATER_MAX_IDLE;
      }
      else{
        state = HEATER_HEATING_UNSAFE;
      }
      break;

    case HEATER_MAX_IDLE:
      if(i_incomingByte == 'G'){
        state = HEATER_COOLING_UNSAFE;
      }
      else{
        state = HEATER_MAX_IDLE;
      }
      break;

    case HEATER_COOLING_UNSAFE:
      if(current_temp < f_thresh_temp){
        state = HEATER_IDLE;
      }
      else{
        state = HEATER_COOLING_UNSAFE;
      }
      break;
  }
}
/*******************************************************************************
*	Function Name: read_temp()
*
* Function inputs: None
*	Function outputs: Temp
*
* Function Description: Reads the temperature from the sensor and returns the value
*
* History: Created by Valentine
*******************************************************************************/
uint16_t read_temp(){
  //read temp sensor
  //0.5V/50C
  analogReadResolution(16);
  volt = analogRead(A0);
  mvolt = volt*1000/65535
  temp = (mvolt-500)/10;
  return temp;
}

/*******************************************************************************
*	Function Name: apply_pwm()
*
* Function inputs: None
*	Function outputs: none
*
* Function Description: Applies PWM signal
*
* History: Created by Valentine
*******************************************************************************/
uint16_t apply_pwm(){
  //apply pwm signal
}

/*******************************************************************************
*	Function Name: stop_pwm()
*
* Function inputs: None
*	Function outputs: none
*
* Function Description: Stops PWM signal
*
* History: Created by Valentine
*******************************************************************************/
uint16_t stop_pwm(){
  //stop pwm signal
}

// States ----------------------------------------------------------------------
void heater_idle(){
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,LOW);
    digitalWrite(LEDB,HIGH);
}

void heater_heating_safe(){ //pwm starts
    apply_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDB,HIGH);
}

void heater_heating_unsafe(){
    apply_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDB,LOW);
}


void heater_max_idle(){
    apply_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDB,LOW);
}

void heater_cooling_unsafe(){ //pwm stops
    stop_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,LOW);
    digitalWrite(LEDB,LOW);
}
