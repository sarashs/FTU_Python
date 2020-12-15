#pragma once

#include "SPI.h"
#include "ArduinoJson.h"
#include "ArduinoJson.hpp"
#include "Arduino.h"
#include "math.h"
#include "strings.h"
#include "WString.h"
#include "time.h"

/************************************************************************/
/* GLOBAL VARIABLES AND CONSTANTS                                       */
/************************************************************************/
//system FSM states
#define IDLE 0 //idle state
#define ADC_UPDATE 1 //reading data from the ADC using SPI at intervals
#define SERIAL_UPDATE 2 //sending data back to the computer through serial at intervals
#define Heater_Update 3 //running the heater_FSM and get the updates
#define Magnetic_Field_update 4 //running the magnetic field FSM and get the updates

//volatile is for variables that can always change in the code
volatile int test_time_count = 0;     //seconds of test time elapsed
volatile int system_state = IDLE;    //initialize System FSM state

//Input data from the MCU
volatile int test_id = 0;
volatile bool test_start = false;
volatile bool test_stop = false;
volatile bool high_speed_test = false;

volatile float desired_temperature = 43;     //in C, Set to a default value but actual value comes from MCU instructions
volatile float desired_magnetic_field = 0;  //in mT, Set to a default value but actual value comes from MCU instructions
volatile int desired_time_for_test = 120;	 //time in seconds, Set to a default value but actual value comes from MCU instructions
volatile float desired_fpga_voltage =3200;     //in mV, Set to a default value but actual value comes from MCU instructions
volatile float serial_output_rate=3000;                 //rate to read data in ms, Set to a default value but actual value comes from MCU instructions


volatile bool test_error = false;
String error_message = ""; //Contains the error message to be sent to python, this catches errors
volatile bool serial_signal = false;

volatile float measured_temperature = 0; // in C
volatile float measured_magnetic_field = 0; // in mT
volatile float current_test_time = 0; //test time in minutes
//Variables for Magnetic field FSM
volatile int magnetic_pwm_duty; //duty cycle value 0-255
volatile int magnetic_fsm_state ; //idle state
//Variables for heater FSM
volatile float heater_pwm_duty; //duty cycle value 0-255
volatile int heater_fsm_state; //idle state
volatile int starting_test_count;	



/************************************************************************/
/* OTHER FUNCTIONS                                                      */
/************************************************************************/

/**
 * \@brief Function raises MCU error, updates the error message
 * 
 * \@param error_text -> Error message as String
 * 
 * \@return void
 */
void raise_mcu_error(String error_text){
	//This function raises and error
	test_error = true;
	error_message.concat(error_text + " ");
	return;
}

