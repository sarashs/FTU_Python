/*******************************************************************************
*	File Name: heater_fsm.h
*
* File Description: Header file for heater_fsm.ino
*
* History: Created by Rohit Singh, 2020-10-21
*******************************************************************************/
// Macros ----------------------------------------------------------------------
#define LEDG = 8
#define LEDR = 9
#define LEDB = 11

// Function Prototypes ---------------------------------------------------------
void heater_fsm(uint16_t current_temp);
void heater_idle();
void heater_heating_safe();
void heater_heating_unsafe();
void heater_max_idle();
void heater_cooling_unsafe();

uint16_t read_temp();
uint16_t apply_pwm();
uint16_t stop_pwm();
