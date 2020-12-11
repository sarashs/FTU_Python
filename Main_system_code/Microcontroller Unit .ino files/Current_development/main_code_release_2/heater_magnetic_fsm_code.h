#pragma once

#define analog_resolution 1023

/************************************************************************/
/* HEATER CLICK FSM                                                     */
/************************************************************************/
/*	This is a state machine for the Magnetic circuit program--
	An external function is needed to keep updating the actual and desired Magnetic field values
*/
	#define heater_fsm_OFF 0
	#define heater_fsm_idle 1 //state where temperature is maintained
	#define heater_fsm_heating 2
	#define heater_fsm_cooling 3
	#define heater_fsm_margin 2.5 //error is +/- 2.5C as we are using integers
	#define heater_threshold_temp 50 // above 40C is hot for human touch

/**
 * \@brief Function updates the heater FSM states
 * 
 * \@param current state
 * 
 * \@return int next_state
 */
int heater_fsm_transition(int current_state, int measured_temperature, int desired_temperature)
{	
	int next_state;
	float temperature_difference =  (float)(measured_temperature - desired_temperature);
		
	switch(current_state){
		case heater_fsm_OFF: {
			next_state = heater_fsm_idle;
			break;
		}
		case heater_fsm_cooling:
		case heater_fsm_heating:
		case heater_fsm_idle: {
			if ( temperature_difference > heater_fsm_margin) next_state = heater_fsm_cooling; //cool temp is above desired + threshold
			else if ( abs(temperature_difference) <= heater_fsm_margin) next_state = heater_fsm_idle; //maintain heat if it between 0.5C threshold
			else next_state = heater_fsm_heating; //heat up if is below the desired+threshold
				
			break;
		}

		default: next_state = heater_fsm_OFF;
	}
	return next_state;
}

/**
 * \@brief Function runs the heating system
 * 
 * \@param int heater_state
 * \@param int pwm_duty_cycle 
 * 
 * \@return int pwm_duty_cycle
 */
int heater_fsm_run(int heater_state, int pwm_duty_cycle, int measured_temperature ){
	if (measured_temperature < heater_threshold_temp && digitalRead(blue_led)!=HIGH) digitalWrite(blue_led,HIGH);//turn on safe to touch LED if not high already
	else digitalWrite(blue_led,LOW);  //turn off safe to touch LED
	
	switch(heater_state){
		case heater_fsm_OFF: {
			//all pins off
			digitalWrite(red_led, LOW); //turn of heating signal
			pwm_duty_cycle = 0;
			break;
		}
		case heater_fsm_idle: {
			digitalWrite(red_led, LOW); 
			break;
		}
		case heater_fsm_heating: {
			digitalWrite(red_led,HIGH); //turn on heating signal
			if (pwm_duty_cycle >= analog_resolution) pwm_duty_cycle=analog_resolution;
			else pwm_duty_cycle++;	
			break;
		}
		case heater_fsm_cooling: {
			digitalWrite(red_led, LOW); //turn of heating signal
			if (pwm_duty_cycle <= 0) pwm_duty_cycle=0;
			else pwm_duty_cycle--;	
			break;
		}
		default: pwm_duty_cycle = 0;
	}
	
	analogWrite(heater_pwm,pwm_duty_cycle);
	return pwm_duty_cycle;
}


/************************************************************************/
/* MAGNETIC FIELD FSM                                                   */
/************************************************************************/

#define magnetic_fsm_idle_state 0
#define magnetic_fsm_increase_state 1
#define magnetic_fsm_decrease_state 2
#define magnetic_error 0.015 //error is 1.5% gotten from the magnetic field data sheet in mT

/**
 * \@brief  This function updates the states for the state machine of the Magnetic circuit program--
 *		   An external function is needed to keep updating the actual and desired Magnetic field values
 * 
 * \@param current_state
 * 
 * \@return int next state
 */
int magnetic_fsm_transition(int current_state, float measured_magnetic_field, float desired_magnetic_field)
{
	float magnetic_field_difference = (float) (measured_magnetic_field - desired_magnetic_field); //getting the error
	int next_state;
	
	 switch(current_state){
		 case magnetic_fsm_idle_state:
		 case magnetic_fsm_increase_state:
		 case magnetic_fsm_decrease_state: {
			 if ( magnetic_field_difference > magnetic_error) next_state = magnetic_fsm_decrease_state; //reduce PWM duty if above desired + threshold
			 else if ( abs(magnetic_field_difference) <= magnetic_error) next_state = magnetic_fsm_idle_state; //maintain PWM if between threshold
			 else next_state = magnetic_fsm_increase_state; //PWM duty increase if below the desired+threshold
		 }
		default: next_state = magnetic_fsm_idle_state; 
	 }
	
	return next_state;
}

/**
 * \@brief Function runs the magnetic Helmholtz coil
 * 
 * \@param current_state
 * \@param pwm_duty
 * 
 * \@return int magnetic_pwm_duty
 */
int magnetic_fsm_run(int current_state, int pwm_duty)
{
	if (current_state == magnetic_fsm_idle_state){	} //no change
	else if (current_state == magnetic_fsm_increase_state) 
	{
		//PWM is increased until desired state
		if (pwm_duty >= analog_resolution) pwm_duty=analog_resolution;
		else pwm_duty++;
	}
	else if (current_state == magnetic_fsm_decrease_state) 
	{
		//PWM is decreased until desired state
		if (pwm_duty <= 0) pwm_duty=0;
		else pwm_duty--;
	}
	analogWrite(helmholtz_pwm, pwm_duty); //write to the pin after changing the duty
	
	return pwm_duty;
}
