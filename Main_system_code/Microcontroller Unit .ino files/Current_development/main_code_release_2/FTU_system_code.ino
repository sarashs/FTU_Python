/************************************************************************/
/*                    FILE DESCRIPTION                                  */
/************************************************************************/

/*H**********************************************************************
* \@filename :        
* FTU_system_code.ino            
*
* \@description :
* This file controls a SAMD21G18A MCU on Arduino MKR1000
* Used to run a HTOL Test (High-temperature operating life) for Accelerated Life Testing
*
* \@notes : 
* This code connects to a python script
*
*
* \@authors :    
* 1. Valentine Ssebuyungo        
*
* \@start_date : 1st June 2020
*
*
* \@documentation_style :	 
* 1. "https://developer.lsst.io/cpp/api-docs.html#cpp-doxygen-see-also"
* 2. "http://www.edparrish.net/common/cppdoc.html#functioncomment"
*
* \@future_improvements :
*Add a header file to contain all data
*IoT communication
*RTC timer for time
*PID control for magnetic and heater system
*Emergency stop
*WDT
*Reset Arduino after test stops
*Frequency divider functionality
*
*H*/

#include "global_variables_and_constants.h"
#include "pins_setup.h"
#include "adc_code.h"
#include "serial_communication_code.h"
#include "timers_and_interrupt_setup.h"
#include "DAC_code.h"
#include "heater_magnetic_fsm_code.h"
#include "memory_functions.h"

//testing
int count = 0;
unsigned long start_time = 0;
unsigned long end_time = 0;
unsigned long delta_time = 0;
/**
 * This setup function is run before a test starts and sets up the system
 * 
 * @param void
 * @return void
 */
void setup() {
  analogWriteResolution(10);
  analogReadResolution(10);
  analogReference(AR_INTERNAL2V23); //sets 3.3V reference
  pin_setup(); //Setup MCU pins
  delay(1000);
  
  if (high_speed_test)
  {
	  //set the ADC DRATE to 11 for fastest reading
	 adc_register_defaults[2] |= 0x03; //this sets the ADC DRATE==11 for the highest sampling rate
	 memory_addresses_linkedlist.Clear();
	 linkedlist_of_mem_addresses_of_other_linkedlists.Clear();
	 flash.setClock(adc_spi_speed);
  }
  adc_setup(); //function sets up the ADC
  memory_setup(&flash);
  
  
  Serial.begin(BAUD_RATE); 
  while (!Serial) continue;//if not connected stall test until connected
  receive_test_instructions(); //run function for handshaking to receive instructions
  Serial.end();

  clock_setup();           //set up 1MHz clock for the timers
  init_tc3();            //set up TC3 whose interrupt sends serial data
  init_tc4();              //initialize TC4 timer, TC4 generates the ADC read interrupt
  init_tc5();              //TC5 is a 1 second counter
  __enable_irq();          //enable interrupts
  test_time_count = 0; //reset test time

}

/**
 * This function runs for the whole test
 * 
 * @param void
 * @return void
 */
void loop() {

	 system_state = system_fsm_transition(system_state,test_start,test_stop);
	 system_fsm_run(system_state);
	 
	 //testing
	 if (!test_stop)
	 {
		 	 Serial.print("Count : ");
		 	 Serial.println(count);
	 }

	 count++;


}


/************************************************************************/
/* INITIALIZATION FUNCTIONS                                             */
/************************************************************************/

/**
 * \@brief This state_machine runs the system
 * 
 * \@param 
 * 
 * \@return void
 */
void system_fsm_run (int system_fsm_state){
	//initialize variables	
	switch (system_fsm_state){
		case IDLE : {
			if (test_stop){ //Test has ended so this is where it is handled
				//clear and turn off all the outputs
				pin_setup(); //this will reset all the pins to their original values;
				
				
				//if high speed test then dump all the data from memory to Serial Port
				if (high_speed_test)
				{
					//send all the current test memory data to serial Port
					//Serial.println("Dumping memory to serial");
					print_all_arrays_in_memory(&flash,ADC_ARRAY_SIZE);
					
					high_speed_test = false; //Turn off high speed test stop loops
										
				}
				
				
				update_json_doc(test_id,test_stop,test_start,test_error,error_message,test_time_count,measured_temperature,measured_magnetic_field,converted_adc_data);
				
				if (serial_signal) {
					send_data_to_serial();
					serial_signal = false;
					
					//Reset device after communicating with Arduino
					//NVIC_SystemReset();			
				}
				
				
			}
			else if (test_start){
				//1. Setting voltage stress for test
				analogWrite(ctrl_vstr,set_dac(desired_fpga_voltage));
				//reset test time
				test_time_count = 0;

			
			}
			
		}
		break;
		case ADC_UPDATE: {

			//1. Update test timer and if time is hit, stop
			//Serial.println("Enter ADC Update");
			if (test_time_count >= desired_time_for_test) test_stop=true;
			
			pin_setup();
			//2. Convert Raw ADC data to understandable values
			Serial.println("Auto scan time (us): ");
			start_time = micros();
			adc_auto_scan(raw_adc_data); //converting ADC data
			end_time = micros();
			delta_time = end_time - start_time;
			Serial.println(delta_time);
			
			Serial.println("converting adc array time (us): ");
			start_time = micros();
			adc_array_convert(raw_adc_data,converted_adc_data); //converts the raw_adc_data into converted data
			
			end_time = micros();
			delta_time = end_time - start_time;
			Serial.println(delta_time);
			//Serial.println("Exit ADC_update");
			
		}
		break;
		case SERIAL_UPDATE:{
			//1. Set Data to be sent to the user from the ADC update, data is sent in intervals in an Interrupt service routine
			
			
			if (high_speed_test) //if high speed test, save array to memory and 
			{
				//save adc array to memory
 				Serial.println("Saving time (us): ");
				start_time = micros(); 
				save_array_in_memory(&flash,converted_adc_data,ADC_ARRAY_SIZE); //saves array in memory
				end_time = micros();
				delta_time = end_time - start_time;
				Serial.println(delta_time);
				//Serial.println("Done saving");
			}
			else if (serial_signal) { //if it is not a high speed test, send the data to serial
				update_json_doc(test_id,test_stop,test_start,test_error,error_message,test_time_count,measured_temperature,measured_magnetic_field,converted_adc_data);
				send_data_to_serial(); //function to send json packet to serial port
				serial_signal = false; //turn off the serial_signal flag
			}
			
		}
		break;
		case Heater_Update:{
			//update actual and desired temperature, run heater click FSM
			measured_temperature = millivolt_to_celcius((float) converted_adc_data[9]); //AIN1 in ADC converted DATA
			heater_fsm_state = heater_fsm_transition(heater_fsm_state, measured_temperature, desired_temperature);
			heater_pwm_duty = heater_fsm_run(heater_fsm_state, heater_pwm_duty, measured_temperature);
			
		}
		break;
		case Magnetic_Field_update:{
			//update actual and desired magnetic field, fun magnetic field FSM
			measured_magnetic_field = millivolts_to_milliTesla(converted_adc_data[8]);
			//1.state_run
			magnetic_pwm_duty = magnetic_fsm_run(magnetic_fsm_state,magnetic_pwm_duty);
			//2. update the state
			magnetic_fsm_state = magnetic_fsm_transition(magnetic_fsm_state, measured_magnetic_field, measured_magnetic_field);
			
		}
		break;
		default :system_fsm_state = IDLE;
	}
	return;
}

/**
 * \@brief This function updates states for the System FSM
 * 
 * \@param 
 * 
 * \@return new state
 */
int system_fsm_transition(int current_system_fsm_state, int test_start, int test_stop)
{
	int next_state;
	
	if (test_stop) {
		next_state = IDLE;	//else state = idle
		return next_state;
	}
	
	switch (current_system_fsm_state){
		case IDLE : {
			if (test_start) next_state =ADC_UPDATE;
		}
		break;
		case ADC_UPDATE:{
			next_state = SERIAL_UPDATE;
		}
		break;
		case SERIAL_UPDATE:{
			//testing
			if (high_speed_test)
			{
				next_state = ADC_UPDATE; //test is only 3 seconds max so avoid extra lines of code
			}
			else {
				next_state = Heater_Update;
			}
			
		}
		break;
		case Heater_Update:{
			next_state = Magnetic_Field_update;
		}
		break;
		case Magnetic_Field_update:{
			next_state = ADC_UPDATE;
		}
		break;
		default :next_state = IDLE;
	}
	return next_state;
}

/************************************************************************/
/*                    Memory addition                                   */
/************************************************************************/