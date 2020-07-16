#include <atmel_start.h>
#include <tcc_lite.h>
#include <hri_tcc_d21.h>

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>

/************************************************************************/
/* PINS DESCRIPTIONS                                                    */
/************************************************************************/

/*
#define TCC1_out GPIO(GPIO_PORTA, 3)
#define HEATER_PWM GPIO(GPIO_PORTA, 4)
#define HELMHOLTZ_PWM GPIO(GPIO_PORTA, 5)
#define Q11_FREQ_COUNTER GPIO(GPIO_PORTA, 8)
#define Q12_FREQ_COUNTER GPIO(GPIO_PORTA, 9)
#define START_ADC GPIO(GPIO_PORTA, 10)
#define _DRDY_ADC GPIO(GPIO_PORTA, 11)
#define MOSI_ADC GPIO(GPIO_PORTA, 16)
#define SCK_ADC GPIO(GPIO_PORTA, 17)
#define MISO_ADC GPIO(GPIO_PORTA, 19)
#define _CS_ADC GPIO(GPIO_PORTA, 21)
#define _RESET_ADC GPIO(GPIO_PORTA, 22)
#define _PWDN_ADC GPIO(GPIO_PORTA, 23)
#define RED_LED GPIO(GPIO_PORTB, 10)
#define BLUE_LED GPIO(GPIO_PORTB, 11)
#define Q14_FREQ_COUNTER GPIO(GPIO_PORTB, 22)
#define Q13_FREQ_COUNTER GPIO(GPIO_PORTB, 23)
*/


//hri_tcc_write_CC_reg(TCC1, 0, 0xf41); /* Channel 0 Compare/Capture Value: 0xf41 */ //how to change serial rate
#define TCC1_FREQUENCY_VAL 1000000 //1MHz
#define TCC1_PRESCALAR_VAL 256

float volt_to_temperature(float);
int twos_complement_to_int(int, int);
int ADC_mv(int, int);
int magnetic_field_mT(int);
//system FSM
int transition_system_FSM(bool, bool, int);
void run_system_FSM(int);
void ADC_array_convert(void);
//magnetic FSM
void magnetic_fsm_run(void);
void magnetic_fsm_transition(void);


//volatile is for variables that can always change in the code
volatile int TCC1_compare_value = 3905;  //compare value for 1sec default rate

volatile int raw_ADC_data [29];       //stores raw ADC values
volatile int converted_ADC_data [29]; //stores converted ADC values
volatile int ADC_GAIN = 1;

volatile int test_time_count = 0;     //seconds of test time elapsed
volatile bool TEST_START = false;
volatile bool TEST_STOP = true;

//Input data from the MCU
volatile int desired_temperature = 0;     //in C
volatile int desired_magnetic_field = 0;  //in mT
volatile int desired_time_for_test = 0;	 //time in minutes
volatile int desired_FPGA_voltage =0;     //in mV
volatile int serial_output_rate=1000;                 //rate to read data in ms

//Variables for Magnetic field FSM
volatile int measured_magnetic_field = 0; // in mT
volatile int magnetic_PWM_duty = 0; //duty cycle value 0-255
volatile char magnetic_fsm_state = 0; //idle state



int main(void)
{
	/* Initializes MCU, drivers and middle ware */
	atmel_start_init();
	
	
	/* Configure/Enable Interrupts */
	NVIC_SetPriority(TCC1_IRQn, 2);											
	NVIC_EnableIRQ(TCC1_IRQn);
	
	NVIC_SetPriority(TCC2_IRQn, 1);
	NVIC_EnableIRQ(TCC2_IRQn);
	
	NVIC_SetPriority(WDT_IRQn, 0);
	NVIC_EnableIRQ(WDT_IRQn);													
	
	/* Enable all IRQs */
	__enable_irq();

	/* Replace with your application code */
	while (1) {
	}
}

void TCC1_Handler(void){
	//serial rate CC0
	//write code to send serial data
	if ( hri_tcc_get_interrupt_MC0_bit(TCC1) )
	{
		//1. Increase test time by 1s
		test_time_count++;
		
		//2. Clear interrupt bit
		hri_tcc_clear_interrupt_MC0_bit(TCC1);
		
	}
	
}

void TCC2_Handler(void){
	//CCO 1 sec counter
	if ( hri_tcc_get_interrupt_MC0_bit(TCC2) )
	{
		//1. Increase test time by 1s
		test_time_count++;
		
		//2. Clear interrupt bit
		hri_tcc_clear_interrupt_MC0_bit(TCC2);
		
	}
	
	//CC1 ADC rate
	if ( hri_tcc_get_interrupt_MC1_bit(TCC2) )
	{
		// 1. Read from ADC
		
		//2. Clear interrupt bit
		hri_tcc_clear_interrupt_MC1_bit(TCC2);
		
	}
	
}

void WDT_Handler(void){
	//early interrupt for WDT, maybe send a message to serial that system is about to reset and resend test configurations
	//we can discuss on this
	
}

/*
Function :volt_to_temperature
Input: Digital Voltage from ADC in millivolts
Output: Temperature conversion in C from TMP235
Formula : T = (V_out - V_offset)/Tc) + T_INFL
Data sheet: https://www.ti.com/lit/ds/symlink/tmp235.pdf?ts=1594142817615&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMP235
*/
float volt_to_temperature(float milli_volts)
{
	#define V_OFFSET_0 500
	#define V_OFFSET_100 1500
	#define V_OFFSET_125 1752.5

	#define T_INFL_0 0
	#define T_INFL_100 100
	#define T_INFL_125 125

	#define T_COEFF_0 10
	#define T_COEFF_100 10.1
	#define T_COEFF_125 10.6

	if (milli_volts < V_OFFSET_100) return (((milli_volts - V_OFFSET_0)/T_COEFF_0) + T_INFL_0); // -40 to 100C
	
	else if (milli_volts >= V_OFFSET_100 && milli_volts <= V_OFFSET_125) return (((milli_volts - V_OFFSET_100)/T_COEFF_100) + T_INFL_100); // 100 to 125C
	
	else return (((milli_volts - V_OFFSET_125)/T_COEFF_125) + T_INFL_125); // 125 to 150C
}

/*
Function :twos_complement_to_int
Purpose:Returns an integer value of a twos complement number
Input: value and number of bits
Output: integer
*/

int twos_complement_to_int (int value, int num_bits)
{
	// msb =  value >> (num_bits-1); //collecting MSB
	if (num_bits <= 2 || num_bits == '\0' || value > (pow(2,num_bits)-1)) return '\0'; //error checking

	else if (value >> (num_bits-1)){	//if msb ==1, negative values
		return (( value & (int)(pow(2,num_bits-1)-1)) - (pow(2,num_bits-1)));
	}

	else {	//positive values
		return value;
	}
}

/*
Function: ADC_mV
Purpose: converts ADV value into millivolts
Input: int reading, int gain
Return: int millivolts
mV = ADC_data * range
	 ----------------
	 Gain * ADC_Resolution
Source: http://scientific-solutions.com/products/faq/ssi_faq_adc_equations%20VER2.shtml#:~:text=2%27s%20Complement%20ADC%20Data%20with%208%2Dbit%20resolution
*/

int ADC_mv(int ADC_reading, int adc_gain){
	#define MAX_POSITIVE_VOLTAGE 0x7FFF
	#define MAX_NEGATIVE_VOLTAGE 0x8000
	#define REFERENCE_VOLTAGE_ADC_mV 65536
	#define ADC_RESOLUTION 32768 //2^15 bits as 16th bit is used for 2's complement
	#define ADC_RANGE 4900 //4900mV (-1.6V to 3.3V)
	
	return (ADC_RANGE*ADC_reading)/(adc_gain*ADC_RESOLUTION);
}

/*
Function: magnetic_field_mT
Purpose: Converts voltage to corresponding magnetic field (reference is for the  magnetic sensor)
Input: Voltage in millivolts
Output: Magnetic field density in milli_Tesla (1G = 0.1mT)
Data sheet: https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjxps3qg8PqAhXBo54KHUn5CvwQFjAAegQIBRAB&url=https%3A%2F%2Fwww.allegromicro.com%2F~%2Fmedia%2FFiles%2FDatasheets%2FA1318-A1319-Datasheet.ashx&usg=AOvVaw39zGCju7QuDLgpcH9PKde_
*/

int magnetic_field_mT(int voltage){
	#define QUIESCENT_VOLTAGE 1650              //1.65V Low->1.638, mean-> 1.65, high-> 1.662 measure this
	#define MAGNETIC_SENSITIVITY 1.35           //1.35mv/G --> 13.5mV/mT Low->1.289, mean-> 1.3, high-> 1.411
	#define TEMPERATURE_SENSITIVITY   0.12      //0.12%/C
	#define magnetic_sesnor_ERROR 1.5							//1.5% ERROR
	#define VOLTAGE_CLAMP_HIGH 2970			    //2.97V +40 mT
	#define VOLTAGE_CLAMP_LOW 0.33			    //2.97V -40mT
	#define MAX_MAGNETIC_FIELD 40              //40mT
	#define MIN_MAGNETIC_FIELD -40              //-40mT
	
	if (voltage == '\0') return '\0';
	else if (voltage == VOLTAGE_CLAMP_HIGH ) return MAX_MAGNETIC_FIELD ; //we can find some sort of error to throw here
	else if (voltage == VOLTAGE_CLAMP_LOW) return MIN_MAGNETIC_FIELD ;
	else return ((voltage-QUIESCENT_VOLTAGE)/MAGNETIC_SENSITIVITY);   //polarity depends on the direction of the field
	
}

/*
Function: System_FSM
input : Void
Output : Void
Purpose : This state_machine runs the system

*/

//STATES {IDLE, SPI_UPDATE, serial_update, Heater_update, Magnetic_field_update}
	
void run_system_FSM (int state){
	
	#define IDLE 0 //idle state
	#define ADC_UPDATE 1 //reading data from the ADC using SPI at intervals
	#define SERIAL_UPDATE 2//sending data back to the computer through serial at intervals
	#define Heater_Update 3 //running the heater_FSM and get the updates
	#define Magnetic_Field_update 4 //running the magnetic field FSM and get the updates
	
	
	
	//Magnetic FSM variables
	int measured_magnetic_field; // in mT
	int magnetic_FSM_duty=0; //0-255
	
	//variables
	int current_test_time = 0; //test time in minutes
	
	
	switch (state){
		case IDLE : {
				/*If (START) {
				- Set Voltage
				- Set total time for test
				- Start signal
				- Set time interval for reading data
				- Set desired Magnetic field value (Voltage) in mT
				- Set desired temperature (Has to be converted to voltage) in C
				- Set Threshold voltage, 
				- Start timer 
				*/
				if (TEST_START && !TEST_STOP){
					//1. secs/60 
					current_test_time = test_time_count/60; 
					
					
					//2. Setting timer compare value from data rate
					TCC1_compare_value = ((TCC1_FREQUENCY_VAL*serial_output_rate)/(TCC1_PRESCALAR_VAL*1000) )- 1; //sets the timer compare value
					
					hri_tcc_write_CC_reg(TCC1, 0, TCC1_compare_value);
					
					//REG_TC4_COUNT16_CC1 = TCC1_compare_value;                      // Set the TC4 CC1 register, default decimal 3905, 1 sec
					//while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
				
				}
				else if (TEST_STOP){
				//clear and turn off everything
				//turn off heater click
				//turn off magnetic field
				}
			}
			break;
		case ADC_UPDATE: {
			    /*
				- Conversion is implicitly done
				- Convert values from the ADC and store them into their various locations
				- How to read from ADC?
				*/
				//1. Update test timer and if time is hit, stop
				if (current_test_time >= desired_time_for_test) TEST_STOP=true;
			
				//2. Convert Raw ADC data to understandable values
				ADC_array_convert();

			
		}
			break;
		case SERIAL_UPDATE:{
			//1. Set Data to be sent to the user from the ADC update, data is sent in intervals in an Interrupt service routine
			
			//out current_test_timer, ADC converted data at intervals
			
		}
			break;
		case Heater_Update:{
			//update actual and desired temperature, run heater click FSM
		}
			break;
		case Magnetic_Field_update:{
			//update actual and desired magnetic field, fun magnetic field FSM
			//1. state_transtions
			magnetic_fsm_transition();
			//2.state_run
			magnetic_fsm_run();
			
			
		}
			break;
		default :state = IDLE;
	}
	return;
}

int transition_system_FSM(bool fsm_start, bool fsm_stop, int state)
{
	#define IDLE 0 //idle state
	#define ADC_UPDATE 1 //reading data from the ADC using SPI at intervals
	#define SERIAL_UPDATE 2 //sending data back to the computer through serial at intervals
	#define Heater_Update 3 //running the heater_FSM and get the updates
	#define Magnetic_Field_update 4 //running the magnetic field FSM and get the updates

	switch (state){
		case IDLE : {
			if (fsm_stop) state = IDLE;	//else state = idle
			else if (fsm_start) state =ADC_UPDATE;
		}
		break;
		case ADC_UPDATE: {
			if (fsm_stop) state = IDLE;
			else state = SERIAL_UPDATE;
		}
		break;
		case SERIAL_UPDATE:{
			if (fsm_stop) state = IDLE;
			else state = Heater_Update;

		}
		break;
		case Heater_Update:{
			if (fsm_stop) state = IDLE;
			else state = Magnetic_Field_update;
		}
		break;
		case Magnetic_Field_update:{
			if (fsm_stop) state = IDLE;
			else state = ADC_UPDATE;
		}
		break;
		default :state = IDLE;
	}
	return state;
}

/*
Function : ADC_array_convert
Purpose : Converts raw_ADC_Data into converted_ADC_data
Input: void
Output: void

*/
void ADC_array_convert(void){
	//0:23 is all in voltage
	//24 OFFSET, 25-> ADC VCC in mV,26-> ADC TEMP in C, 27-> GAIN V/V, 28-> REF
	#define ADC_temp_sensor_coefficient 563 //563uV if ADS1158 and test PCB temperatures are forced together, 
											//394uV if ADS1158 temperature is forced and test PCB is in free air
	 for (int i =0; i <= 23; i++)
	 {
		 converted_ADC_data[i] = ADC_mv(raw_ADC_data[i], ADC_GAIN); //converting all the voltages to millivolts
	 }
	//for OFFSET conversion
	converted_ADC_data[24] = raw_ADC_data[24];
	//for VCC conversion
	converted_ADC_data[25] = ((float) raw_ADC_data[25]/(float) 3072);
	//for ADC temp conversion
	converted_ADC_data[26] =( ((float) (1000*ADC_mv( raw_ADC_data[26], ADC_GAIN) - 168000)/(float) ADC_temp_sensor_coefficient)+ 25);
	//ADC GAIN conversion
	converted_ADC_data[27] = ((float) raw_ADC_data[27]/(float) 30720);
	//ADC REF conversion
	converted_ADC_data[28] = ((float) raw_ADC_data[28]/(float) 3072);
	
return;
}

/************************************************************************/
/* MAGNETIC FIELD FSM                                                   */
/************************************************************************/
/*	This is a state machine for the Magnetic circuit program--
	An external function is needed to keep updating the actual and desired Magnetic field values
*/
#define magnetic_fsm_idle_state 0
#define magnetic_fsm_increase_state 1
#define magnetic_fsm_decrease_state 2
#define magnetic_error 0.015 //error is 1.5% gotten from the magnetic field data sheet

void magnetic_fsm_transition(void)
{
	if (magnetic_fsm_state == magnetic_fsm_idle_state) 
	{
		if (measured_magnetic_field < (desired_magnetic_field*(1-magnetic_error)) ) magnetic_fsm_state = magnetic_fsm_increase_state;
		else if (measured_magnetic_field > (desired_magnetic_field*(1 + magnetic_error)) ) magnetic_fsm_state = magnetic_fsm_decrease_state; //else if equal, state is unchanged
	}
	else if (magnetic_fsm_state == magnetic_fsm_increase_state) 
	{
		if (measured_magnetic_field >= desired_magnetic_field) magnetic_fsm_state = magnetic_fsm_idle_state; //else state is unchanged
	}
	else if (magnetic_fsm_state == magnetic_fsm_decrease_state) 
	{
		if (measured_magnetic_field <= desired_magnetic_field) magnetic_fsm_state = magnetic_fsm_idle_state; //else state is unchanged
	}
}
void magnetic_fsm_run(void)
{
	if (magnetic_fsm_state == magnetic_fsm_idle_state){	} //no change
	else if (magnetic_fsm_state == magnetic_fsm_increase_state) 
	{
		//PWM is increased until desired state
		if (magnetic_PWM_duty >= 255) magnetic_PWM_duty=255;
		else magnetic_PWM_duty++;
	}
	else if (magnetic_fsm_state == magnetic_fsm_decrease_state) 
	{
		//PWM is decreased until desired state
		if (magnetic_PWM_duty <= 0) magnetic_PWM_duty=0;
		else magnetic_PWM_duty--;
	}
	//analogWrite(MG_pin, MG_duty); //write to the pin after changing the duty
}
