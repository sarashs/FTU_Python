/*
 * MCU_code.c
 *
 * Created: 7/8/2020 12:21:33 PM
 * Author : hp
 */ 


#include "sam.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>


#define LED1 PORT_PB30;
#define LED0 PORT_PA06;
#define TIMER4_FREQUENCY 1000000; //1MHz
#define TIMER4_PRESCALAR 256;

void init_TC4(void);
void init_TC5(void);
void clock_setup(void);
float volt_to_temperature(float);
int twos_complement_to_int(int, int);
int ADC_mv(int, int);
int magnetic_field_mT(int);
int transition_system_FSM(bool, bool, int);
unsigned int test_time_left(clock_t,int);
//
void run_system_FSM(int);
void ADC_array_convert(void);

//volatile is for variables that can always change in the code
volatile int timer4_compare_value = 3905;  //compare value for 1sec default rate 
volatile int raw_ADC_data [29];       //stores raw ADC values
volatile int converted_ADC_data [29]; //stores converted ADC values
volatile int test_time_count;
volatile bool TEST_START = false;
volatile bool TEST_STOP = true;

//Input data


int main(void)
{
	/* Replace with your application code */
	SystemInit(); //Initialize system
	clock_setup(); //Sets up the clocks
	init_TC4(); // Enables TC4 timer with the predefined interrupts
	
	//Configure LED0 and LED 1 as OUTPUTS
	REG_PORT_DIRSET1 = LED0;
	
	while (1)
	{
		
	}
	return 0;
}
void clock_setup(){ //setting up the clock speeds
	// Set up the generic clock (GCLK4) used to clock timers
	REG_GCLK_GENDIV = GCLK_GENDIV_DIV(48) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
	GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

	REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
	GCLK_GENCTRL_GENEN |         // Enable GCLK4
	GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
	GCLK_GENCTRL_ID(4);          // Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
}

void init_TC4(){ //initialize TC4 timer
	// Feed GCLK4 to TC4 and TC5
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
	GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
	GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
	//GCLK_CLKCTRL_ID_TCC2_TC3; this can feed it to TC3 as well
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	// Configure TC4 (16 bit counter by default)
	
	
	REG_TC4_COUNT16_CC0 = 0x1;                      // Set the TC4 CC0 register to  , decimal 1952(0x7A0) -> 0.5 sec
	                                                // This is the rate of reading the ADC
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	REG_TC4_COUNT16_CC1 = 0xF41;                      // Set the TC4 CC1 register to  decimal 3905, 1 sec
													  // This is the rate at which serial data is sent 
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	//Enabling interrupts
	NVIC_SetPriority(TC4_IRQn, 1);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
	NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

	REG_TC4_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 ;        // Clear the interrupt flags
	REG_TC4_INTENSET = TC_INTENSET_MC1 | TC_INTENSET_MC0 ;     // Enable TC4 Match CC0 and CC1 interrupts
	// REG_TC4_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 ;     // Disable TC4 interrupts

	REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV256 |     // Set prescaler to 256,
	TC_CTRLA_ENABLE;               // Enable TC4
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
}

void init_TC5(){ //initialize TC3 timer
	//Clock is already initialized by code from TC4
	
	// Configure TC4 (16 bit counter by default)
	
	
	//REG_TC5_COUNT16_CC0 = 0x7A0;                      // Set the TC4 CC0 register to  decimal 1952, 0.5 sec
	//while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	REG_TC5_COUNT16_CC1 = 0xF41;                      // Set the TC4 CC1 register to  decimal 3905, 1 sec
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	//Enabling interrupts
	NVIC_SetPriority(TC5_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
	NVIC_EnableIRQ(TC5_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

	REG_TC5_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 ;        // Clear the interrupt flags
	REG_TC5_INTENSET = TC_INTENSET_MC1 | TC_INTENSET_MC0 ;     // Enable TC4 Match CC0 and CC1 interrupts
	// REG_TC5_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 ;     // Disable TC4 interrupts

	REG_TC5_CTRLA |= TC_CTRLA_PRESCALER_DIV256 |     // Set prescaler to 256,
	TC_CTRLA_ENABLE;               // Enable TC4
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
}

void TC4_Handler()                              // Interrupt Service Routine (ISR) for timer TC4
{
	// Check for match counter 0 (MC0) interrupt
	if (TC4->COUNT16.INTFLAG.bit.MC0 && TC4->COUNT16.INTENSET.bit.MC0)
	{
		// compare 0 (CC0) code here:
		
		REG_TC4_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag
	}

	// Check for match counter 1 (MC1) interrupt
	if (TC4->COUNT16.INTFLAG.bit.MC1 && TC4->COUNT16.INTENSET.bit.MC1)
	{
		//compare 1 (CC1) code here:
		REG_PORT_OUTTGL1 = LED0; //toggle LED
		
		REG_TC4_INTFLAG = TC_INTFLAG_MC1;        // Clear the MC1 interrupt flag
	}
	
}

void TC5_Handler(){
	// Check for match counter 1 (MC1) interrupt
	if (TC4->COUNT16.INTFLAG.bit.MC1 && TC4->COUNT16.INTENSET.bit.MC1)
	{
		//compare 1 (CC1) code here:
		test_time_count++; //increament +1 every second
		
		REG_TC4_INTFLAG = TC_INTFLAG_MC1;        // Clear the MC1 interrupt flag
	}
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
	if (num_bits <= 2 || num_bits == NULL || value > (pow(2,num_bits)-1)) return NULL; //error checking

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
	#define ERROR 1.5							//1.5% ERROR
	#define VOLTAGE_CLAMP_HIGH 2970			    //2.97V +40 mT
	#define VOLTAGE_CLAMP_LOW 0.33			    //2.97V -40mT
	#define MAX_MAGNETIC_FIELD 40              //40mT
	#define MIN_MAGNETIC_FIELD -40              //-40mT
	
	if (voltage == NULL) return NULL;
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
	#define SERIAL_UPDATE 2 //sending data back to the computer through serial at intervals
	#define Heater_Update 3 //running the heater_FSM and get the updates
	#define Magnetic_Field_update 4 //running the magnetic field FSM and get the updates
	
	//Inputs from the MCU
	int desired_temperature = 0;     //in C
	int desired_magnetic_field = 0;  //in mT
	int desired_time_for_test = 0;	 //time in minutes
	int desired_FPGA_voltage =0;     //in mV
	int serial_output_rate=1000;                 //rate to read data in ms
	
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
					timer4_compare_value = ((TIMER4_FREQUENCY*serial_output_rate)/(TIMER4_PRESCALAR*1000) )- 1; //sets the timer compare value
					
					REG_TC4_COUNT16_CC1 = timer4_compare_value;                      // Set the TC4 CC1 register, default decimal 3905, 1 sec
					while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
				
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
	 for (int i =0, i <= 23, i++)
	 {
		 converted_ADC_data[i] = ADC_mv(raw_ADC_data[i]); //converting all the voltages to millivolts
	 }
	//for OFFSET conversion
	converted_ADC_data[24] = raw_ADC_data[24];
	//for VCC conversion
	converted_ADC_data[25] = ((float) raw_ADC_data[25]/(float) 3072);
	//for ADC temp conversion
	converted_ADC_data[26] =( ((float) (1000*ADC_mv( raw_ADC_data[26]) - 168000)/(float) ADC_temp_sensor_coefficient)+ 25);
	//ADC GAIN conversion
	converted_ADC_data[27] = ((float) raw_ADC_data[27]/(float) 30720);
	//ADC REF conversion
	converted_ADC_data[28] = ((float) raw_ADC_data[28]/(float) 3072);
	
return;
}

