/*H**********************************************************************
* FILENAME :        FTU_system_code.ino            DESIGN REF: 
*
* DESCRIPTION :
*       This file controls a SAMD21G18A MCU on Arduino MKR1000   
*		Used to run a HTOL Test (High-temperature operating life) for Accelerated Life Testing   
*
* NOTES : This code connects to a python script
*       
*
* AUTHOR :    Valentine Ssebuyungo        START DATE :    1st June 2020
*
* CHANGES : TC3 timer removed
*
*
* FUTURE IMPROVEMENTS:
*							Add a header file to contain all data
*							IoT communication
*							RTC timer for time
*							PID control for magnetic and heater system
*							Emergency stop
*						    WDT
*							Faster Serial communication - 0.1uS
*							Reset Arduino after test stops
*							Frequency divider functionality
*H*/

#include "driver_init.h"
#include "ArduinoJson.h"
#include "ArduinoJson.hpp"
#include "Arduino.h"
#include "samd21/include/samd21g18a.h"
#include "math.h"
#include "strings.h"
#include "WString.h"
#include "time.h"
#include <stdbool.h>
#include <SPI.h>

/************************************************************************/
/* PINS DESCRIPTIONS                                                    */
/************************************************************************/
#define CTRL_VSTR A0 //A0 the control signal for the voltage stress circuit, labeled CTRL_VSTR in figure 1
#define HEATER_PWM A3 //A3 This is the control signal for the Heater board
#define HELMHOLTZ_PWM A4 //A4 This is the control signal for the Helmholtz circuit
#define _RESET_ADC 0          //D0 Should be configured as a digital output. Connected to *RESET on the ADC
#define _PWDN_ADC 1           //D1 Should be configured as a digital output. Connected to *PWDN on the ADC
#define START_ADC 2           //D2 Should be configured as a digital output. Connected to START on the ADC.
#define _DRDY_ADC 3           //D3 Should be configured as a digital input. Connected to *DRDY on the ADC.
#define RED_LED 4          //D4 Should be configured as a digital output. Control signal for the red LED on the heater board.
#define BLUE_LED 5            //D5 Should be configured as a digital output. Control signal for the Blue LED on the heater board.
#define CLR_FREQ_DIVIDER 6    //D6 Should be configured as a digital output. The clear command for the frequency divider. labeled CLR in figure 1.
#define _CS_ADC 7             //D7 Should be configured as a digital output. This is the *CS line for SPI communication with the ADC
#define MOSI_ADC 8            //D8 Should be configured as a digital output. This is the MOSI line on the SPI bus. labeled DIN on the ADC.
#define SCK_ADC 9             //D9 Should be configured as a digital output. This is the SCLK line on the SPI bus.
#define MISO_ADC 10           //D10 Should be configured as a digital input. This is the MISO line on the SPI bus. labeled DOUT on the ADC.
#define Q11_FREQ_COUNTER 11   //D11 Should be configured as a digital input. Connected to Q11 on the frequency divider
#define Q12_FREQ_COUNTER 12   //D12 Should be configured as a digital input. Connected to Q12 on the frequency divider
#define Q13_FREQ_COUNTER 13   //D13 Should be configured as a digital input. Connected to Q13 on the frequency divider
#define Q14_FREQ_COUNTER 14   //D14 Should be configured as a digital input. Connected to Q14 on the frequency divider
//A1, A2, A5, A6   4 MCU pins that are not used in the design and left disconnected. They will be accessible through the male/female headers mounted on the MCU board.
#define ANALOG_RESOLUTION 1023

/************************************************************************/
/* ADC VARIABLES AND CONSTANTS                                          */
/************************************************************************/
//ADC Register Addresses
#define CONFIG0_address 0x00
#define CONFIG1_address 0x01
#define MUXSCH_address 0x02
#define MUXDIF_address 0x03
#define MUXSG0_address 0x04
#define MUXSG1_address 0x05
#define SYSRED_address 0x06
#define GPIOC_address 0x07
#define GPIOD_address 0x08

//ADC constants
#define NUMBER_OF_BITS_ADC 16
#define ADC_Register_Array_Size 9
#define ADC_SPI_SPEED 6000000  //6MHZ -> Must be less than 0.5 adc_clk(15.7MHz)

uint8_t ADC_Register_Addresses[ADC_Register_Array_Size] = {CONFIG0_address,CONFIG1_address,MUXSCH_address,
MUXDIF_address,MUXSG0_address,MUXSG1_address,SYSRED_address,GPIOC_address,GPIOD_address};

//Default ADC values to be programmed
#define CONFIG0_default 0x02
#define CONFIG1_default 0x00
#define MUXSCH_default 0x00
#define MUXDIF_default 0x00
#define MUXSG0_default 0xFF //0xFF
#define MUXSG1_default 0x7F  //	0x7F
#define SYSRED_default 0xFF  //0x3D if you wanna see measurements
#define GPIOC_default 0x00
#define GPIOD_default 0x00

uint8_t ADC_Register_Defaults[ADC_Register_Array_Size] = {CONFIG0_default,CONFIG1_default,MUXSCH_default,
MUXDIF_default,MUXSG0_default,MUXSG1_default,SYSRED_default,GPIOC_default,GPIOD_default};

uint8_t CONFIG0_value = 0;
uint8_t CONFIG1_value = 0;
uint8_t MUXSCH_value =  0;
uint8_t MUXDIF_value =  0;
uint8_t MUXSG0_value =  0;
uint8_t MUXSG1_value =  0;
uint8_t SYSRED_value =  0;
uint8_t GPIOC_value =   0;
uint8_t GPIOD_value =   0;

volatile uint16_t raw_ADC_data [29] = {0}; //initialize the array with zero
volatile int converted_ADC_data[29] = {0}; //this array hold the converted ADC data have to change this to  double
//initialize some values to avoid errors in calculations

/*How values are stored in these arrays
The Bits CHID is the Array[index] up to index 24,
Index 25 is CHID 26 -> OFFSET
Index 26 is CHID 27 -> Temperature
Index 27 is CHID 28 -> GAIN
Index 28 is CHID 29 -> External reference

From Table 10 in data sheet
BITS CHID[4:0]          PRIORITY        CHANNEL         DESCRIPTION
00h                     1 (highest)     DIFF0 (AIN0–AIN1) Differential 0
01h                     2               DIFF1 (AIN2–AIN3) Differential 1
02h						3				DIFF2 (AIN4–AIN5) Differential 2
03h				        4               DIFF3 (AIN6–AIN7) Differential 3
04h						5			    DIFF4 (AIN8– AIN9) Differential 4
05h						6				DIFF5 (AIN10–AIN11) Differential 5
06h						7				DIFF6 (AIN12–AIN13) Differential 6
07h						8				DIFF7 (AIN14–AIN15) Differential 7
08h						9				AIN0 Single-ended 0
09h						10				AIN1 Single-ended 1
0Ah						11				AIN2 Single-ended 2
0Bh						12				AIN3 Single-ended 3
0Ch						13				AIN4 Single-ended 4
0Dh						14				AIN5 Single-ended 5
0Eh						15				AIN6 Single-ended 6
0Fh						16				AIN7 Single-ended 7
10h						17				AIN8 Single-ended 8
11h						18				AIN9 Single-ended 9
12h						19				AIN10 Single-ended 10
13h						20				AIN11 Single-ended 11
14h						21				AIN12 Single-ended 12
15h						22				AIN13 Single-ended 13
16h						23				AIN14 Single-ended 14
17h						24			    AIN15 Single-ended 15
18h						25				OFFSET Offset
1Ah						26				VCC AVDD – AVSS supplies
1Bh						27				TEMP Temperature
1Ch						28				GAIN Gain
1Dh						29 (lowest)		REF External reference


*/
/************************************************************************/
/* GLOBAL VARIABLES AND CONSTANTS                                       */
/************************************************************************/
//volatile is for variables that can always change in the code
volatile int TCC1_compare_value = 3905;  //compare value for 1sec default rate
volatile int test_time_count = 0;     //seconds of test time elapsed
volatile int System_fsm_state = 0;    //initialize System FSM state

//Input data from the MCU
volatile int Test_ID = 0;
volatile bool TEST_START = false;
volatile bool TEST_STOP = false;
volatile float desired_temperature = 43;     //in C, Set to a default value but actual value comes from MCU instructions
volatile float desired_magnetic_field = 0;  //in mT, Set to a default value but actual value comes from MCU instructions
volatile int desired_time_for_test = 2;	 //time in minutes, Set to a default value but actual value comes from MCU instructions
volatile float desired_FPGA_voltage =0;     //in mV, Set to a default value but actual value comes from MCU instructions
volatile int serial_output_rate=3000;                 //rate to read data in ms, Set to a default value but actual value comes from MCU instructions

//Variables for Magnetic field FSM
volatile int measured_magnetic_field = 0; // in mT
volatile int magnetic_PWM_duty = 0; //duty cycle value 0-255
volatile int magnetic_fsm_state = 0; //idle state

//Variables for heater FSM
volatile float measured_temperature = 0; // in C
volatile float heater_PWM_duty = 0; //duty cycle value 0-255
volatile int heater_fsm_state = 0; //idle state
volatile bool HEATER_START = false;

String message;
String instruction = "instruction";  //This will be a json instruction in string format from the CPU Python script
volatile bool TEST_ERROR = false; 
String error_message = ""; //Contains the error message to be sent to python, this catches errors
volatile float current_test_time = 0; //test time in minutes
volatile bool serial_signal = false;

/************************************************************************/
/* USING JSON LIBRARY TO SEND COMMUNICATE WITH PYTHON SCRIPT            */
/************************************************************************/
//Original source : https://arduinojson.org/
//preparing the JSON document to be used in the test
// Allocate the JSON document
//
// StaticJsonObject allocates memory on the stack, it can be
// replaced by DynamicJsonDocument which allocates in the heap.
//1024 is the RAM dedicated to this document
DynamicJsonDocument  doc(700);

/************************************************************************/
/* TESTING VARIABLES AND CONSTANTS                                      */
/************************************************************************/
#define  REDLED A1
#define  BLUELED A2
#define  YELLOWLED A5
#define  ANALOG_PIN A6

volatile float measured_voltage=0;

void setup() {
	//setting up clocks
	Serial.begin(256000); //9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000
	while (!Serial) continue;//if not connected stall test until connected
	clock_setup();           //set up 1MHz clock for the timers
	//init_TC3();            //set up TC3 whose interrupt sends serial data 
	init_TC4();              //initialize TC4 timer, TC4 generates the ADC read interrupt
	init_TC5();              //TC5 is a 1 second counter
	pin_setup();             //set up pins
	__enable_irq();          //enable interrupts
	ADC_Setup(); //function sets up the ADC
	analogWriteResolution(10); //Sets default to resolution
	analogReference(AR_DEFAULT); //sets 3.3V reference
	receive_test_instructions(); //run function for handshaking to receive instructions
	
// 	pinMode(LED_BUILTIN, OUTPUT);  //testing turn on LED if test starts
// 	if (TEST_START) digitalWrite(LED_BUILTIN,HIGH);
		
}

// the loop function runs over and over again forever
void loop() {
		System_fsm_Run();
		pass();
		System_fsm_Transition();		
}

/**
\Empty function for debugging
\param[in] N/A
\param[out] N/A
*/
void pass (void){
	return;
}

/**
\Function to receive test instructions from CPU
\param[in] N/A
\param[out] N/A
*/
void receive_test_instructions(void){
	while (Serial.available()==0){ //wait for something at serial
	}	
	message = Serial.readStringUntil('\n'); //read till end of the line

	while (Serial.available()==0){ //wait for instruction json at serial
		if (message.equals(instruction)){
			Serial.println("ready");
		}
	}
	
	message = Serial.readStringUntil('\n'); //
	Serial.println(message); //send json back as string to to check if it is correct
	
	//change string to char array for JSON buffer simplicity
	char charBuf[message.length() + 1];
	message.toCharArray(charBuf, message.length()+1);

	// De serialize the JSON document
	DeserializationError error = deserializeJson(doc, charBuf);

	// Test if parsing succeeds.
	if (error) {
		Serial.print(F("deserializeJson() failed: "));
		Serial.println(error.c_str());
		//raise error
		raise_MCU_error(error.c_str());
		return;
	}

	Test_ID = doc["id"]; // 0
	const char* description = doc["description"]; // "Apply required stress for 2 hours, etcetera"

    //Setting test parameters
	JsonObject test_values = doc["test_values"];
	desired_temperature = test_values["temperature"]; // 120
	desired_FPGA_voltage = test_values["v_stress"]; // -400
	desired_time_for_test = test_values["test_time"]; // 5
	desired_magnetic_field = test_values["magnetic_field"]; // 5
	if (test_values["Test_start"] == 1) TEST_START= true; // 1
	if (test_values["Test_stop"]==1) TEST_STOP = true; // 0
	serial_output_rate = test_values["serial_rate"]; // 1500

	JsonObject measurement_params = doc["measurement_params"];

	const char* measurement_params_temperature_unit = measurement_params["temperature"]["unit"]; // "C"
	const char* measurement_params_v_stress_unit = measurement_params["v_stress"]["unit"]; // "mV"
	const char* measurement_params_test_time_unit = measurement_params["test_time"]["unit"]; // "seconds"
	const char* measurement_params_magnetic_field_unit = measurement_params["magnetic_field"]["unit"]; // "mT"
	const char* measurement_params_serial_rate_unit = measurement_params["serial_rate"]["unit"]; // "milliseconds"
	
}

/**
\Function to send data to Python script
\param[in] N/A
\param[out] N/A
*/
void send_data_to_serial(){
	//tell python there is ready data
	//send the data
	//wait for confirmation
   char userInput;	
	if(Serial.available()> 0){
		userInput = Serial.read();               // read user input
		if(userInput == 'g'){                  // if we get expected value		
			//printing the document
			serializeJson(doc, Serial);
			Serial.println();

			while (Serial.available() == 0){
				continue; //wait for confirmation
			}
			while (Serial.read() != 'd'){
				continue;
			}
			Serial.println("done");	
		}
	}
}

/**
\brief Sets up 1MHz clock used by TC3,TC4,TCC2,TC5
\param[in] N/A
\param[out] N/A
*/
void clock_setup(){ //setting up the clock speeds
	
	// Set up the generic clock (GCLK4) used to clock timers
	REG_GCLK_GENDIV = GCLK_GENDIV_DIV(48) |          // Divide the 48MHz clock source by divisor 8: 8MHz/8=1MHz
	GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

	REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
	GCLK_GENCTRL_GENEN |         // Enable GCLK4
	GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
	GCLK_GENCTRL_ID(4);          // Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
	GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
	GCLK_CLKCTRL_ID_TC4_TC5 ; // Feed the GCLK4 to TC4 and TC5  
	while (GCLK->STATUS.bit.SYNCBUSY);	
	
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
	GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
	GCLK_CLKCTRL_ID_TCC2_TC3; // Feed the GCLK4 to TC4 and TC5
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	return;
}

/**
\brief Sets up TC4
\param[in] N/A
\param[out] N/A
*/
void init_TC4(){ //initialize TC4 timer
	// Configure TC4 (16 bit counter by default)
	REG_TC4_COUNT16_CC0 = countervalue(1,256,200);  //set period for timer
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	//Enabling interrupts
	NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)
	
	REG_TC4_INTFLAG |= TC_INTFLAG_OVF; //TC_INTFLAG_MC1 | TC_INTFLAG_MC0 ;        // Clear the interrupt flags (overflow flag)
	REG_TC4_INTENSET |= TC_INTENSET_OVF; //TC_INTENSET_MC1 | TC_INTENSET_MC0 ;     // Enable TC4 counter interrupts
	// REG_TC4_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 ;     // Disable TC4 interrupts

	REG_TC4_CTRLA = TC_CTRLA_WAVEGEN_MFRQ | //this makes CC0 register have the top  value before the overflow interrupt
	TC_CTRLA_MODE_COUNT16 |         //set as a 16 bit counter
	TC_CTRLA_PRESCALER_DIV256 |     // Set prescaler to 256,
	TC_CTRLA_ENABLE;               // Enable TC4
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	return;
}

/**
\brief Sets up TC5
\param[in] N/A
\param[out] N/A
*/
void init_TC5(){ //initialize T5 timer

	REG_TC5_COUNT16_CC0 =  countervalue(1,64,1000);                     // Set the TC4 CC1 register to  decimal 3905, 1 sec
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	//Enabling interrupts
	NVIC_EnableIRQ(TC5_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)
	NVIC_SetPriority(TC5_IRQn,0);

	REG_TC5_INTFLAG |= TC_INTFLAG_OVF;        // Clear the interrupt flags
	REG_TC5_INTENSET |= TC_INTENSET_OVF;     // Enable TC5 Match CC0 and CC1 interrupts
	// REG_TC5_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 ;     // Disable TC5 interrupts

	REG_TC5_CTRLA = TC_CTRLA_WAVEGEN_MFRQ | //this makes CC0 register have the top  value before the overflow interrupt
	TC_CTRLA_MODE_COUNT16 |        //set as a 16 bit counter
	TC_CTRLA_PRESCALER_DIV64 |     // Set prescaler to 256,
	TC_CTRLA_ENABLE;               // Enable TC5
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	return;
}
/**
\brief Sets up TC3
\param[in] N/A
\param[out] N/A
*/
void init_TC3(){ //initialize TC3 timer, this timer controls the rate at which serial data is sent
	
	REG_TC3_COUNT16_CC0 =  countervalue(1,1024,1000);                     // Set the TC3 CC0 register 
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	//Enabling interrupts
	NVIC_EnableIRQ(TC3_IRQn);         // Connect TC3 to Nested Vector Interrupt Controller (NVIC)

	REG_TC3_INTFLAG |= TC_INTFLAG_OVF;        // Clear the interrupt flags
	REG_TC3_INTENSET |= TC_INTENSET_OVF;     // Enable TC3 Match CC0 and CC1 interrupts
	// REG_TC3_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 ;     // Disable TC3 interrupts

	REG_TC3_CTRLA = TC_CTRLA_WAVEGEN_MFRQ | //this makes CC0 register have the top  value before the overflow interrupt
	TC_CTRLA_MODE_COUNT16 |        //set as a 16 bit counter
	TC_CTRLA_PRESCALER_DIV1024 |     // Set prescaler to 1024, with a 1Mhz clock 67 seconds is the slowest we can go
	TC_CTRLA_ENABLE;               // Enable TC3
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	
	return;
}

/**
\Interrupt service routine for TC4, handles the ADC interrupt
\param[in] N/A
\param[out] N/A
*/
void TC4_Handler()// ADC interrupt handler
{
	// Check for OVF interrupt
	if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)
	{
		//Overflow interrupt code here:
		//ADC read code
		
		//testing
		
// 		if (digitalRead(REDLED)==LOW)
// 		{
// 			digitalWrite(REDLED,HIGH);
// 		} 
// 		else
// 		{
// 			digitalWrite(REDLED,LOW);
// 		}	
		// Clear the interrupt flag
		REG_TC4_INTFLAG = TC_INTFLAG_OVF;         
	}
	return;
}

void TC5_Handler(){ //counter interrupt
	// Check for OVF interrupt
	if (TC5->COUNT16.INTFLAG.bit.OVF && TC5->COUNT16.INTENSET.bit.OVF)
	{
		//Overflow interrupt code here:
		
		test_time_count++; //increment +1 every second
		
		if ((test_time_count*1000)%serial_output_rate == 0){
			serial_signal = true;
		}
		
// 		//testing
// 				if (digitalRead(BLUELED)==HIGH)
// 				{
// 					digitalWrite(BLUELED,LOW);
// 				}
// 				else
// 				{
// 					digitalWrite(BLUELED,HIGH);
// 				}
		REG_TC5_INTFLAG = TC_INTFLAG_OVF;        // Clear the MC0 interrupt flag
	}
	return;
}

void TC3_Handler(){//Timer 3 interrupt handler, NOT USED
	//If test is completed return test complete
	if (TEST_STOP){
		//Serial.println("Test Completed");
		return;
	}
	
	// Check for match counter 1 (MC1) interrupt
	if (TC3->COUNT16.INTFLAG.bit.OVF && TC3->COUNT16.INTENSET.bit.OVF && TEST_START)
	{

		REG_TC3_INTFLAG = TC_INTFLAG_OVF;        // Clear the OVF interrupt flag
	}
	return;
}

/**
\brief Returns value for the overflow counter
\param[in] Clock Frequency in MHz, prescaler value, period of timer in milliseconds
\param[out] integer count value
*/
int countervalue (float Clock_FrequencyMHz,float prescaler, float period_ms){
	return (int) (( (Clock_FrequencyMHz*1000*period_ms) / (prescaler) )-1) ;
	
}

/*
* Name        : pin_setup
* Purpose     : This function configures PINS for the MCU
* Description : ...
* Input       : VOID
* Output      : VOID
*/
void pin_setup(void){
	// initialize digital pin LED_BUILTIN as an output.
	pinMode(REDLED,OUTPUT);
	pinMode(BLUELED, OUTPUT);
	pinMode(YELLOWLED,OUTPUT);
	pinMode (ANALOG_PIN, INPUT);
	
	pinMode(CTRL_VSTR,OUTPUT);
	analogWriteResolution(10); //the second point says to apply 0V from the DAC and use the trimmer so that -0.5V is visible on the output. But the DAC should be set to 3.3V because CTRL_VSTR and VSTR are inversely related
	analogWrite(CTRL_VSTR,setDAC(3200)); //set to 1023, 3.3V
	
	pinMode(HEATER_PWM,OUTPUT);
	pinMode(HELMHOLTZ_PWM,OUTPUT);
	pinMode(_RESET_ADC,OUTPUT); //active low
	digitalWrite(_RESET_ADC,HIGH); //setting it off
	
	pinMode(_PWDN_ADC,OUTPUT); //active low
	digitalWrite(_PWDN_ADC,LOW); //setting it off
	
	pinMode(START_ADC,OUTPUT); //active high
	digitalWrite(START_ADC,LOW); //setting it off
	
	pinMode(_DRDY_ADC,INPUT);
	pinMode(RED_LED,OUTPUT);
	pinMode(BLUE_LED,OUTPUT);
	pinMode(CLR_FREQ_DIVIDER,OUTPUT);
	pinMode(_CS_ADC,OUTPUT);

	//SPI pins are already initialized by Arduino
	pinMode(MOSI_ADC,OUTPUT);
	pinMode(SCK_ADC,OUTPUT);
	pinMode(MISO_ADC,INPUT);
	pinMode(Q11_FREQ_COUNTER,INPUT);
	pinMode(Q12_FREQ_COUNTER,INPUT);
	pinMode(Q13_FREQ_COUNTER,INPUT);
	pinMode(Q14_FREQ_COUNTER,INPUT);
	
	return;
}

/**
\Function :volt_to_temperature
\Input: Digital Voltage from ADC in millivolts
\Output: Temperature conversion in C from TMP235
\Formula : T = (V_out - V_offset)/Tc) + T_INFL
\Data sheet: https://www.ti.com/lit/ds/symlink/tmp235.pdf?ts=1594142817615&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMP235
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
Function: magnetic_field_mT
Purpose: Converts voltage to corresponding magnetic field (reference is for the  magnetic sensor)
Input: Voltage in millivolts
Output: Magnetic field density in milli_Tesla (1G = 0.1mT)
Data sheet: https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjxps3qg8PqAhXBo54KHUn5CvwQFjAAegQIBRAB&url=https%3A%2F%2Fwww.allegromicro.com%2F~%2Fmedia%2FFiles%2FDatasheets%2FA1318-A1319-Datasheet.ashx&usg=AOvVaw39zGCju7QuDLgpcH9PKde_
*/
float magnetic_field_mT(float voltage){
	#define QUIESCENT_VOLTAGE 1650              //1.65V Low->1.638, mean-> 1.65, high-> 1.662 measure this
	#define MAGNETIC_SENSITIVITY 13.5           //1.35mv/G --> 13.5mV/mT Low->1.289, mean-> 1.3, high-> 1.411
	#define TEMPERATURE_SENSITIVITY   0.12      //0.12%/C
	#define magnetic_sesnor_ERROR 1.5							//1.5% ERROR
	#define VOLTAGE_CLAMP_HIGH 2970			    //2.97V +40 mT
	#define VOLTAGE_CLAMP_LOW 0.33			    //2.97V -40mT
	#define MAX_MAGNETIC_FIELD 40              //40mT
	#define MIN_MAGNETIC_FIELD -40              //-40mT
	
	if (voltage == '\0') return '\0';
	else if (voltage == VOLTAGE_CLAMP_HIGH ) return MAX_MAGNETIC_FIELD ; //we can find some sort of error to throw here
	else if (voltage == VOLTAGE_CLAMP_LOW) return MIN_MAGNETIC_FIELD ;
	else return (float)((voltage-QUIESCENT_VOLTAGE)/(float)MAGNETIC_SENSITIVITY);   //polarity depends on the direction of the field
	
}

/**
\Function: System_FSM
\input : Void
\Output : Void
\Purpose : This state_machine runs the system
\ //STATES {IDLE, SPI_UPDATE, serial_update, Heater_update, Magnetic_field_update}
*/	
void System_fsm_Run (void){	
	#define IDLE 0 //idle state
	#define ADC_UPDATE 1 //reading data from the ADC using SPI at intervals
	#define SERIAL_UPDATE 2//sending data back to the computer through serial at intervals
	#define Heater_Update 3 //running the heater_FSM and get the updates
	#define Magnetic_Field_update 4 //running the magnetic field FSM and get the updates
	
	//variables
	
	float starting_test_count = 0;
	
	
	switch (System_fsm_state){
		case IDLE : {
				if (TEST_STOP){
					//clear and turn off all the outputs
					//turn off heater click
					//turn off magnetic field				
						digitalWrite(CTRL_VSTR,LOW);
						analogWrite(HEATER_PWM,LOW);
						analogWrite(HELMHOLTZ_PWM,LOW);
						digitalWrite(_RESET_ADC,LOW);
						digitalWrite(_PWDN_ADC,LOW);
						digitalWrite(START_ADC,LOW);
						digitalWrite(RED_LED,LOW);
						digitalWrite(BLUE_LED,LOW);
						digitalWrite(CLR_FREQ_DIVIDER,LOW);
						digitalWrite(MOSI_ADC,LOW);
						digitalWrite(SCK_ADC,LOW);
						heater_PWM_duty = 0;
						magnetic_PWM_duty = 0;
						
						//
						doc.clear(); //clear the document as this frees up the memory
						// Add values to the document
						doc["test id"] = Test_ID;
						//doc["test run"] = TEST_RUN; //0 test is not running, 1 test is running
						
						
						//doc["test stop"] = TEST_STOP; //1 test is stopped, 0 test is running
						if (TEST_STOP) doc["test stop"] = 1;
						else doc["test stop"] = 0;
						//doc["test error"] = TEST_ERROR; //0 no error, 1 there was an error
						if (TEST_ERROR) doc["test error"]=1;
						else doc["test error"]=0;
						
						doc["error message"] = error_message;
						
						// Add an array.
						JsonArray ADCdata = doc.createNestedArray("ADC data");
						for (int i =0; i < 29; i++){
							ADCdata.add(converted_ADC_data[i]);
						}
						//add another array
						JsonArray TESTdata = doc.createNestedArray("test data");

						TESTdata.add(current_test_time); //current test time
						TESTdata.add(measured_temperature); //temperature (C)
						TESTdata.add(measured_magnetic_field); //magnetic field (mT)
						//out current_test_timer, ADC converted data at intervals
						
						if (serial_signal) {
							send_data_to_serial();
							serial_signal = false;
						}
				}
				else if (TEST_START){
					//1. Setting voltage for test
					analogWrite(CTRL_VSTR,setDAC(desired_FPGA_voltage));
					
					//2. Setting total time for test
					starting_test_count = test_time_count; //initializing starting point
					current_test_time = (test_time_count - starting_test_count);//   /60; //current test time in secs
					
					//3. Setting time interval for sending data rate
					//Temporarily changed to counter file 
					//REG_TC3_COUNT16_CC0 =  countervalue(1,1024,serial_output_rate);                     // Set the TC3 CC0 register
					//while (TC3->COUNT16.STATUS.bit.SYNCBUSY);											// Wait for synchronization
					
					//4. Setting desired magnetic field in mT
					//This is set in "receiving instructions function"		
					//5. Setting desired temperature in C
					//This is set in "receiving instructions function"					
				}
				
			}
			break;
		case ADC_UPDATE: {

				//1. Update test timer and if time is hit, stop
				current_test_time = (test_time_count - starting_test_count); //  /60; testing secs
				if (current_test_time > desired_time_for_test*60) TEST_STOP=true;
	
				//2. Convert Raw ADC data to understandable values
				ADC_Auto_Scan(); //converting ADC data
				ADC_array_convert(); //converts the ADC array data
		}
			break;
		case SERIAL_UPDATE:{
			//1. Set Data to be sent to the user from the ADC update, data is sent in intervals in an Interrupt service routine
			//Preparing json file
			
			doc.clear(); //clear the document as this frees up the memory
			// Add values to the document
			doc["test id"] = Test_ID;
			//doc["test run"] = TEST_RUN; //0 test is not running, 1 test is running
			//doc["test stop"] = TEST_STOP; //1 test is stopped, 0 test is running
			if (TEST_STOP) doc["test stop"] = 1;
			else doc["test stop"] = 0;
			//doc["test error"] = TEST_ERROR; //0 no error, 1 there was an error
			if (TEST_ERROR) doc["test error"]=1;
			else doc["test error"]=0;
			
			doc["error message"] = error_message;
			
			// Add an array.
			JsonArray ADCdata = doc.createNestedArray("ADC data");
			for (int i =0; i < 29; i++){
				ADCdata.add(converted_ADC_data[i]);
			}
			//add another array
			JsonArray TESTdata = doc.createNestedArray("test data");

			TESTdata.add(current_test_time); //current test time
			TESTdata.add(measured_temperature); //temperature (C)
			TESTdata.add(measured_magnetic_field); //magnetic field (mT)
			//out current_test_timer, ADC converted data at intervals 
			
			if (serial_signal) {
				send_data_to_serial(); //function to send json packet to serial port
				serial_signal = false; //turn off the serial_signal flag
			}
			
		}
			break;
		case Heater_Update:{
			//update actual and desired temperature, run heater click FSM
			HEATER_START = true;  //enable heater
			heater_fsm_RUN();
			heater_fsm_transition();
			HEATER_START = false; //disable heater
			
		}
			break;
		case Magnetic_Field_update:{
			//update actual and desired magnetic field, fun magnetic field FSM
			//1.state_run
			magnetic_fsm_run();
			//2. state_transtions
			magnetic_fsm_transition();
			
		}
			break;
		default :System_fsm_state = IDLE;
	}
	return;
}

/**
\Function: System_FSM_Transition
\input : Void
\Output : Void
\Purpose : This function updates funtions for the System FSM
\ //STATES {IDLE, SPI_UPDATE, serial_update, Heater_update, Magnetic_field_update}
*/
void System_fsm_Transition(void)
{
	#define IDLE 0 //idle state
	#define ADC_UPDATE 1 //reading data from the ADC using SPI at intervals
	#define SERIAL_UPDATE 2 //sending data back to the computer through serial at intervals
	#define Heater_Update 3 //running the heater_FSM and get the updates
	#define Magnetic_Field_update 4 //running the magnetic field FSM and get the updates

	switch (System_fsm_state){
		case IDLE : {
			if (TEST_STOP) System_fsm_state = IDLE;	//else state = idle
			else if (TEST_START) System_fsm_state =ADC_UPDATE;
		}
		break;
		case ADC_UPDATE: {
			if (TEST_STOP) System_fsm_state = IDLE;
			else System_fsm_state = SERIAL_UPDATE;
		}
		break;
		case SERIAL_UPDATE:{
			if (TEST_STOP) System_fsm_state = IDLE;
			else System_fsm_state = Heater_Update;

		}
		break;
		case Heater_Update:{
			if (TEST_STOP) System_fsm_state = IDLE;
			else System_fsm_state = Magnetic_Field_update;
		}
		break;
		case Magnetic_Field_update:{
			if (TEST_STOP) System_fsm_state = IDLE;
			else System_fsm_state = ADC_UPDATE;
		}
		break;
		default :System_fsm_state = IDLE;
	}
	return;
}


/************************************************************************/
/* ADC FUNCTIONS                                                        */
/************************************************************************/
/*
Name	:void ADC_CHID_STATUS
Purpose : Returns the Channel ID from the STATUS_byte byte
Input	: STATUS_byte Byte
Output  : Channel ID of the read data
*/
uint8_t ADC_CHID_STATUS(uint8_t STATUS_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |
		return (uint8_t) (STATUS_byte & 0b00011111); //return last 5 bits of STATUS_byte byte
}
/*
Name        : ADC_NEW_STATUS_BIT
Purpose		: Checks if the NEW bit is set in the STATUS_byte byte
Description : The NEW bit is set when the results of a Channel Data Read Command returns new channel data. 
			  The bit remains set indefinitely until the channel data are read
Input	    : STATUS_byte Byte
Output		: TRUE if bit is set, FALSE if bit is not set 
*/
boolean ADC_NEW_STATUS_BIT(uint8_t STATUS_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |	
		uint8_t NEW_bit = (STATUS_byte & 0b10000000)>>7;
		if (NEW_bit == 0){
			return false;
		}
		else {
			return true;
		}
}
/*
* Name        : ADC_OVF_STATUS_BIT
* Purpose     : Checks if the OVF bit is set in the STATUS_byte byte
* Description : When this bit is set, it indicates that the differential voltage applied to the ADC inputs have exceeded the range
				of the converter |VIN| > 1.06VREF. During over-range, the output code of the converter clips to either positive FS
				(VIN ? 1.06 × VREF) or negative FS (VIN ? –1.06 × VREF). This bit, with the MSB of the data, can be used to
				detect positive or negative over-range conditions
* Input       : STATUS_byte Byte
* Output      : TRUE if bit is set, FALSE if bit is not set 
*/
boolean ADC_OVF_STATUS_BIT(uint8_t STATUS_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |
		uint8_t OVF_bit = (STATUS_byte & 0b01000000)>>6;
		if (OVF_bit == 0){
			return false;
		}
		else {
			return true;
		}
}
/*
* Name        : ADC_SUPPLY_STATUS_BIT
* Purpose     : Checks if the SUPPLY bit is set in the STATUS_byte byte
* Description : This bit indicates that the analog power-supply voltage (AVDD – AVSS) is below a preset limit. The SUPPLY bit
				is set when the value falls below 4.3V (typically) and is reset when the value rises 50mV higher (typically) than
				the lower trip point. The output data of the ADC may not be valid under low power-supply conditions.
* Input       : STATUS_byte Byte
* Output      : TRUE if bit is set, FALSE if bit is not set 
*/
boolean ADC_SUPPLY_STATUS_BIT(uint8_t STATUS_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |	
		uint8_t SUPPLY_bit = (STATUS_byte & 0b00100000)>>5;
		if (SUPPLY_bit == 0){
			return false;
		}
		else {
			return true;
		}
}
/*
Name : ADCregisterWrite
Purpose : Writes 8bit value into provided register address
Input : Register address, Value to be written
Output : void
*/
void ADC_RegisterWrite(uint8_t REG_address, uint8_t Value){
  uint8_t command = 0x60| REG_address; // 8b'0100_0000 | REG_address
  SPI.begin(); //initialize SPI pins
  SPI.beginTransaction (SPISettings (ADC_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(_CS_ADC,LOW);
  delayMicroseconds(20);
  SPI.transfer(command); // send the command byte
  SPI.transfer(Value);   // send the value of register 
  delayMicroseconds(20);
  digitalWrite(_CS_ADC,HIGH);
  SPI.endTransaction();
    
  }
/*
Name : ADCregisterRead
Purpose : Reads 8bit value stored in register
Input : Register address
Output : Value in register
*/	
uint8_t ADC_RegisterRead(uint8_t REG_address){
	uint8_t command = 0x40 | REG_address;
	uint8_t REG_Value = NULL;
	SPI.begin(); //initialize SPI pins
	SPI.beginTransaction (SPISettings (ADC_SPI_SPEED, MSBFIRST, SPI_MODE0));
	digitalWrite(_CS_ADC,LOW);
	delayMicroseconds(20);
	SPI.transfer(command); // send command
	REG_Value = SPI.transfer(0x0);   // Read response
	delayMicroseconds(20);
	digitalWrite(_CS_ADC,HIGH);
	SPI.endTransaction();
	
	return REG_Value;
}
/*
Name : ADCchannelRead_registerFormat
Purpose : Reads converted data in a channel ID from the ADC and returns it to the user
		  Reads 3 bytes: STATUS_BYTE, MSB_BYTE, LSB_BYTE
Input : VOID
Output : Converted value in a 32bit data, [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
*/
uint32_t ADCchannelRead_registerFormat(void){
	ADC_toggle_start_pin(); //Pulse the start pin, this moves ADC the conversion to the next channel to be converted
			
	//Channel data read - register format
	uint8_t command = 0x30;
	uint32_t STATUS_byte = 0xFFFFFFFF;    //STATUS_byte byte
	uint32_t MSB_data = 0xFFFFFFFF;  //upper 8 bits
	uint32_t LSB_data = 0xFFFFFFFF;  //lower 8 bits
	uint32_t Channel_data = 0;
			
	SPI.begin(); //initialize SPI pins
	SPI.beginTransaction (SPISettings (ADC_SPI_SPEED, MSBFIRST, SPI_MODE0));
	digitalWrite(_CS_ADC,LOW);
	delayMicroseconds(20);
			
	SPI.transfer(command); // send command
	STATUS_byte = SPI.transfer(0x0);   // Read STATUS_byte byte
	MSB_data = SPI.transfer(0x0); // Read MSB byte
	LSB_data = SPI.transfer(0x0); // Read LSB byte
			
	Serial.println("\n\n");
	delayMicroseconds(20);
	digitalWrite(_CS_ADC,HIGH);
	SPI.endTransaction();
	
	//add STATUS_byte byte to positions [23:16]
	//add MSB to [15:8];
	//add LSB to [7:0]
	Channel_data |= (STATUS_byte<<16)|(MSB_data<<8)|LSB_data;
	return Channel_data;
	
}
/*
* Name        : ADC_Setup
* Purpose     : This functions sets up the ADC
* Description : 
* Input       : VOID
* Output      : VOID
*/
void ADC_Setup(){
	  //ADC GAIN
	  converted_ADC_data[27] = 1;
	  //ADC Vref in mV
	  converted_ADC_data[28] = 3330; //Vref = 3.33V
	  
	  delay(50);
	  //Setting up ADC
	  digitalWrite(_PWDN_ADC,HIGH); //setting it off
	  delay(200);
	  //1. Reset the SPI interface
	  digitalWrite(_CS_ADC,HIGH);
	  delay(150);
	  //2.Stop the converter by setting start pin low
	  digitalWrite(START_ADC,LOW);
	  //3.Reset the converter, Pulse the reset button low
	  digitalWrite(_RESET_ADC,LOW);
	  delay(150);
	  digitalWrite(_RESET_ADC,HIGH);
	  delay(500); //delay for ADC startup time
	  //4. Configure registers
	  for (int i = 0; i < ADC_Register_Array_Size; i++){
		  ADC_RegisterWrite(ADC_Register_Addresses[i],ADC_Register_Defaults[i]); //write values in registers
	  }
	  
	  //5. Check register values
	  for (int j = 0; j < ADC_Register_Array_Size; j++){
		  if (ADC_RegisterRead(ADC_Register_Addresses[j]) != ADC_Register_Defaults[j]){
			  Serial.print("Error in register address : ");
			  Serial.println(ADC_Register_Addresses[j]);
		  }
	  }
	  //6. Start the converter
	  //set up ADC conversions
	  for (int i = 5; i<0 ;i--)
	  {
		  ADC_Auto_Scan();
		  delay(100);
	  }
	  
}
/*
* Name        : ADC_RESET
* Purpose     : This function resets the ADC
* Description : ...
* Input       : VOID
* Output      : VOID
*/
void ADC_RESET(void){
	digitalWrite(_RESET_ADC,LOW);
	delay(150);
	digitalWrite(_RESET_ADC,HIGH);
	delay(500); //delay for ADC startup time
}
/*
* Name        : ADC_RETURN_STATUSBYTE
* Purpose     : This function returns the STATUS byte from the Channel Data register read
* Description : ...
* Input       : 32bit Value from SPI reading, this 32bit value is from "ADCchannelRead_registerFormat" function
* Output      : 8bit value of status byte
*/
uint8_t ADC_RETURN_STATUSBYTE(uint32_t RAW_CHANNEL_REGISTER_READ_DATA){
	//The input 32bit value contains [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
	//We return status
	return (uint8_t)((RAW_CHANNEL_REGISTER_READ_DATA >> 16)& 0x000000FF); //0X000F = 0b0000_0000_0000_0000_0000_0000_1111_1111, this returns the status byte
}
/*
* Name        : ADC_RETURN_RAWDATA
* Purpose     : This function returns the RAW data(MSB+LSB) byte from the Channel Data register read
* Description : ...
* Input       : 32bit Value from SPI reading, this 32bit value is from "ADCchannelRead_registerFormat" function
* Output      : 16bit value of Raw ADC data
*/
uint16_t ADC_RETURN_RAWDATA(uint32_t RAW_CHANNEL_REGISTER_READ_DATA){
	//The input 32bit value contains [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
	//We return MSB and LSB
	return (uint16_t)((RAW_CHANNEL_REGISTER_READ_DATA )& 0x0000FFFF); //0X0F = 0b0000_0000_0000_0000_1111_1111_1111_1111, this returns the status byte
}
/*
* Name        : ADC_Auto_Scan
* Purpose     : Reads all the ADC channels selected in configuration settings(Registers MUXDIF,MUXSG0,MUXSG1) and stores the RAW data in the ADC_raw_data_array, the Channel ID is the corresponding index
* Description : 
* Input       : VOID
* Output      : VOID
*/
void ADC_Auto_Scan(){
/*
*Read all values from the ADC
*Method
*Count number of loops
*Scan through the channel ID's
*if new bit is set and OVF+SUPPLY bits are not set, store the raw value
*The CHID is the Array index
*Return
*/
//counting all the number of channels to measure
	int count = countSetBits(MUXDIF_default)+countSetBits(MUXSG0_default)+countSetBits(MUXSG1_default); //this is the number of loops to make
	for (int p = 0; p < count; p++){
		uint32_t Channel_data = ADCchannelRead_registerFormat();
		uint8_t Status_byte = ADC_RETURN_STATUSBYTE(Channel_data);
		int CHID = ADC_CHID_STATUS(Status_byte);
		uint16_t Raw_data = ADC_RETURN_RAWDATA(Channel_data);
	
		if ((ADC_NEW_STATUS_BIT(Status_byte) == true) && (ADC_SUPPLY_STATUS_BIT(Status_byte)== false) && (ADC_OVF_STATUS_BIT(Status_byte) == false))
		{
			//store the data in the array
			if (CHID > 0x19) //for some reason CHID skips address 0x19h
			{
				raw_ADC_data[CHID-1] = Raw_data; 
			}
			else {
				raw_ADC_data[CHID] = Raw_data; //store the value
			}
		}
		else {
			//try to report error
			if (ADC_SUPPLY_STATUS_BIT(Status_byte)) { }//Serial.println("Error: ADC Status SUPPLY bit set");}
			else if (ADC_OVF_STATUS_BIT(Status_byte)) { }//Serial.println("Error: ADC Status OVF bit set");}

		}
	}
}

/* Function to get no of set bits in binary 
 *representation of passed binary no. 
 */
unsigned int countSetBits(int n){
	unsigned int count = 0;
	while (n) {
		n &= (n - 1);
		count++;
	}
	return count;
} 
/*Function: ADC_toggle_start_pin
*Purpose : Toggles the start pin on the ADC
Definition: Start PIN -> Start conversion input: active high.
*Input : Void
*Output : Void
*/
void ADC_toggle_start_pin(void){
	digitalWrite(START_ADC,HIGH); //starts conversion
	delayMicroseconds(2); //wait for data to settle
	digitalWrite(START_ADC,LOW); //stops conversion so that we can go to the next Channel ID, Check channel ID in Table 10 of the ADC manual
	delayMicroseconds(600); //Start condition to _DRDY delay, Table 8 -  8836*(1/15.7Mhz) us
	/*
	For example, if channels 3, 4, 7, and 8 are selected for measurement in the list, the ADS1158 converts
	the channels in that order, skipping all other channels. After channel 8 is converted,
	starts over, beginning at the top of the channel list, circuitry are completely disabled. channel 3
	*/
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

int ADC_mv(int ADC_reading){
	#define MAX_POSITIVE_VOLTAGE 0x7FFF
	#define MAX_NEGATIVE_VOLTAGE 0x8000
	#define REFERENCE_VOLTAGE_ADC_mV 65536
	#define ADC_RESOLUTION 32768 //2^15 bits as 16th bit is used for 2's complement
	#define ADC_RANGE 4900 //4900mV (-1.6V to 3.3V)
	#define VREF 3340 //3.3V //this is VREFP-VREFN
	
	//converted_ADC_data[28] == Vref
	//converted_ADC_data[27] == ADC gain
	//return ADC_reading*(VREF/(float)0x7800);
	return (ADC_reading*converted_ADC_data[28]*1000) / (double) ((double)30720  * converted_ADC_data[27]);
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

	//for OFFSET conversion
	converted_ADC_data[24] = raw_ADC_data[24]; //Ideally, the code from this register function is 0h, but varies because of the noise of the ADC and offsets stemming from the ADC and
// 	external signal conditioning. This register can be used to calibrate or track the offset of the ADS1158 and Figure 40. Conversion Control, Auto-Scan Mode external signal conditioning.
	//for VCC conversion
	converted_ADC_data[25] = ((float) raw_ADC_data[25]/(double) 3072);
	//for ADC temp conversion
	converted_ADC_data[26] =( ((double) ( (1000*ADC_mv(twos_complement_to_int(raw_ADC_data[26],NUMBER_OF_BITS_ADC))) - 168000)/(double) ADC_temp_sensor_coefficient)+ 25);
	//ADC GAIN conversion
	converted_ADC_data[27] = ((float) raw_ADC_data[27]/(double) 30720);
	//ADC REF conversion
	converted_ADC_data[28] = ((float) raw_ADC_data[28]/(double) 3072);
	for (int i =0; i <= 23; i++) //for loop is at the bottom so that we can first convert useful constants used in these calculations
	{
		converted_ADC_data[i] = ADC_mv(twos_complement_to_int(raw_ADC_data[i],NUMBER_OF_BITS_ADC)); //converting all the voltages to millivolts
	}
	
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
#define magnetic_error 0.015 //error is 1.5% gotten from the magnetic field data sheet in mT

void magnetic_fsm_transition(void)
{
	float magnetic_field_difference = (float) (measured_magnetic_field - desired_magnetic_field); //getting the error
	
	if (magnetic_fsm_state == magnetic_fsm_idle_state) 
	{
		if ( magnetic_field_difference > magnetic_error) magnetic_fsm_state = magnetic_fsm_decrease_state; //reduce PWM duty if above desired + threshold
		else if ( abs(magnetic_field_difference) <= magnetic_error) magnetic_fsm_state = magnetic_fsm_idle_state; //maintain PWM if between threshold
		else magnetic_fsm_state = magnetic_fsm_increase_state; //PWM duty increase if below the desired+threshold
	}
	else if (magnetic_fsm_state == magnetic_fsm_increase_state) 
	{
		if ( magnetic_field_difference > magnetic_error) magnetic_fsm_state = magnetic_fsm_decrease_state; //reduce PWM duty if above desired + threshold
		else if ( abs(magnetic_field_difference) <= magnetic_error) magnetic_fsm_state = magnetic_fsm_idle_state; //maintain PWM if between threshold
		else magnetic_fsm_state = magnetic_fsm_increase_state; //PWM duty increase if below the desired+threshold
	}
	else if (magnetic_fsm_state == magnetic_fsm_decrease_state) 
	{
		if ( magnetic_field_difference > magnetic_error) magnetic_fsm_state = magnetic_fsm_decrease_state; //reduce PWM duty if above desired + threshold
		else if ( abs(magnetic_field_difference) <= magnetic_error) magnetic_fsm_state = magnetic_fsm_idle_state; //maintain PWM if between threshold
		else magnetic_fsm_state = magnetic_fsm_increase_state; //PWM duty increase if below the desired+threshold
	}
	return;
}

/**
\Function: heater_fsm_RUN
\input : Void
\Output : Void
\Purpose : Function runs the magnetic helmholtz coil
*/
void magnetic_fsm_run(void)
{
	if (magnetic_fsm_state == magnetic_fsm_idle_state){	} //no change
	else if (magnetic_fsm_state == magnetic_fsm_increase_state) 
	{
		//PWM is increased until desired state
		if (magnetic_PWM_duty >= ANALOG_RESOLUTION) magnetic_PWM_duty=ANALOG_RESOLUTION;
		else magnetic_PWM_duty++;
	}
	else if (magnetic_fsm_state == magnetic_fsm_decrease_state) 
	{
		//PWM is decreased until desired state
		if (magnetic_PWM_duty <= 0) magnetic_PWM_duty=0;
		else magnetic_PWM_duty--;
	}
	analogWrite(HELMHOLTZ_PWM, magnetic_PWM_duty); //write to the pin after changing the duty
}

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
	#define heater_fsm_margin 4 //error is +/- 1C as we are using integers
	#define heater_threshold_temp 50 // above 40C is hot for human touch
	/**
	\Function: heater_fsm_transition
	\input : Void
	\Output : Void
	\Purpose : Function updates the heater FSM states
	*/
void heater_fsm_transition(void)
{	
	if (HEATER_START){
		float temperature_difference =  (float)(measured_temperature - desired_temperature);
		
		switch(heater_fsm_state){
			case heater_fsm_OFF: {
				heater_fsm_state = heater_fsm_idle;
				break;
			}
			case heater_fsm_cooling:
			case heater_fsm_heating:
			case heater_fsm_idle: {
				if ( temperature_difference > heater_fsm_margin) heater_fsm_state = heater_fsm_cooling; //cool temp is above desired + threshold
				else if ( abs(temperature_difference) <= heater_fsm_margin) heater_fsm_state = heater_fsm_idle; //maintain heat if it between 0.5C threshold
				else heater_fsm_state = heater_fsm_heating; //heat up if is below the desired+threshold
				
				break;
			}
// 			case heater_fsm_heating: {
// 				if ( temperature_difference > heater_fsm_margin) heater_fsm_state = heater_fsm_cooling; //cool temp is above desired + threshold
// 				else if ( abs(temperature_difference) <= heater_fsm_margin) heater_fsm_state = heater_fsm_idle; //maintain heat if it between 0.5C threshold
// 				else heater_fsm_state = heater_fsm_heating; //heat up if is below the desired+threshold
// 				break;
// 			}
// 			case heater_fsm_cooling: {
// 				if ( temperature_difference > heater_fsm_margin) heater_fsm_state = heater_fsm_cooling; //cool temp is above desired + threshold
// 				else if ( abs(temperature_difference) <= heater_fsm_margin) heater_fsm_state = heater_fsm_idle; //maintain heat if it between 0.5C threshold
// 				else heater_fsm_state = heater_fsm_heating; //heat up if is below the desired+threshold
// 				break;
// 			}
			default: heater_fsm_state = heater_fsm_OFF;
		}
	}
	else heater_fsm_state = heater_fsm_OFF; //if the heater is stopped at any moment, it goes off
	
	return;
}

/**
\Function: heater_fsm_RUN
\input : Void
\Output : Void
\Purpose : Function runs the heating system
*/
void heater_fsm_RUN(void){
	if (measured_temperature < heater_threshold_temp && digitalRead(BLUE_LED)!=HIGH) digitalWrite(BLUE_LED,HIGH);//turn on safe to touch LED if not high already
	else digitalWrite(BLUE_LED,LOW);  //turn off safe to touch LED
	
	switch(heater_fsm_state){
		case heater_fsm_OFF: {
			//all pins off
			digitalWrite(RED_LED, LOW); //turn of heating signal
			heater_PWM_duty =0;
			break;
		}
		case heater_fsm_idle: {
			digitalWrite(RED_LED, LOW); 
			break;
		}
		case heater_fsm_heating: {
			digitalWrite(RED_LED,HIGH); //turn on heating signal
			if (heater_PWM_duty >= ANALOG_RESOLUTION) heater_PWM_duty=ANALOG_RESOLUTION;
			else heater_PWM_duty++;	
			break;
		}
		case heater_fsm_cooling: {
			digitalWrite(RED_LED, LOW); //turn of heating signal
			if (heater_PWM_duty <= 0) heater_PWM_duty=0;
			else heater_PWM_duty--;	
			break;
		}
		default: heater_fsm_state = heater_fsm_OFF;
	}
	
	analogWrite(HEATER_PWM,heater_PWM_duty);
	return;
}
/************************************************************************/
/* DAC FUNCTIONS                                                        */
/************************************************************************/
/**
\Function: setDAC
\input : Desired voltage in mini volts
\Output : DAC value
\Purpose : this function converts a user entered voltage value into a 10 bit DAC value
*/
int setDAC(float volt) {
	//formula for calculating DAC output voltage Vdac = (dVal / 1023)*3.3V
	return (int)((volt*1023)/3300);
}

/************************************************************************/
/* OTHER FUNCTIONS                                                      */
/************************************************************************/
/**
\Function: raise_MCU_error
\input : Error message as String
\Output : Void
\Purpose : Function raises MCU error, updates the error message
*/
void raise_MCU_error(String error_text){
	//This function raises and error
	TEST_ERROR = true;
	error_message = error_text;
	
	return;
}