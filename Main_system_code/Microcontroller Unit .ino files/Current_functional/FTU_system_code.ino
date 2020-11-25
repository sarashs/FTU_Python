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
* \@change_log : 
* tc3 timer removed
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
*Faster Serial communication - 0.1uS
*Reset Arduino after test stops
*Frequency divider functionality
*
*H*/
#include "SPI.h"
#include "ArduinoJson.h"
#include "ArduinoJson.hpp"
#include "Arduino.h"
#include "samd21/include/samd21g18a.h"
#include "math.h"
#include "strings.h"
#include "WString.h"
#include "time.h"


/************************************************************************/
/* PINS DESCRIPTIONS                                                    */
/************************************************************************/
#define ctrl_vstr A0 //A0 the control signal for the voltage stress circuit, labeled CTRL_VSTR in figure 1
#define heater_pwm A3 //A3 This is the control signal for the Heater board
#define helmholtz_pwm A4 //A4 This is the control signal for the Helmholtz circuit
#define _reset_adc 0          //D0 Should be configured as a digital output. Connected to *RESET on the ADC
#define _pwdn_adc 1           //D1 Should be configured as a digital output. Connected to *PWDN on the ADC
#define start_adc 2           //D2 Should be configured as a digital output. Connected to START on the ADC.
#define _drdy_adc 3           //D3 Should be configured as a digital input. Connected to *DRDY on the ADC.
#define red_led 4             //D4 Should be configured as a digital output. Control signal for the red LED on the heater board.
#define blue_led 5            //D5 Should be configured as a digital output. Control signal for the Blue LED on the heater board.
#define clr_freq_divider 6    //D6 Should be configured as a digital output. The clear command for the frequency divider. labeled CLR in figure 1.
#define _cs_adc 7             //D7 Should be configured as a digital output. This is the *CS line for SPI communication with the ADC
#define mosi_adc 8            //D8 Should be configured as a digital output. This is the MOSI line on the SPI bus. labeled DIN on the ADC.
#define sck_adc 9             //D9 Should be configured as a digital output. This is the SCLK line on the SPI bus.
#define miso_adc 10           //D10 Should be configured as a digital input. This is the MISO line on the SPI bus. labeled DOUT on the ADC.
#define q11_freq_counter 11   //D11 Should be configured as a digital input. Connected to Q11 on the frequency divider
#define q12_freq_counter 12   //D12 Should be configured as a digital input. Connected to Q12 on the frequency divider
#define q13_freq_counter 13   //D13 Should be configured as a digital input. Connected to Q13 on the frequency divider
#define q14_freq_counter 14   //D14 Should be configured as a digital input. Connected to Q14 on the frequency divider
//A1, A2, A5, A6   4 MCU pins that are not used in the design and left disconnected. They will be accessible through the male/female headers mounted on the MCU board.
#define analog_resolution 1023
#define BAUD_RATE 256000 //9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000
#define number_of_adc_arrays 1

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
#define ADC_CLK_SPEED 15700000
#define number_of_bits_adc 16
#define adc_register_array_size 9
#define adc_spi_speed 3925000  //6MHZ -> Must be less than 0.5 adc_clk(15.7MHz) best if in bunches of 1/2,1/4,1/8 of the ADC clock use (3.925MHz) or (7.85Mhz)
//24 OFFSET, 25-> ADC VCC in mV,26-> ADC TEMP in C, 27-> GAIN V/V, 28-> REF
#define ADC_temp_sensor_coefficient 563 //563uV if ADS1158 and test PCB temperatures are forced together,
//394uV if ADS1158 temperature is forced and test PCB is in free air

//ADC GPIO pins
#define adc_gpio_blue_led 6
#define adc_gpio_red_led 7

//For converting ADC raw value to mV
#define MAX_POSITIVE_VOLTAGE 0x7FFF
#define MAX_NEGATIVE_VOLTAGE 0x8000
#define REFERENCE_VOLTAGE_ADC_mV 65536
#define ADC_RESOLUTION 32768 //2^15 bits as 16th bit is used for 2's complement
#define ADC_RANGE 4900 //4900mV (-1.6V to 3.3V)
#define ADC_GAIN 1 //this is the normal ADC GAIN
#define ADC_VREF 3.340 //3.3V //this is VREFP-VREFN

const uint8_t adc_register_addresses[adc_register_array_size] = {CONFIG0_address,CONFIG1_address,MUXSCH_address,
MUXDIF_address,MUXSG0_address,MUXSG1_address,SYSRED_address,GPIOC_address,GPIOD_address};

//Default ADC values to be programmed
#define CONFIG0_default 0x02
#define CONFIG1_default 0x03 //0x00 default
#define MUXSCH_default 0x00
#define MUXDIF_default 0x00
#define MUXSG0_default 0xFF //0xFF
#define MUXSG1_default 0x7F  //	0x7F
#define SYSRED_default 0x3D  //0x3D if you wanna see measurements
#define GPIOC_default 0x00
#define GPIOD_default 0x00

//Default ADC commands
#define Pulse_convert_command 0x80
#define Reset_command 0xC0

const uint8_t adc_register_defaults[adc_register_array_size] = {CONFIG0_default,CONFIG1_default,MUXSCH_default,
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

uint16_t raw_adc_data [29] = {0}; //initialize the array with zero
double converted_adc_data[29] = {0}; //this array hold the converted ADC data have to change this to  double
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

volatile float desired_temperature = 43;     //in C, Set to a default value but actual value comes from MCU instructions
volatile float desired_magnetic_field = 0;  //in mT, Set to a default value but actual value comes from MCU instructions
volatile int desired_time_for_test = 2;	 //time in minutes, Set to a default value but actual value comes from MCU instructions
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
/* USING JSON LIBRARY TO SEND COMMUNICATE WITH PYTHON SCRIPT            */
/************************************************************************/
/**
* @see Original source : "https://arduinojson.org/"
* preparing the JSON document to be used in the test
* Allocate the JSON document

* StaticJsonObject allocates memory on the stack, it can be
* replaced by DynamicJsonDocument which allocates in the heap.
*1024 is the RAM dedicated to this document 
*/
DynamicJsonDocument  doc(800*number_of_adc_arrays); //650 per doc

/**
 * This setup function is run before a test starts and sets up the system
 * 
 * @param void
 * @return void
 */
void setup() {
  analogWriteResolution(10);
  analogReference(AR_INTERNAL2V23); //sets 3.3V reference
  pin_setup(); //Setup MCU pins
  delay(1000);
  adc_setup(); //function sets up the ADC
  
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


}

/************************************************************************/
/*	ADC FUNCTIONS                                                       */
/************************************************************************/


/**
 * This function returns the Channel ID from the STATUS_byte byte
 * 
 * \@param STATUS_byte (8 bits)
 * 
 * \@return uint8_t, 8 bit value representing the channel ID from where the data is read
 */
uint8_t adc_chid_status(uint8_t status_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |
		return (uint8_t) (status_byte & 0b00011111); //return last 5 bits of STATUS_byte byte
}

/**
 * \@brief Checks if the NEW bit is set in the STATUS_byte byte
 *		   The NEW bit is set when the results of a Channel Data Read Command returns new channel data. 
 * 
 * \@param STATUS_byte
 * 
 * \@return boolean
 */
boolean adc_new_status_bit(uint8_t status_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |	
		uint8_t NEW_bit = (status_byte & 0b10000000)>>7;
		if (NEW_bit == 0){
			return false;
		}
		else {
			return true;
		}
}

/**
 * \@brief  Checks if the OVF bit is set in the STATUS_byte byte
 *			Description : When this bit is set, it indicates that the differential voltage applied to the ADC inputs have exceeded the range
 *			of the converter |VIN| > 1.06VREF. During over-range, the output code of the converter clips to either positive FS
 *			(VIN ? 1.06 × VREF) or negative FS (VIN ? –1.06 × VREF). This bit, with the MSB of the data, can be used to
 *			detect positive or negative over-range conditions 
 * 
 * \@param STATUS_byte
 * 
 * \@return boolean true if bit is set, false if bit is not set 
 */
boolean adc_ovf_status_bit(uint8_t status_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |
		uint8_t OVF_bit = (status_byte & 0b01000000)>>6;
		if (OVF_bit == 0){
			return false;
		}
		else {
			return true;
		}
}

/**
 * \brief    Checks if the SUPPLY bit is set in the STATUS_byte byte
 *			 Description : This bit indicates that the analog power-supply voltage (AVDD – AVSS) is below a preset limit. The SUPPLY bit
 *			 is set when the value falls below 4.3V (typically) and is reset when the value rises 50mV higher (typically) than
 *			 the lower trip point. The output data of the ADC may not be valid under low power-supply conditions.
 * 
 * \@param STATUS_byte
 * 
 * \@return boolean true if bit is set, false if bit is not set 
 */
boolean adc_supply_status_bit(uint8_t status_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |	
		uint8_t SUPPLY_bit = (status_byte & 0b00100000)>>5;
		if (SUPPLY_bit == 0){
			return false;
		}
		else {
			return true;
		}
}

/**
 * \@brief  Writes 8bit value into provided register address
 *  
 * \@param REG_address
 * \@param Value to be written
 * 
 * \@return void
 */
void adc_register_write(uint8_t reg_address, uint8_t value){
  uint8_t command = 0x60| reg_address; // 8b'0100_0000 | REG_address
  SPI.begin(); //initialize SPI pins
  SPI.beginTransaction (SPISettings (adc_spi_speed, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_adc,LOW);
  delayMicroseconds(2);
  SPI.transfer(command); // send the command byte
  SPI.transfer(value);   // send the value of register 
  delayMicroseconds(2);
  digitalWrite(_cs_adc,HIGH);
  SPI.endTransaction();
    
  }
  
 /**
  * \@brief This function sends a command to the ADC
  * 
  * \@param command , 
  * 
  * \@return void
  */
 void adc_send_command(uint8_t command){
	 
	 SPI.begin(); //initialize SPI pins
	 SPI.beginTransaction (SPISettings (adc_spi_speed, MSBFIRST, SPI_MODE0));
	 digitalWrite(_cs_adc,LOW);
	 delayMicroseconds(2);
	 SPI.transfer(command); // send the command byte
	 delayMicroseconds(2);
	 digitalWrite(_cs_adc,HIGH);
	 SPI.endTransaction();
	 
 }

/**
 * \@brief Reads 8bit value stored in register
 * 
 * \@param REG_address, register address
 * 
 * \@return uint8_t -> value in register
 */
uint8_t adc_register_read(uint8_t reg_address){
	uint8_t command = 0x40 | reg_address;
	uint8_t reg_value = NULL;
	SPI.begin(); //initialize SPI pins
	SPI.beginTransaction (SPISettings (adc_spi_speed, MSBFIRST, SPI_MODE0));
	digitalWrite(_cs_adc,LOW);
	delayMicroseconds(2);
	SPI.transfer(command); // send command
	reg_value = SPI.transfer(0x0);   // Read response
	delayMicroseconds(2);
	digitalWrite(_cs_adc,HIGH);
	SPI.endTransaction();
	
	return reg_value;
}

/**
 * \@brief Reads converted data in a channel ID from the ADC and returns it to the user, Reads 3 bytes: STATUS_BYTE, MSB_BYTE, LSB_BYTE
 * 
 * \@param void 
 * 
 * \@return uint32_t ->  Converted value in a 32bit data, [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
 */
uint32_t adc_channel_read_register_format(void){
	//adc_toggle_start_pin(); //Pulse the start pin, this moves ADC the conversion to the next channel to be converted
	adc_send_command(Pulse_convert_command); //send pulse convert command
			
	//Channel data read - register format
	uint8_t command = 0x30;
	uint32_t STATUS_byte = 0xFFFFFFFF;    //STATUS_byte byte
	uint32_t MSB_data = 0xFFFFFFFF;  //upper 8 bits
	uint32_t LSB_data = 0xFFFFFFFF;  //lower 8 bits
	uint32_t Channel_data = 0;
			
	SPI.begin(); //initialize SPI pins
	SPI.beginTransaction (SPISettings (adc_spi_speed, MSBFIRST, SPI_MODE0));
	digitalWrite(_cs_adc,LOW);
	delayMicroseconds(2); //CS low to first clock is minimum 2.5/15.7Mhz (we need 0.19us) 
			
	SPI.transfer(command); // send command
	STATUS_byte = SPI.transfer(0x0);   // Read STATUS_byte byte
	MSB_data = SPI.transfer(0x0); // Read MSB byte
	LSB_data = SPI.transfer(0x0); // Read LSB byte
			
	delayMicroseconds(2);
	digitalWrite(_cs_adc,HIGH);
	SPI.endTransaction();
	
	//add STATUS_byte byte to positions [23:16]
	//add MSB to [15:8];
	//add LSB to [7:0]
	Channel_data |= (STATUS_byte<<16)|(MSB_data<<8)|LSB_data;
	return Channel_data;
	
}

/**
 * \@brief This functions sets up the ADC
 *         STEPS : 1. Reset the SPI Interface, 2.Stop the converter, 3.Reset the Converter, 4.Configure the registers, 5.Verify Register Data, 6.Start the converter, 7.Read Channel Data
 * \@see Page 40 configuration guide. "https://www.ti.com/lit/ds/symlink/ads1158.pdf?ts=1603990103929&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1158"
 * 
 * 
 * \@return void
 */
void adc_setup(){
	  //ADC GAIN
	  converted_adc_data[27] = ADC_GAIN;
	  //ADC Vref in mV
	  converted_adc_data[28] = ADC_VREF; //Vref = 3.33V
	  
	  delay(50);
	  //Setting up ADC
	  digitalWrite(_pwdn_adc,HIGH); //setting it off
	  delay(200);
	  //1. Reset the SPI interface
	  digitalWrite(_cs_adc,HIGH);
	  delay(150);
	  //2.Stop the converter by setting start pin low
	  digitalWrite(start_adc,LOW);
	  //3.Reset the converter, Pulse the reset button low
	  digitalWrite(_reset_adc,LOW);
	  delay(150);
	  digitalWrite(_reset_adc,HIGH);
	  delay(500); //delay for ADC startup time
	  //4. Configure registers
	  for (int i = 0; i < adc_register_array_size; i++){
		  adc_register_write(adc_register_addresses[i],adc_register_defaults[i]); //write values in registers
	  }
	  
	  //5. Check register values
	  for (int j = 0; j < adc_register_array_size; j++){
		  if (adc_register_read(adc_register_addresses[j]) != adc_register_defaults[j]){
			  //Raise MCU error
			  String error_text = "Error in register address : ";
			  error_text.concat(adc_register_addresses[j]);
			  raise_mcu_error(error_text);
		  }
	  }
	  //6. Start the converter
	  //set up ADC conversions
	  for (int i = 0; i<5 ; i++)
	  {
		  adc_auto_scan(raw_adc_data);
		  adc_array_convert(raw_adc_data,converted_adc_data);
		  delay(10);
	  }
	  
}

 /**
  * \@brief This function configures PINS for the MCU
  * 
  * \@param 
  * 
  * \@return void
  */
 void pin_setup(void){
  
  pinMode(ctrl_vstr,OUTPUT);
   //the second point says to apply 0V from the DAC and use the trimmer so that -0.5V is visible on the output. But the DAC should be set to 3.3V because CTRL_VSTR and VSTR are inversely related
  analogWrite(ctrl_vstr,1023); //set to 1023, 2.2V is the max output we can get
  
  pinMode(heater_pwm,OUTPUT);
  analogWrite(heater_pwm,0);
  pinMode(helmholtz_pwm,OUTPUT);
  analogWrite(helmholtz_pwm,0);
  pinMode(_reset_adc,OUTPUT); //active low
  digitalWrite(_reset_adc,HIGH); //setting it off
  
  pinMode(_pwdn_adc,OUTPUT); //active low
  digitalWrite(_pwdn_adc,LOW); //setting it off
  
  pinMode(start_adc,OUTPUT); //active high
  digitalWrite(start_adc,LOW); //setting it off
  
  pinMode(_drdy_adc,INPUT);
  pinMode(red_led,OUTPUT);
  pinMode(blue_led,OUTPUT);
  pinMode(clr_freq_divider,OUTPUT);
  pinMode(_cs_adc,OUTPUT);

  //SPI pins are already initialized by Arduino
  pinMode(mosi_adc,OUTPUT);
  pinMode(sck_adc,OUTPUT);
  pinMode(miso_adc,INPUT);
  pinMode(q11_freq_counter,INPUT);
  pinMode(q12_freq_counter,INPUT);
  pinMode(q13_freq_counter,INPUT);
  pinMode(q14_freq_counter,INPUT);
  
  return;
}

/**
 * \brief This function resets the ADC
 * 
 * \param 
 * 
 * \return void
 */
void adc_reset(void){
	digitalWrite(_reset_adc,LOW);
	delay(150);
	digitalWrite(_reset_adc,HIGH);
	delay(500); //delay for ADC startup time
}


/**
 * \@brief This is just a random function that has various tests for the ADC 
 * 
 * 
 * \@return void
 */
void testing_suite(){
// 	  CONFIG0_value = adc_register_read(0x00);
// 	  CONFIG1_value = adc_register_read(0x01);
// 	  MUXSCH_value  = adc_register_read(0x02);
// 	  MUXDIF_value  = adc_register_read(0x03);
// 	  MUXSG0_value  = adc_register_read(0x04);
// 	  MUXSG1_value  = adc_register_read(0x05);
// 	  SYSRED_value  = adc_register_read(0x06);
// 	  GPIOC_value  = adc_register_read(0x07);
// 	  GPIOD_value  = adc_register_read(0x08);
// 
// 	  //Testing pins
// 	  if (digitalRead(_drdy_adc)== HIGH){
// 		  Serial.println("_DRDY_ADC is HIGH");
// 	  }
// 	  else {
// 		  Serial.println("_DRDY_ADC is LOW");
// 	  }
// 	  
// 	  Serial.print("CONFIG0 :");
// 	  Serial.println(CONFIG0_value );
// 
// 	  Serial.print("CONFIG1 :");
// 	  Serial.println(CONFIG1_value );
// 
// 	  Serial.print("MUXSCH :");
// 	  Serial.println(MUXSCH_value );
// 
// 	  Serial.print("MUXDIF :");
// 	  Serial.println(MUXDIF_value );
// 
// 	  Serial.print("MUXSG0 :");
// 	  Serial.println(MUXSG0_value );
// 
// 	  Serial.print("MUXSG1 :");
// 	  Serial.println(MUXSG1_value );
// 
// 	  Serial.print("SYSRED :");
// 	  Serial.println(SYSRED_value );
// 
// 	  Serial.print("GPIOC :");
// 	  Serial.println(GPIOC_value );
// 
// 	  Serial.print("GPIOD :");
// 	  Serial.println(GPIOD_value );
// 	  
// 	  //testing two's complement
// 	  Serial.println("Should be -1 : ");
// 	  Serial.println(twos_complement_to_int(0XFFFF,16));
// 	  Serial.println("Should be -345 : ");
// 	  Serial.println(twos_complement_to_int(0XFEA7,16));
// 	  Serial.println("Should be -139 : ");
// 	  Serial.println(twos_complement_to_int(0XFF75,16));
// 	  Serial.println("Should be 567 : ");
// 	  Serial.println(twos_complement_to_int(0X0237,16));
// 	  Serial.println("Should be 120 : ");
// 	  Serial.println(twos_complement_to_int(0X0078,16));
// 	  Serial.println("Should be 0 : ");
// 	  Serial.println(twos_complement_to_int(0X00,16));
// 	  
// 	  /*testing one value converting it*/
// 	  uint32_t Channel_data = adc_channel_read_register_format();
// 	  uint8_t Status_byte = adc_return_status_byte(Channel_data);
// 	  int CHID = adc_chid_status(Status_byte);
// 	  uint16_t Raw_data = adc_return_raw_data(Channel_data);
// 	  int converted_data = adc_mv(twos_complement_to_int(Raw_data,16), converted_adc_data[28], converted_adc_data[27]);
// 	  
// 
// 	  Serial.print("CHID :");
// 	  Serial.println(adc_chid_status(Status_byte));
// 	  Serial.print("NEW BIT :");
// 	  Serial.println(adc_new_status_bit(Status_byte));
// 	  Serial.print("OVF :");
// 	  Serial.println(adc_ovf_status_bit(Status_byte));
// 	  Serial.print("SUPPLY :");
// 	  Serial.println(adc_supply_status_bit(Status_byte));
// 	  Serial.print("Raw data = ");
// 	  Serial.println(Raw_data);
// 	  Serial.print("Converted data in mV = ");
// 	  Serial.println(converted_data);
	  
	  //Testing the DAC
	  //Heater
	  // analogWrite(HEATER_PWM, 1023);
	  // ADC_Auto_Scan();
	  //
	  // for (int i =0; i<29; i++){
	  // 	Serial.print("Priority : ");
	  // 	Serial.print(i);
	  //
	  // 	Serial.print(" : Value :");
	  // 	Serial.println(converted_ADC_data[i]);
	  // }
	  // Serial.println();
	  // delay(2000);
	  
	  // 	  for (int j = 0; j < ADC_Register_Array_Size; j++){
	  // 		  int data = ADC_RegisterRead(ADC_Register_Addresses[j]);
	  // 		  Serial.print("Reg Address : ");
	  // 		  Serial.print(ADC_Register_Addresses[j],HEX);
	  // 		  Serial.print(" Reg Value : ");
	  // 		  Serial.println(data,HEX);
	  // 		  if ( data!= ADC_Register_Defaults[j]){
	  // 			  Serial.println("error");
	  // 		  }
	  // 	  }
	  //
	  // 	  Serial.println();
	  
// 	  analogWriteResolution(10); //the second point says to apply 0V from the DAC and use the trimmer so that -0.5V is visible on the output. But the DAC should be set to 3.3V because CTRL_VSTR and VSTR are inversely related
// 	  
	//testing adc_auto_scan, adc_Drate and adc_initial_delay
// 	adc_register_write(CONFIG1_address, 0X03);
// 	Serial.print("For DRATE : ");
// 	Serial.print( adc_drate() );
// 	Serial.print(" , delay, uS = ");
// 	Serial.print( adc_initial_delay_time(),4 );
// 	Serial.println("_________________\n ");
// 
// 	adc_register_write(CONFIG1_address, 0X01);
// 	Serial.print("For DRATE : ");
// 	Serial.print( adc_drate() );
// 	Serial.print(" , delay, uS = ");
// 	Serial.print( adc_initial_delay_time(),4 );
// 	Serial.println("_________________\n ");
// 		
// 	adc_register_write(CONFIG1_address, 0X02);
// 	Serial.print("For DRATE : ");
// 	Serial.print( adc_drate() );
// 	Serial.print(" , delay, uS = ");
// 	Serial.print( adc_initial_delay_time(),4 );
// 	Serial.println("_________________\n ");
// 		
// 		
// 	adc_register_write(CONFIG1_address, 0X00);
// 	Serial.print("For DRATE : ");
// 	Serial.print( adc_drate() );
// 	Serial.print(" , delay, uS = ");
// 	Serial.print( adc_initial_delay_time(),4 );
// 	Serial.println("_________________\n ");
// 
// 	adc_auto_scan(raw_adc_data);
// 	adc_array_convert(raw_adc_data,converted_adc_data);
// 		
// 	for (int i = 8; i<29; i++)
// 	{
// 		Serial.print("ADC priority bit :");
// 		Serial.print(i+1);
// 		Serial.print(" = ");
// 		Serial.println(converted_adc_data[i], 4);
// 	}
// 		
// 	Serial.println("__________________________________  \n");
// 		
// 	delay(1000);
		
		
/************************************************************************/
/* Testing adc_gpio_high and adc_gpio_low functions                     */
/************************************************************************/
		
// 	for (int i=0;i<8;i++)
// 	{
// 			Serial.print("Setting GPIO :");
// 			Serial.print(i);
// 			Serial.println("HIGH");
//
// 			adc_gpio_high(i);
//
// 			Serial.print("GPIOC value :");
// 			Serial.println(adc_register_read(GPIOC_address),BIN);
//
// 			Serial.print("GPIOD value :");
// 			Serial.println(adc_register_read(GPIOD_address),BIN);
//
// 			delay(5000);
// 	}
//
// 		for (int i=0;i<8;i++)
// 		{
// 			Serial.print("Setting GPIO :");
// 			Serial.print(i);
// 			Serial.println(" LOW ");
//
// 			adc_gpio_low(i);
//
// 			Serial.print("GPIOC value :");
// 			Serial.println(adc_register_read(GPIOC_address),BIN);
//
// 			Serial.print("GPIOD value :");
// 			Serial.println(adc_register_read(GPIOD_address),BIN);
//
// 			delay(5000);
// 		}

}

/**
 * \@brief This function returns the STATUS byte from the Channel Data register read
 * 
 * \@param RAW_CHANNEL_REGISTER_READ_DATA, -> 32bit Value from SPI reading, this 32bit value is from "ADCchannelRead_registerFormat" function
 * 
 * \@return uint8_t -> 8bit value of status byte
 */
uint8_t adc_return_status_byte(uint32_t raw_channel_register_read_data){
	//The input 32bit value contains [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
	//We return status
	return (uint8_t)((raw_channel_register_read_data >> 16)& 0x000000FF); //0X000F = 0b0000_0000_0000_0000_0000_0000_1111_1111, this returns the status byte
}

/**
 * \@brief This function returns the RAW data(MSB+LSB) byte from the Channel Data register read
 * 
 * \@param RAW_CHANNEL_REGISTER_READ_DATA -> 32bit Value from SPI reading, this 32bit value is from "ADCchannelRead_registerFormat" function
 * 
 * \@return uint16_t -> 16bit value of Raw ADC data
 */
uint16_t adc_return_raw_data(uint32_t raw_channel_register_read_data){
	//The input 32bit value contains [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
	//We return MSB and LSB
	return (uint16_t)((raw_channel_register_read_data )& 0x0000FFFF); //0X0F = 0b0000_0000_0000_0000_1111_1111_1111_1111, this returns the status byte
}

/**
 * \@brief  Reads all the ADC channels selected in configuration settings(Registers MUXDIF,MUXSG0,MUXSG1) and stores the RAW data in the ADC_raw_data_array, the Channel ID is the corresponding index
 *          Methodology
 *			Read all values from the ADC
 *
 *			Method
 *			Count number of loops
 *			Scan through the channel ID's
 *			if new bit is set and OVF+SUPPLY bits are not set, store the raw value
 *			The CHID is the Array index
 *
 *			For example, if channels 3, 4, 7, and 8 are selected for measurement in the list, the ADS1158 converts
 *			the channels in that order, skipping all other channels. After channel 8 is converted,
 *			starts over, beginning at the top of the channel list, circuitry are completely disabled. channel 3
 * 
 * \@return void
 */
void adc_auto_scan(uint16_t raw_data_array[]){

	float switch_time_delay = adc_initial_delay_time();
	int count = count_set_bits(MUXDIF_default) + count_set_bits(MUXSG0_default) + count_set_bits(MUXSG1_default); //this is the number of loops to make, by measuring 
	for (int p = 0; p < count; p++){
		uint32_t Channel_data = adc_channel_read_register_format();
		uint8_t Status_byte = adc_return_status_byte(Channel_data);
		int CHID = adc_chid_status(Status_byte);
		uint16_t Raw_data = adc_return_raw_data(Channel_data);
	
		if ((adc_new_status_bit(Status_byte) == true) && (adc_supply_status_bit(Status_byte)== false) && (adc_ovf_status_bit(Status_byte) == false)) {
			//store the data in the array
			if (CHID > 0x19){ //CHID skips address 0x19h according to data sheet
				raw_data_array[CHID-1] = Raw_data; 
			}
			else {
				raw_data_array[CHID] = Raw_data; //store the value
			}
		}
		else {
			//try to report error
			if (adc_supply_status_bit(Status_byte)) {//Serial.println("Error: ADC Status SUPPLY bit set");}
				//Raise MCU error
				String error_text = "Error: ADC Status SUPPLY bit set";
				raise_mcu_error(error_text);
				}
			else if (adc_ovf_status_bit(Status_byte)) {//Serial.println("Error: ADC Status OVF bit set");}
				String error_text = "Error: ADC Status OVF bit set";
				raise_mcu_error(error_text);
				}
		}
	
		delayMicroseconds(switch_time_delay);
	}
}

/**
 * \@brief Function to get no of set bits in binary representation of passed binary no. 
 * 
 * \@param n -> number from which to count the bits
 * 
 * \@return unsigned int -> representing the number of set bits in the number
 */
unsigned int count_set_bits(int n){
	unsigned int count = 0;
	while (n) {
		n &= (n - 1);
		count++;
	}
	return count;
} 

/**
 * \@brief Toggles the start pin on the ADC
 * 
 * \@param void
 * 
 * \@return void
 */
void adc_toggle_start_pin(void){
	digitalWrite(start_adc,HIGH); //starts conversion
	delayMicroseconds(2); //wait for data to settle
	digitalWrite(start_adc,LOW); //stops conversion so that we can go to the next Channel ID, Check channel ID in Table 10 of the ADC manual
}

/**
 * \@brief Returns an integer value of a twos complement number
 * 
 * \@param value
 * \@param num_bits
 * 
 * \@return int
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

/**
 * \@brief converts ADC value into millivolts
 *         mV = ADC_data * range
 *				----------------
 *				Gain * ADC_Resolution 
 *
 * \@see "http://scientific-solutions.com/products/faq/ssi_faq_adc_equations%20VER2.shtml#:~:text=2%27s%20Complement%20ADC%20Data%20with%208%2Dbit%20resolution"
 * \@param ADC_reading
 * 
 * \@return int millivolts of ADC value
 */
double adc_mv(int ADC_reading, double vref, double gain){
	
	if ((vref == NULL) || (vref == 0)) vref =  ADC_VREF;
	if (gain == NULL || (gain == 0)) gain = ADC_GAIN;
	//return (ADC_reading*converted_adc_data[28]*1000) / (double) ((double)30720  * converted_adc_data[27]);
	return (((double)ADC_reading*vref*1000) /( (double) 30720.00 * gain) );
}


/**
 * \@brief Converts raw_ADC_Data into converted_ADC_data 
 * 
 * \@param 
 * 
 * \@return void
 */
void adc_array_convert(uint16_t raw_data[], double converted_data[]){
	//both converted_data and raw_data have sizes of 9
	//ADC GAIN conversion
	converted_data[27] = ((float) raw_data[27]/(double) 30720);
	//ADC REF conversion
	converted_data[28] = ((float) raw_data[28]/(double) 3072);
	// 
	//for OFFSET conversion
	converted_data[24] = raw_data[24]; //Ideally, the code from this register function is 0h, but varies because of the noise of the ADC and offsets stemming from the ADC and
	//external signal conditioning. This register can be used to calibrate or track the offset of the ADS1158 and Figure 40. Conversion Control, Auto-Scan Mode external signal conditioning.
	//for VCC conversion
	converted_data[25] = ((float) raw_data[25]/(double) 3072);
	//for ADC temp conversion
	converted_data[26] =( ((double) ( (1000*adc_mv(twos_complement_to_int(raw_data[26],number_of_bits_adc),converted_adc_data[28], converted_adc_data[27])) - 168000)/(double) ADC_temp_sensor_coefficient)+ 25);

	for (int i =0; i <= 23; i++) //for loop is at the bottom so that we can first convert useful constants used in these calculations
	{
		converted_data[i] = adc_mv( (twos_complement_to_int(raw_data[i],number_of_bits_adc) - twos_complement_to_int(raw_data[24],number_of_bits_adc)),converted_data[28], converted_data[27]); //Vin = (Vref/GainError) * [ (OutputCode - OffsetCode) / 0x7800 ]
	}
	
	return;
}

/**
 * \brief Returns the ADC data rate
 * 
 * \param 
 * 
 * \return int the DRATE, either 11,10,01,00
 */
uint8_t adc_drate(void){
	
	return ( 0x03 & adc_register_read(CONFIG1_address));
}

/**
 * \brief returns delay time for sleep mode of the ADC with the recommended initial delay time for the ADC, see Table 8 in ADS1158 document
 * \conditions Chop = 0 and DLY[2:0] = 000
 *													Table 8. 
 *								Start Condition to DRDY Delay, Chop = 0, DLY[2:0] = 000
 *
 *							INITIAL DELAY (Standby Mode)			INITIAL DELAY (Sleep Mode)
 *								(fCLK cycles)							(fCLK cycles)
 *
 *		DRATE[1:0]		   Fixed-Channel	Auto-Scan					Fixed-Channel		Auto-Scan
 *			11					802				708							866					772
 *			10					1186			1092						1250				1156
 *			01					2722			2628						2786				2692
 *			00					8866			8772						8930				8836
 * 
 * 
 * \return uint16_t
 */
float adc_initial_delay_time(void){
	
	float delay_time = 0;
	
		switch( adc_drate() ){ // get the adc DRATE
			case 0b00: {
				delay_time = ( (8772.00 * 1000000)/ADC_CLK_SPEED );
				break;
			}
			case 0b01: {
				delay_time = ( (2628.00 * 1000000)/ADC_CLK_SPEED );
				break;
			}
			case 0b10: {
				delay_time = ( (1092.00 * 1000000)/ADC_CLK_SPEED );
				break;
			}
			case 0b11: {
				delay_time = ( (708.00 * 1000000)/ADC_CLK_SPEED );
				break;
			}

			default: delay_time = ( (8772.00 * 1000000)/ADC_CLK_SPEED );
		}
	
	return delay_time;
	}
	

/**
 * \brief  sets a GPIO on the ADC HIGH
 * 
 * \param gpio_number 
 * 
 * \return void
 */
void adc_gpio_high (int gpio_number){
	//Make sure it is an output (0) 
	uint8_t gpioc_current_value = adc_register_read(GPIOC_address);
	//clear bit at GPIO number position to set as output
	gpioc_current_value &= (~(1 << gpio_number));
	
	adc_register_write(GPIOC_address, gpioc_current_value); //write new value to ADC register
	
	//make it high, by setting to one(|) to GPIOD register
	
	uint8_t gpiod_current_value = adc_register_read(GPIOD_address);
	//set bit at GPIO number position
	gpiod_current_value |= (1 << gpio_number);
	
	adc_register_write(GPIOD_address, gpiod_current_value); //write new value to ADC register
	
	
	
}

/**
 * \brief  sets a GPIO on the ADC LOW
 * 
 * \param gpio_number 
 * 
 * \return void
 */
void adc_gpio_low (int gpio_number){
	//ensure it is an output (0)
	
	uint8_t gpioc_current_value = adc_register_read(GPIOC_address);
	//clear bit at GPIO number positioto set as outputn 
	gpioc_current_value &= (~(1 << gpio_number));
		
	adc_register_write(GPIOC_address, gpioc_current_value); //write new value to ADC register
	
	//make GPIO low, by setting the corresponding bit to zero(&)
	uint8_t gpiod_current_value = adc_register_read(GPIOD_address);
	//clear bit at GPIO number position
	gpiod_current_value &= (~(1 << gpio_number));
		
	adc_register_write(GPIOD_address, gpiod_current_value); //write new value to ADC register
	
}


/************************************************************************/
/* INITIALIZATION FUNCTIONS                                             */
/************************************************************************/

/**
* \@brief Empty function for debugging
* 
* \@param
*/
void pass (void){
	return;
}

/**
 * \@brief Function to receive test instructions from CPU 
 * 
 * \@param 
 * 
 * \@return void
 */
void receive_test_instructions(void){
	String receieved_instruction;
	String instruction = "instruction";  //This will be a json instruction in string format from the CPU Python script
	
	while (Serial.available()==0){ //wait for something at serial
	}
	receieved_instruction = Serial.readStringUntil('\n'); //read till end of the line

	while (Serial.available()==0){ //wait for instruction json at serial
		if (receieved_instruction.equals(instruction)){
			Serial.println("ready");
		}
	}
	
	receieved_instruction = Serial.readStringUntil('\n'); //
	Serial.println(receieved_instruction); //send json back as string to to check if it is correct
	
	//change string to char array for JSON buffer simplicity
	char charBuf[receieved_instruction.length() + 1];
	receieved_instruction.toCharArray(charBuf, receieved_instruction.length()+1);

	// De serialize the JSON document
	DeserializationError error = deserializeJson(doc, charBuf);

	// Test if parsing succeeds.
	if (error) {
		//raise error
		raise_mcu_error(error.c_str());
		return;
	}

	test_id = doc["id"]; // 0
	const char* description = doc["description"]; // "Apply required stress for 2 hours, etcetera"

	//Setting test parameters
	JsonObject test_values = doc["test_values"];
	desired_temperature = test_values["temperature"]; // 120
	desired_fpga_voltage = test_values["v_stress"]; // -400
	desired_time_for_test = test_values["test_time"]; // 5
	desired_magnetic_field = test_values["magnetic_field"]; // 5
	if (test_values["Test_start"] == 1) test_start= true; // 1
	if (test_values["Test_stop"]==1) test_stop = true; // 0
	serial_output_rate = test_values["serial_rate"]; // 1500

	JsonObject measurement_params = doc["measurement_params"];

	const char* measurement_params_temperature_unit = measurement_params["temperature"]["unit"]; // "C"
	const char* measurement_params_v_stress_unit = measurement_params["v_stress"]["unit"]; // "mV"
	const char* measurement_params_test_time_unit = measurement_params["test_time"]["unit"]; // "seconds"
	const char* measurement_params_magnetic_field_unit = measurement_params["magnetic_field"]["unit"]; // "mT"
	const char* measurement_params_serial_rate_unit = measurement_params["serial_rate"]["unit"]; // "milliseconds"
	
}

/**
 * \@brief  Function to send data to Python script
 *			tell python there is ready data
 *			send the data
 *			wait for confirmation
 * 
 * 
 * \@return void
 */
void send_data_to_serial(){

	char userInput;
// 	if(Serial.available()> 0){
// 		userInput = Serial.read();               // read user input
// 		if(userInput == 'g'){                  // if we get expected value
// 			//printing the document
// 			serializeJson(doc, Serial);
// 			Serial.println();
// 
// 			while (Serial.available() == 0){
// 				continue; //wait for confirmation
// 			}
// 			while (Serial.read() != 'd'){
// 				continue;
// 			}
// 			Serial.println("done");
// 		}
// 	}
			Serial.begin(BAUD_RATE); //9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000
			//printing the document
			serializeJson(doc, Serial);
			Serial.flush();
			Serial.write("\n");
			Serial.flush();
			Serial.end();

// 			while (Serial.available() == 0){
// 				continue; //wait for confirmation
// 			}
// 			while (Serial.read() != 'd'){
// 				continue;
// 			}
// 			Serial.println("done");
}


/**
 * \@brief Sets up 1MHz clock used by TC3,TC4,TCC2,TC5
 * 
 * 
 * \@return void
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
	GCLK_CLKCTRL_ID_TCC2_TC3; // Feed the GCLK4 to TCC2 and TC3
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	return;
}


/**
 * \@brief Sets up TC4 timer
 * 
 * 
 * \@return void
 */
void init_tc4(){ //initialize TC4 timer
	// Configure TC4 (16 bit counter by default)
	REG_TC4_COUNT16_CC0 = counter_value(1,256,200);  //set period for timer
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
 * \@brief Sets up TC5 timer
 * 
 * 
 * \@return void
 */
void init_tc5(){ //initialize T5 timer

	REG_TC5_COUNT16_CC0 =  counter_value(1,64,1000);                     // Set the TC4 CC1 register to  decimal 3905, 1 sec
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
 * \@brief Sets up TC3 timer
 * 
 * 
 * \@return void
 */
void init_tc3(){ //initialize TC3 timer, this timer controls the rate at which serial data is sent
	
	REG_TC3_COUNT16_CC0 =  counter_value(1,1024,serial_output_rate);                     // Set the TC3 CC0 register
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
 * \@brief Interrupt service routine for TC4, handles the ADC interrupt
 * 
 * 
 * \@return void
 */
void TC4_Handler()// ADC interrupt handler
{
	// Check for OVF interrupt
	if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)
	{
		//Overflow interrupt code here:
		//ADC read code, set a flag

		// Clear the interrupt flag
		REG_TC4_INTFLAG = TC_INTFLAG_OVF;
	}
	return;
}

/**
 * \@brief TC5 timer ISR 
 * 
 * 
 * \@return void
 */
void TC5_Handler(){ //counter interrupt
	// Check for OVF interrupt
	if (TC5->COUNT16.INTFLAG.bit.OVF && TC5->COUNT16.INTENSET.bit.OVF)
	{
		//Overflow interrupt code here:
		test_time_count++; //increment test count
		
// 		if ((test_time_count*1000)%serial_output_rate == 0){
// 			serial_signal = true;
// 		}
		
		REG_TC5_INTFLAG = TC_INTFLAG_OVF;        // Clear the MC0 interrupt flag
	}
	return;
}

/**
 * \@brief TC3 timer ISR
 * 
 * 
 * \@return void
 */
void TC3_Handler(){//TC3 NOT USED	
	// Check for match counter 1 (MC1) interrupt
	if (TC3->COUNT16.INTFLAG.bit.OVF && TC3->COUNT16.INTENSET.bit.OVF)
	{
		serial_signal = true;
		REG_TC3_INTFLAG = TC_INTFLAG_OVF;        // Clear the OVF interrupt flag
	}
	return;
}

/**
 * \@brief Returns value for the overflow counter
 * 
 * \@param Clock_FrequencyMHz
 * \@param prescaler
 * \@param period_ms
 * 
 * \@return int
 */
int counter_value (float clock_frequency_MHz,float prescaler, float period_ms){
	return (int) (( (clock_frequency_MHz*1000*period_ms) / (prescaler) )-1) ;
	
}


/**
 * \@brief   Function converts millivolts from the temperature sensor output to temperature in Celsius
 *			Formula : T = (V_out - V_offset)/Tc) + T_INFL
 * 
 * \@see Data sheet: "https://www.ti.com/lit/ds/symlink/tmp235.pdf?ts=1594142817615&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMP235"
 *
 * \@param milli_volts -> Digital Voltage from ADC
 * 
 * \@return float -> Temperature conversion in C from TMP235
 */
float millivolt_to_celcius(float milli_volts)
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


/**
 * \@brief  Converts voltage to corresponding magnetic field (reference is for the  magnetic sensor)
 *
 * \@see Data sheet: "https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjxps3qg8PqAhXBo54KHUn5CvwQFjAAegQIBRAB&url=https%3A%2F%2Fwww.allegromicro.com%2F~%2Fmedia%2FFiles%2FDatasheets%2FA1318-A1319-Datasheet.ashx&usg=AOvVaw39zGCju7QuDLgpcH9PKde_"
 * 
 * \@param milli_volts
 * 
 * \@return float  -> Magnetic field density in milli_Tesla (1G = 0.1mT)
 */
float millivolts_to_milliTesla(double milli_volts){
	#define QUIESCENT_VOLTAGE 1650              //1.65V Low->1.638, mean-> 1.65, high-> 1.662 measure this
	#define MAGNETIC_SENSITIVITY 13.5           //1.35mv/G --> 13.5mV/mT Low->1.289, mean-> 1.3, high-> 1.411
	#define TEMPERATURE_SENSITIVITY   0.12      //0.12%/C
	#define magnetic_sensor_error 1.5							//1.5% ERROR
	#define VOLTAGE_CLAMP_HIGH 2970			    //2.97V +40 mT
	#define VOLTAGE_CLAMP_LOW 0.33			    //2.97V -40mT
	#define MAX_MAGNETIC_FIELD 40              //40mT
	#define MIN_MAGNETIC_FIELD -40              //-40mT
	
	if (milli_volts == '\0') return '\0';
	else if (milli_volts == VOLTAGE_CLAMP_HIGH ) return MAX_MAGNETIC_FIELD ; //we can find some sort of error to throw here
	else if (milli_volts == VOLTAGE_CLAMP_LOW) return MIN_MAGNETIC_FIELD ;
	else return (double)((milli_volts-QUIESCENT_VOLTAGE)/(float)MAGNETIC_SENSITIVITY);   //polarity depends on the direction of the field
	
}

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
			if (test_stop){
				//clear and turn off all the outputs
				pin_setup(); //this will reset all the pins to their original values;
				update_json_doc(test_id,test_stop,test_start,test_error,error_message,converted_adc_data,test_time_count,measured_temperature,measured_magnetic_field);
				
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
				
				//2. Setting time interval for sending data rate
				
				//Temporarily changed to counter file
// 				REG_TC3_COUNT16_CC0 =  counter_value(1,1024,serial_output_rate);                     // Set the TC3 CC0 register
// 				while (TC3->COUNT16.STATUS.bit.SYNCBUSY);											// Wait for synchronization
			}
			
		}
		break;
		case ADC_UPDATE: {

			//1. Update test timer and if time is hit, stop
			if (test_time_count >= desired_time_for_test*60) test_stop=true;
			
			//2. Convert Raw ADC data to understandable values
			adc_auto_scan(raw_adc_data); //converting ADC data
			adc_array_convert(raw_adc_data,converted_adc_data); //converts the raw_adc_data into converted data
			
		}
		break;
		case SERIAL_UPDATE:{
			//1. Set Data to be sent to the user from the ADC update, data is sent in intervals in an Interrupt service routine
			update_json_doc(test_id,test_stop,test_start,test_error,error_message,converted_adc_data,test_time_count,measured_temperature,measured_magnetic_field);
			
			if (serial_signal) {
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
			next_state = Heater_Update;
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
 * \@brief This function prepares the JSON document that is to be sent to the serial
 * 
 * \@param test_id
 * \@param test_stop
 * \@param test_start
 * \@param test_error
 * \@param error_message
 * \@param adc_data
 * \@param test_time
 * \@param temperature
 * \@param measured_magnetic_field
 * 
 * \@return void
 */
void update_json_doc(int test_id, bool test_stop, bool test_start, bool test_error, String error_message, 
double adc_data[], float test_time, float temperature, float magnetic_field){
	//Preparing json file
	String str_testid = "test id";
	String str_teststop = "test stop";
	String str_testeror = "test error";
	String str_errormessage = "error message";
	String str_ADCdatastring = "ADC data";
	String str_testdata = "test data";
				
	doc.clear(); //clear the document as this frees up the memory
	// Add values to the document

	
	doc[str_testid] = test_id;
	//doc["test stop"] = TEST_STOP; //1 test is stopped, 0 test is running
	if (test_stop) doc[str_teststop] = 1;
	else doc[str_teststop] = 0;
	//doc["test error"] = TEST_ERROR; //0 no error, 1 there was an error
	if (test_error) doc[str_testeror]=1;
	else doc[str_testeror]=0;
				
	doc[str_errormessage] = error_message;
	doc["number of adc arrays"] = number_of_adc_arrays;
	
	for (int i=0;i<number_of_adc_arrays;i++){
		//send multiple arrays in adc doc
		str_ADCdatastring.concat(String(i));
		str_testdata.concat(String(i));
			
		// Add an array.
		JsonArray ADCdata = doc.createNestedArray(str_ADCdatastring);
		for (int i =8; i < 29; i++){ //starting from index 8 as that is how much we use in the ADC array, this is to save space
			ADCdata.add(adc_data[i]);
		}
		//add another array
		JsonArray TESTdata = doc.createNestedArray(str_testdata);

		TESTdata.add(test_time); //current test time
		TESTdata.add(temperature); //temperature (C)
		TESTdata.add(magnetic_field); //magnetic field (mT)
		//out current_test_timer, ADC converted data at intervals
		
		str_testid.remove(str_testid.lastIndexOf(String(i)));
		str_teststop.remove(str_teststop.lastIndexOf(String(i)));
		str_testeror.remove(str_testeror.lastIndexOf(String(i)));
		str_errormessage.remove(str_errormessage.lastIndexOf(String(i)));
		str_ADCdatastring.remove(str_ADCdatastring.lastIndexOf(String(i)));
		str_testdata.remove(str_testdata.lastIndexOf(String(i)));
	}
	
	return;
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
/* DAC FUNCTIONS                                                        */
/************************************************************************/

/**
 * \@brief This function converts a user entered voltage value into a 10 bit DAC value
 * 
 * \@param volt -> desired voltage in mini_volts
 * 
 * \@return int -> DAC value
 */
int set_dac(float volt) {
	//formula for calculating DAC output voltage Vdac = (dVal / 1023)*2.23V
	return (int)((volt*1023)/2230);
}

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