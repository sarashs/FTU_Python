/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include <SPI.h>

#define  REDLED A1
#define  BLUELED A2
#define  YELLOWLED A5
#define  ANALOG_PIN A6
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

#define ADC_SPI_SPEED 6000000  //6MHZ -> Must be less than 0.5 adc_clk(15.7MHz)

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
volatile double converted_ADC_data[29] = {0}; //this array hold the converted ADC data	
//initialize some values to avoid errors in calculations

float dac_data [35] = {0};
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

void setup() {
  pin_setup(); //Setup MCU pins
  Serial.begin(9600);
  delay(1000);
  ADC_Setup(); //function sets up the ADC
}

void loop() {
  // put your main code here, to run repeatedly:
// 	float counter = 3300; //initial voltage
// 	int i = 0;
// 	
// 	while (counter > 0){
// 		
// 		analogWrite(CTRL_VSTR, setDAC(counter/1000));
// 		ADC_Auto_Scan();
// 		ADC_array_convert();
// 		dac_data[i] = converted_ADC_data[12];
// 		
// 		//printing
// 		Serial.print("Loop :");
// 		Serial.println(i);
// 		
// 		Serial.print(converted_ADC_data[12],5);
// 		Serial.println(" : mV")
// 		
// 		i++;
// 		counter -= 100;
// 		delay(3000);
// 	}
	
	
	
	ADC_Auto_Scan();
	ADC_array_convert();
	for (int j = 0; j<29; j++){
		Serial.print("RAW CHID ");
		Serial.print(j);
		Serial.print(" : ");
		Serial.println(raw_ADC_data[j]);
	}
    for (int j = 0; j<29; j++){
	    Serial.print("Converted Priority ");
	    Serial.print(j+1);
	    Serial.print(" : ");
	    Serial.print(converted_ADC_data[j],4);
		Serial.println();

    }

		delay(1500);
}
//this function converts a user entered voltage value into a 10 bit DAC value
int setDAC(float volt) {
	//formula for calculating DAC output voltage Vdac = (dVal / 1023)*3.3V
	return (int)((volt*1023)/3.3);
}


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
  digitalWrite(CTRL_VSTR,HIGH); //the second point says to apply 0V from the DAC and use the trimmer so that -0.5V is visible on the output. But the DAC should be set to 3.3V because CTRL_VSTR and VSTR are inversely related
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
void testing_suite(){
	  CONFIG0_value = ADC_RegisterRead(0x00);
	  CONFIG1_value = ADC_RegisterRead(0x01);
	  MUXSCH_value  = ADC_RegisterRead(0x02);
	  MUXDIF_value  = ADC_RegisterRead(0x03);
	  MUXSG0_value  = ADC_RegisterRead(0x04);
	  MUXSG1_value  = ADC_RegisterRead(0x05);
	  SYSRED_value  = ADC_RegisterRead(0x06);
	  GPIOC_value  = ADC_RegisterRead(0x07);
	  GPIOD_value  = ADC_RegisterRead(0x08);

	  //Testing pins
	  if (digitalRead(_DRDY_ADC)== HIGH){
		  Serial.println("_DRDY_ADC is HIGH");
	  }
	  else {
		  Serial.println("_DRDY_ADC is LOW");
	  }
	  
	  Serial.print("CONFIG0 :");
	  Serial.println(CONFIG0_value );

	  Serial.print("CONFIG1 :");
	  Serial.println(CONFIG1_value );

	  Serial.print("MUXSCH :");
	  Serial.println(MUXSCH_value );

	  Serial.print("MUXDIF :");
	  Serial.println(MUXDIF_value );

	  Serial.print("MUXSG0 :");
	  Serial.println(MUXSG0_value );

	  Serial.print("MUXSG1 :");
	  Serial.println(MUXSG1_value );

	  Serial.print("SYSRED :");
	  Serial.println(SYSRED_value );

	  Serial.print("GPIOC :");
	  Serial.println(GPIOC_value );

	  Serial.print("GPIOD :");
	  Serial.println(GPIOD_value );
	  
	  //testing two's complement
	  Serial.println("Should be -1 : ");
	  Serial.println(twos_complement_to_int(0XFFFF,16));
	  Serial.println("Should be -345 : ");
	  Serial.println(twos_complement_to_int(0XFEA7,16));
	  Serial.println("Should be -139 : ");
	  Serial.println(twos_complement_to_int(0XFF75,16));
	  Serial.println("Should be 567 : ");
	  Serial.println(twos_complement_to_int(0X0237,16));
	  Serial.println("Should be 120 : ");
	  Serial.println(twos_complement_to_int(0X0078,16));
	  Serial.println("Should be 0 : ");
	  Serial.println(twos_complement_to_int(0X00,16));
	  
	  /*testing one value converting it*/
	  uint32_t Channel_data = ADCchannelRead_registerFormat();
	  uint8_t Status_byte = ADC_RETURN_STATUSBYTE(Channel_data);
	  int CHID = ADC_CHID_STATUS(Status_byte);
	  uint16_t Raw_data = ADC_RETURN_RAWDATA(Channel_data);
	  int converted_data = ADC_mv(twos_complement_to_int(Raw_data,16));
	  

	  Serial.print("CHID :");
	  Serial.println(ADC_CHID_STATUS(Status_byte));
	  Serial.print("NEW BIT :");
	  Serial.println(ADC_NEW_STATUS_BIT(Status_byte));
	  Serial.print("OVF :");
	  Serial.println(ADC_OVF_STATUS_BIT(Status_byte));
	  Serial.print("SUPPLY :");
	  Serial.println(ADC_SUPPLY_STATUS_BIT(Status_byte));
	  Serial.print("Raw data = ");
	  Serial.println(Raw_data);
	  Serial.print("Converted data in mV = ");
	  Serial.println(converted_data);
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

