#pragma once
/**
 * \file adc_code.h
 *
 * \brief This file contains functions used to communicate with the ADC
 *
 * \author Valentine Ssebuyungo 
 *
 * \version Revision: 1.0 
 *
 * \date  2020/12/16 
 *
 * \note ADC data sheet used https://www.ti.com/lit/ds/symlink/ads1158.pdf?ts=1608162703638&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1158 \n
 *Magnetic sensor Data sheet: "https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjxps3qg8PqAhXBo54KHUn5CvwQFjAAegQIBRAB&url=https%3A%2F%2Fwww.allegromicro.com%2F~%2Fmedia%2FFiles%2FDatasheets%2FA1318-A1319-Datasheet.ashx&usg=AOvVaw39zGCju7QuDLgpcH9PKde_" \n
 *Temperature sensor Data sheet: "https://www.ti.com/lit/ds/symlink/tmp235.pdf?ts=1594142817615&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMP235"
 *
 * \details
 *How values are stored in these arrays \n\n
 *The Bits CHID is the Array[index] up to index 24, \n
 *Index 25 is CHID 26 -> OFFSET \n
 *Index 26 is CHID 27 -> Temperature \n
 *Index 27 is CHID 28 -> GAIN \n
 *Index 28 is CHID 29 -> External reference \n \n
 *
 *From Table 10 in data sheet  \n
 *BITS CHID[4:0]          PRIORITY        CHANNEL         DESCRIPTION \n
 *00h                     1 (highest)     DIFF0 (AIN0–AIN1) Differential 0 \n
 *01h                     2               DIFF1 (AIN2–AIN3) Differential 1 \n
 *02h						3				DIFF2 (AIN4–AIN5) Differential 2 \n
 *03h				        4               DIFF3 (AIN6–AIN7) Differential 3 \n
 *04h						5			    DIFF4 (AIN8– AIN9) Differential 4 \n
 *05h						6				DIFF5 (AIN10–AIN11) Differential 5 \n
 *06h						7				DIFF6 (AIN12–AIN13) Differential 6 \n
 *07h						8				DIFF7 (AIN14–AIN15) Differential 7 \n
 *08h						9				AIN0 Single-ended 0 \n
 *09h						10				AIN1 Single-ended 1 \n
 *0Ah						11				AIN2 Single-ended 2 \n
 *0Bh						12				AIN3 Single-ended 3 \n
 *0Ch						13				AIN4 Single-ended 4 \n
 *0Dh						14				AIN5 Single-ended 5 \n
 *0Eh						15				AIN6 Single-ended 6 \n
 *0Fh						16				AIN7 Single-ended 7 \n
 *10h						17				AIN8 Single-ended 8 \n
 *11h						18				AIN9 Single-ended 9 \n
 *12h						19				AIN10 Single-ended 10 \n
 *13h						20				AIN11 Single-ended 11 \n
 *14h						21				AIN12 Single-ended 12 \n
 *15h						22				AIN13 Single-ended 13 \n
 *16h						23				AIN14 Single-ended 14 \n
 *17h						24			    AIN15 Single-ended 15 \n
 *18h						25				OFFSET Offset \n
 *1Ah						26				VCC AVDD – AVSS supplies \n
 *1Bh						27				TEMP Temperature \n
 *1Ch						28				GAIN Gain \n
 *1Dh						29 (lowest)		REF External reference \n
 *
 *
*/
 

/************************************************************************/
/* ADC VARIABLES AND CONSTANTS                                          */
/************************************************************************/
///ADC Register Addresses
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
///ADC clock speed
#define ADC_CLK_SPEED 15700000
/// number of bits of adc data
#define number_of_bits_adc 16
/// size of the array containing ADC register addresses and values
#define adc_register_array_size 9
 ///6MHZ -> Must be less than 0.5 adc_clk(15.7MHz) best if in bunches of 1/2,1/4,1/8 of the ADC clock use (3.925MHz) or (7.85Mhz)
#define adc_spi_speed 3925000 
///563uV if ADS1158 and test PCB temperatures are forced together,
///394uV if ADS1158 temperature is forced and test PCB is in free air
#define ADC_temp_sensor_coefficient 563 
///ADC GPIO pin for Blue led on heater 
#define adc_gpio_blue_led 6
///ADC GPIO pin for Red led on heater 
#define adc_gpio_red_led 7

///For converting ADC raw value to mV
#define MAX_POSITIVE_VOLTAGE 0x7FFF
#define MAX_NEGATIVE_VOLTAGE 0x8000
#define REFERENCE_VOLTAGE_ADC_mV 65536
///2^15 bits as 16th bit is used for 2's complement
#define ADC_RESOLUTION 32768 
///4900mV (-1.6V to 3.3V)
#define ADC_RANGE 4900 
///this is the normal ADC GAIN
#define ADC_GAIN 1 
///3.3V this is VREFP-VREFN
#define ADC_VREF 3.340 

const uint8_t adc_register_addresses[adc_register_array_size] = {CONFIG0_address,CONFIG1_address,MUXSCH_address,
MUXDIF_address,MUXSG0_address,MUXSG1_address,SYSRED_address,GPIOC_address,GPIOD_address};

///Default ADC values to be programmed
#define CONFIG0_default 0x02
#define CONFIG1_default 0x00 //0x00 default
#define MUXSCH_default 0x00
#define MUXDIF_default 0x00
#define MUXSG0_default 0xFF //0xFF
#define MUXSG1_default 0x7F  //	0x7F
#define SYSRED_default 0x3D  //0x3D if you wanna see measurements
#define GPIOC_default 0x00
#define GPIOD_default 0x00

///Default ADC pulse convert command
#define Pulse_convert_command 0x80
///Default ADC reset command
#define Reset_command 0xC0

uint8_t adc_register_defaults[adc_register_array_size] = {CONFIG0_default,CONFIG1_default,MUXSCH_default,
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
/// Size of the array of ADC data
#define ADC_ARRAY_SIZE 29
//ADC global variables
uint16_t raw_adc_data [ADC_ARRAY_SIZE] = {0}; //initialize the array with zero
double converted_adc_data[ADC_ARRAY_SIZE] = {0}; //this array hold the converted ADC data have to change this to  double

//initialize some values to avoid errors in calculations




/************************************************************************/
/*	ADC FUNCTIONS                                                       */
/************************************************************************/
/**
 * \brief Reads 8bit value stored in a register
 *
 * \param reg_address register address
 * 
 * \return uint8_t -> value in register
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
 * \brief Returns the ADC data rate
 * 
 * \param None
 * 
 * \return int the DRATE, either 11,10,01,00
 */
uint8_t adc_read_drate(void){
	
	return ( 0x03 & adc_register_read(CONFIG1_address));
}



/**
 * \brief returns delay time for sleep mode of the ADC with the recommended initial delay time for the ADC, see Table 8 in ADS1158 document
 * \note conditions Chop = 0 and DLY[2:0] = 000
 *.													Table 8. 
 *.								Start Condition to DRDY Delay, Chop = 0, DLY[2:0] = 000
 *.
 *.							INITIAL DELAY (Standby Mode)			INITIAL DELAY (Sleep Mode)
 *.								(fCLK cycles)							(fCLK cycles)
 *.
 *.		DRATE[1:0]		   Fixed-Channel	Auto-Scan					Fixed-Channel		Auto-Scan
 *.			11					802				708							866					772
 *.			10					1186			1092						1250				1156
 *.			01					2722			2628						2786				2692
 *.			00					8866			8772						8930				8836
 *
 *
 * \return uint16_t
 */
float adc_initial_delay_time(void){
	
	float delay_time = 0;
	
		switch( adc_read_drate() ){ // get the adc DRATE
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
 * \brief This function returns the Channel ID from the STATUS_byte byte
 * \note STATUS_byte BIT \n
 *|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |
 * 
 * \param status_byte (8 bits)
 * 
 * \return uint8_t, 8 bit value representing the channel ID from where the data is read
 */
uint8_t adc_chid_status(uint8_t status_byte){
		//STATUS_byte BIT
		//|  NEW    |  OVF   |  SUPPLY  |  CHID 4  |  CHID 3  |  CHID 2  |  CHID 1  |  CHID 0  |
		return (uint8_t) (status_byte & 0b00011111); //return last 5 bits of STATUS_byte byte
}

/**
 * \brief Checks if the NEW bit is set in the STATUS_byte byte \n
 *The NEW bit is set when the results of a Channel Data Read Command returns new channel data. 
 * 
 * \param status_byte
 * 
 * \return boolean
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
 * \brief  Checks if the OVF bit is set in the STATUS_byte byte
 * \details	Description : When this bit is set, it indicates that the differential voltage applied to the ADC inputs have exceeded the range
 *	of the converter |VIN| > 1.06VREF. During over-range, the output code of the converter clips to either positive FS
 *	(VIN ? 1.06 × VREF) or negative FS (VIN ? –1.06 × VREF). This bit, with the MSB of the data, can be used to
 *	detect positive or negative over-range conditions 
 * 
 * \param status_byte
 * 
 * \return boolean true if bit is set, false if bit is not set 
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
 * \details	 Description : This bit indicates that the analog power-supply voltage (AVDD – AVSS) is below a preset limit. The SUPPLY bit
 *			 is set when the value falls below 4.3V (typically) and is reset when the value rises 50mV higher (typically) than
 *			 the lower trip point. The output data of the ADC may not be valid under low power-supply conditions.
 * 
 * \param status_byte
 * 
 * \return boolean true if bit is set, false if bit is not set 
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
 * \brief  Writes 8bit value into provided register address
 *  
 * \param reg_address register address
 * \param value to be written
 * 
 * \return void
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
  * \brief This function sends a command to the ADC
  * 
  * \param command , 
  * 
  * \return void
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
 * \brief Reads converted data in a channel ID from the ADC and returns it to the user, Reads 3 bytes: STATUS_BYTE, MSB_BYTE, LSB_BYTE
 * 
 * \param void 
 * 
 * \return uint32_t ->  Converted value in a 32bit data, [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
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
 * \brief This function returns the STATUS byte from the Channel Data register read
 * 
 * \param raw_channel_register_read_data 32bit Value from SPI reading, this 32bit value is from "ADCchannelRead_registerFormat" function
 * 
 * \return uint8_t -> 8bit value of status byte
 */
uint8_t adc_return_status_byte(uint32_t raw_channel_register_read_data){
	//The input 32bit value contains [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
	//We return status
	return (uint8_t)((raw_channel_register_read_data >> 16)& 0x000000FF); //0X000F = 0b0000_0000_0000_0000_0000_0000_1111_1111, this returns the status byte
}

/**
 * \brief This function returns the RAW data(MSB+LSB) byte from the Channel Data register read
 * 
 * \param raw_channel_register_read_data -> 32bit Value from SPI reading, this 32bit value is from "ADCchannelRead_registerFormat" function
 * 
 * \return uint16_t -> 16bit value of Raw ADC data
 */
uint16_t adc_return_raw_data(uint32_t raw_channel_register_read_data){
	//The input 32bit value contains [31:24] Don't care, [23:16] STATUS byte,[15:8] MSB of measurement, [7:0] LSB of measurement
	//We return MSB and LSB
	return (uint16_t)((raw_channel_register_read_data )& 0x0000FFFF); //0X0F = 0b0000_0000_0000_0000_1111_1111_1111_1111, this returns the status byte
}

/**
 * \brief Function to get no of set bits in binary representation of passed binary no. 
 * 
 * \param n -> number from which to count the bits
 * 
 * \return unsigned int -> representing the number of set bits in the number
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
 * \brief  Reads all the ADC channels selected in configuration settings(Registers MUXDIF,MUXSG0,MUXSG1) and stores the RAW data in the ADC_raw_data_array, the Channel ID is the corresponding index \n
 * \details Methodology \n
 *			Read all values from the ADC \n
 *
 *			Method \n
 *			Count number of loops\n
 *			Scan through the channel ID's\n
 *			if new bit is set and OVF+SUPPLY bits are not set, store the raw value\n
 *			The CHID is the Array index\n\n
 *
 *			For example, if channels 3, 4, 7, and 8 are selected for measurement in the list, the ADS1158 converts
 *			the channels in that order, skipping all other channels. After channel 8 is converted,
 *			starts over, beginning at the top of the channel list, circuitry are completely disabled. channel 3
 * 
 * \return void
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
 * \brief Toggles the start pin on the ADC
 * 
 * \param void
 * 
 * \return void
 */
void adc_toggle_start_pin(void){
	digitalWrite(start_adc,HIGH); //starts conversion
	delayMicroseconds(2); //wait for data to settle
	digitalWrite(start_adc,LOW); //stops conversion so that we can go to the next Channel ID, Check channel ID in Table 10 of the ADC manual
}

/**
 * \brief Returns an integer value of a twos complement number
 * 
 * \param value
 * \param num_bits
 * 
 * \return int
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
 * \brief converts ADC value into millivolts \n
 *         mV = ADC_data * range\n
 *				----------------\n
 *				Gain * ADC_Resolution\n 
 *
 * \note "http://scientific-solutions.com/products/faq/ssi_faq_adc_equations%20VER2.shtml#:~:text=2%27s%20Complement%20ADC%20Data%20with%208%2Dbit%20resolution"
 * \param ADC_reading
 * 
 * \return int millivolts of ADC value
 */
double adc_mv(int ADC_reading, double vref, double gain){
	
	if ((vref == NULL) || (vref == 0)) vref =  ADC_VREF;
	if (gain == NULL || (gain == 0)) gain = ADC_GAIN;
	//return (ADC_reading*converted_adc_data[28]*1000) / (double) ((double)30720  * converted_adc_data[27]);
	return (((double)ADC_reading*vref*1000) /( (double) 30720.00 * gain) );
}


/**
 * \brief Converts raw_ADC_Data into converted_ADC_data 
 * 
 * \param 
 * 
 * \return void
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
	{ //started from index 8 because the first indices 0->7 in the array are not used
		converted_data[i] = adc_mv( (twos_complement_to_int(raw_data[i],number_of_bits_adc) - twos_complement_to_int(raw_data[24],number_of_bits_adc)),converted_data[28], converted_data[27]); //Vin = (Vref/GainError) * [ (OutputCode - OffsetCode) / 0x7800 ]
	}
	
	return;
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
	//clear bit at GPIO number position set as output 
	gpioc_current_value &= (~(1 << gpio_number));
		
	adc_register_write(GPIOC_address, gpioc_current_value); //write new value to ADC register
	
	//make GPIO low, by setting the corresponding bit to zero(&)
	uint8_t gpiod_current_value = adc_register_read(GPIOD_address);
	//clear bit at GPIO number position
	gpiod_current_value &= (~(1 << gpio_number));
		
	adc_register_write(GPIOD_address, gpiod_current_value); //write new value to ADC register
	
}


/**
 * \brief   Function converts millivolts from the temperature sensor output to temperature in Celsius\n
 *			Formula : T = (V_out - V_offset)/Tc) + T_INFL
 * 
 * \note Temperature sensor Data sheet: "https://www.ti.com/lit/ds/symlink/tmp235.pdf?ts=1594142817615&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTMP235"
 *
 * \param milli_volts -> Digital Voltage from ADC
 * 
 * \return float -> Temperature conversion in C from TMP235
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
 * \brief  Converts voltage to corresponding magnetic field (reference is for the  magnetic sensor)
 *
 * \note Magnetic sensor Data sheet: "https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjxps3qg8PqAhXBo54KHUn5CvwQFjAAegQIBRAB&url=https%3A%2F%2Fwww.allegromicro.com%2F~%2Fmedia%2FFiles%2FDatasheets%2FA1318-A1319-Datasheet.ashx&usg=AOvVaw39zGCju7QuDLgpcH9PKde_"
 * 
 * \param milli_volts
 * 
 * \return float  -> Magnetic field density in milli_Tesla (1G = 0.1mT)
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
 * \brief This functions sets up the ADC
 *         STEPS : 1. Reset the SPI Interface, 2.Stop the converter, 3.Reset the Converter, 4.Configure the registers, 5.Verify Register Data, 6.Start the converter, 7.Read Channel Data
 * \note Page 40 configuration guide. "https://www.ti.com/lit/ds/symlink/ads1158.pdf?ts=1603990103929&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1158"
 * 
 * 
 * \return void
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