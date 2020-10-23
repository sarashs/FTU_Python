/************************************************************************
 *
 * 	File Name: FTU__system_code.h
 *
 * 	File creator: Rohit Singh 2020-10-21
 *
 * 	File Description: This file is a header file for the file
 * 				FTU_system_code.ino
 *
 * 	File Revision History:
 *
 *************************************************************************/
// Macros -----------------------------------------------------------------------------------------------
// Pin Defintions - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Analog Pins ..........................................................................................
#define CTRL_VSTR A0 			//A0 the control signal for the voltage stress circuit, labeled CTRL_VSTR in figure 1
#define HEATER_PWM A3 			//A3 This is the control signal for the Heater board
#define HELMHOLTZ_PWM A4 		//A4 This is the control signal for the Helmholtz circuit
#define _RESET_ADC 0          		//D0 Should be configured as a digital output. Connected to *RESET on the ADC
// Digital Pins .........................................................................................
#define _PWDN_ADC 1           		//D1 Should be configured as a digital output. Connected to *PWDN on the ADC
#define START_ADC 2           		//D2 Should be configured as a digital output. Connected to START on the ADC.
#define _DRDY_ADC 3           		//D3 Should be configured as a digital input. Connected to *DRDY on the ADC.
#define RED_LED 4          		//D4 Should be configured as a digital output. Control signal for the red LED on the heater board.
#define BLUE_LED 5            		//D5 Should be configured as a digital output. Control signal for the Blue LED on the heater board.
#define CLR_FREQ_DIVIDER 6    		//D6 Should be configured as a digital output. The clear command for the frequency divider. labeled CLR in figure 1.
#define _CS_ADC 7             		//D7 Should be configured as a digital output. This is the *CS line for SPI communication with the ADC
#define MOSI_ADC 8            		//D8 Should be configured as a digital output. This is the MOSI line on the SPI bus. labeled DIN on the ADC.
#define SCK_ADC 9             		//D9 Should be configured as a digital output. This is the SCLK line on the SPI bus.
#define MISO_ADC 10           		//D10 Should be configured as a digital input. This is the MISO line on the SPI bus. labeled DOUT on the ADC.
#define Q11_FREQ_COUNTER 11   		//D11 Should be configured as a digital input. Connected to Q11 on the frequency divider
#define Q12_FREQ_COUNTER 12   		//D12 Should be configured as a digital input. Connected to Q12 on the frequency divider
#define Q13_FREQ_COUNTER 13   		//D13 Should be configured as a digital input. Connected to Q13 on the frequency divider
#define Q14_FREQ_COUNTER 14   		//D14 Should be configured as a digital input. Connected to Q14 on the frequency divider
//A1, A2, A5, A6   4 MCU pins that are not used in the design and left disconnected. They will be accessible through the male/female headers mounted on the MCU board.
#define ANALOG_RESOLUTION 1023
// Register Addresses ..............................................................................................
#define CONFIG0_address 0x00
#define CONFIG1_address 0x01
#define MUXSCH_address 0x02
#define MUXDIF_address 0x03
#define MUXSG0_address 0x04
#define MUXSG1_address 0x05
#define SYSRED_address 0x06
#define GPIOC_address 0x07
#define GPIOD_address 0x08

//Default ADC values to be programmed.............................................................................
#define CONFIG0_default 0x02
#define CONFIG1_default 0x00
#define MUXSCH_default 0x00
#define MUXDIF_default 0x00
#define MUXSG0_default 0xFF
#define MUXSG1_default 0x7F
#define SYSRED_default 0xFF
#define GPIOC_default 0x00
#define GPIOD_default 0x00

//ADC constants ...................................................................................................
#define NUMBER_OF_BITS_ADC 16
#define ADC_Register_Array_Size 9
#define ADC_SPI_SPEED 6000000  //6MHZ -> Must be less than 0.5 adc_clk(15.7MHz)
#define ANALOG_RESOLUTION 1023

// Pin Numbers ....................................................................................................
#define  REDLED A1
#define  BLUELED A2
#define  YELLOWLED A5
#define  ANALOG_PIN A6
