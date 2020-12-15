#pragma once

/************************************************************************/
/* PINS DESCRIPTIONS                                                    */
/************************************************************************/
#define ctrl_vstr A0 //A0 the control signal for the voltage stress circuit, labeled CTRL_VSTR in figure 1
#define  _csPin_memory  A1 //Chip select pin for memory, should be configured as digital output
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
  
  pinMode(_csPin_memory,OUTPUT);

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