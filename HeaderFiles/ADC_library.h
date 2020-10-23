/************************************************************************
 *
 * 	File Name: FTU__system_code.h
 *
 * 	File creator: Rohit Singh 2020-10-21
 *
 * 	File Description: This file contains some ADC specific functions
 *
 * 	File Revision History:
 *
 *************************************************************************/
// Functions -------------------------------------------------------------

/*************************************************************************
*
*		Function Name: clock_setup()
*
*		Inputs: None
*		Outputs: None
*
*		Description: This function Sets up 1MHz clock used by TC3,TC4,TCC2,TC5
*
*		Function Author: Valentine
*
**************************************************************************/
void clock_setup(void){ //setting up the clock speeds

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

/*************************************************************************
*
*		Function Name: init_TC3()
*
*		Inputs: None
*		Outputs: None
*
*		Description: This function Sets up TC3
*
*		Function Author: Valentine
*
**************************************************************************/
void init_TC3(void){
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

/*************************************************************************
*
*		Function Name: init_TC4()
*
*		Inputs: None
*		Outputs: None
*
*		Description: This function Sets up TC4
*
*		Function Author: Valentine
*
**************************************************************************/
void init_TC4(void){ //initialize TC4 timer
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

/*************************************************************************
*
*		Function Name: init_TC5()
*
*		Inputs: None
*		Outputs: None
*
*		Description: This function Sets up TC5
*
*		Function Author: Valentine
*
**************************************************************************/
void init_TC5(void){
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

/*************************************************************************
*
*		Function Name: pin_setup()
*
*		Inputs: None
*		Outputs: None
*
*		Description: This function initializes pins for MCU
*
*		Function Author: Valentine
*
**************************************************************************/
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
