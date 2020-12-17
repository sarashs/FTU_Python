#pragma once
/**
 * \file timers_and_interrupt_setup.h
 *
 * \brief This file contains functions used to initialize timers and interrupts service routines that happen during time interrupts
 * \details The MCU has Up to five 16-bit Timer/Counters (TC), configurable as either: \n
 * One 16-bit TC with compare/capture channels\n
 * One 8-bit TC with compare/capture channels\n
 * One 32-bit TC with compare/capture channels, by using two TCs\n
 * Three 24-bit Timer/Counters for Control (TCC), with extended functions: \n\n\n
 * Up to four compare channels with optional complementary output\n
 * Generation of synchronized pulse width modulation (PWM) pattern across port pins\n
 * Deterministic fault protection, fast decay and configurable dead-time between complementary output\n
 * Dithering that increase resolution with up to 5 bit and reduce quantization error\n
 * \author Valentine Ssebuyungo 
 *
 * \version Revision: 1.0 
 *
 * \date  2020/12/16 
 *
 * \details
 */

#include "samd21/include/samd21g18a.h"


/**
 * \brief Returns value for the overflow counter
 * 
 * \param Clock_FrequencyMHz
 * \param prescaler
 * \param period_ms
 * 
 * \return int
 */
int counter_value (float clock_frequency_MHz,float prescaler, float period_ms){
	return (int) (( (clock_frequency_MHz*1000*period_ms) / (prescaler) )-1) ;
	
}

/**
 * \brief Sets up 1MHz clock used by TC3,TC4,TCC2,TC5
 * 
 * 
 * \return void
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
 * \brief Sets up TC4 timer
 * 
 * 
 * \return void
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
 * \brief Sets up TC5 timer
 * 
 * 
 * \return void
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
 * \brief Sets up TC3 timer
 * 
 * 
 * \return void
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
 * \brief Interrupt service routine for TC4, handles the ADC interrupt
 * 
 * 
 * \return void
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
 * \brief TC5 timer ISR 
 * 
 * 
 * \return void
 */
void TC5_Handler(){ //counter interrupt
	// Check for OVF interrupt
	if (TC5->COUNT16.INTFLAG.bit.OVF && TC5->COUNT16.INTENSET.bit.OVF)
	{
		//Overflow interrupt code here:
		test_time_count++; //increment test count
	
		REG_TC5_INTFLAG = TC_INTFLAG_OVF;        // Clear the MC0 interrupt flag
	}
	return;
}

/**
 * \brief TC3 timer ISR
 * 
 * 
 * \return void
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


