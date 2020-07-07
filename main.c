/*
 * Timer_trial.c
 *
 * Created: 7/4/2020 3:04:44 PM
 * Author : hp
 */ 
#include "sam.h"

#define LED1 PORT_PB30;
#define LED0 PORT_PA06;


int main(void)
{
  /* Replace with your application code */
  SystemInit(); //Initialize system
  init_TC4(); // Enables TC4 timer with the predefined interrupts
  
  //Configure LED0 and LED 1 as OUTPUTS
  REG_PORT_DIRSET1 = LED0;  
  
  while (1)
  {
    
  }
  return 0;
}

void init_TC4(){ //initialize TC4 timer
  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
  GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
  GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  //GCLK_CLKCTRL_ID_TCC2_TC3; this can feed it to TC3 as well
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Configure TC4 (16 bit counter by default)
  
  
  REG_TC4_COUNT16_CC0 = 0x7A0;                      // Set the TC4 CC0 register to  decimal 1952, 0.5 sec
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_TC4_COUNT16_CC1 = 0xF41;                      // Set the TC4 CC1 register to  decimal 3905, 1 sec
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  //Enabling interrupts
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 ;        // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_MC1 | TC_INTENSET_MC0 ;     // Enable TC4 Match CC0 and CC1 interrupts 
  // REG_TC4_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 ;     // Disable TC4 interrupts

  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV256 |     // Set prescaler to 256,
  TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
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

void TC4_Handler()                              // Interrupt Service Routine (ISR) for timer TC4
{
 // Check for match counter 0 (MC0) interrupt
  if (TC4->COUNT16.INTFLAG.bit.MC0 && TC4->COUNT8.INTENSET.bit.MC0)
  {
    // compare 0 (CC0) code here:
    
    REG_TC4_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag
  }

  // Check for match counter 1 (MC1) interrupt
  if (TC4->COUNT16.INTFLAG.bit.MC1 && TC4->COUNT8.INTENSET.bit.MC1)
  {
    //compare 1 (CC1) code here:
     REG_PORT_OUTTGL1 = LED0; //toggle LED
    
    REG_TC4_INTFLAG = TC_INTFLAG_MC1;        // Clear the MC1 interrupt flag
  }
  
}

