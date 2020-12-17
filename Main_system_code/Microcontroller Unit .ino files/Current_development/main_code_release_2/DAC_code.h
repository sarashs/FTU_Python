#pragma once
/**
 * \file DAC.h
 *
 * \brief This file contains functions used to with the MCU digital to Analog Converter
 *
 * \author Valentine Ssebuyungo 
 *
 * \version Revision: 1.0 
 *
 * \date  2020/12/16 
 *
 * \details
 */

/************************************************************************/
/* DAC FUNCTIONS                                                        */
/************************************************************************/

/**
 * \brief This function converts a user entered voltage value into a 10 bit DAC value
 * 
 * \param volt -> desired voltage in mini_volts
 * 
 * \return int -> DAC value
 */
int set_dac(float volt) {
	//formula for calculating DAC output voltage Vdac = (dVal / 1023)*2.23V
	return (int)((volt*1023)/2230);
}

