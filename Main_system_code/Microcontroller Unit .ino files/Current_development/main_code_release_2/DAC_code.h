#pragma once

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

