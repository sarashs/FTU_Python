// driver_init.h

// Macros ----------------------------------------------------------------------
//USED FOR TESTING
#define  REDLED A1
#define  BLUELED A2
#define  YELLOWLED A5
#define  ANALOG_PIN A6

#ifndef _DRIVER_INIT_h
#define _DRIVER_INIT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif
