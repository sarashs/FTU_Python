#include <SPIMemory.h>
#include <SPI.h>



#define  LM335_pin  A1  
float celcius = 0;


/*
 * memory_w25q32jv_development.ino
 *
 * Created: 12/1/2020 4:23:14 PM
 * Author: Valentine Ssebuyungo
 */ 




void setup()
{
	Serial.begin(9600);
	pinMode(LM335_pin,INPUT);
	
	  /* add setup code here, setup code runs once when the processor starts */

}

void loop()
{
	 delay(1000);
	 Serial.print("Temperature = ");
	 analogReadResolution(10);
	 float  voltageOut = (analogRead(LM335_pin) *3349.0) / 1024.00;
//	 celcius = analogRead(LM335_pin);
	 celcius = ((analogRead(A1) * 3340 * 0.1 ) / 1024.00) - 273;
	 float celcius2 = (voltageOut / 10.00) - 273 ;
	 Serial.println(celcius, 2);
	 Serial.println(celcius2, 2);
	  /* add main program code here, this code starts again each time it ends */

}
