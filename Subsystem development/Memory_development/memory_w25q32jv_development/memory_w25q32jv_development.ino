/*
 * memory_w25q32jv_development.ino
 *
 * Created: 12/1/2020 4:23:14 PM
 * Author: Valentine Ssebuyungo
 */ 
#include "memory_functions.h"





void setup()
{
		
	/* add setup code here, setup code runs once when the processor starts */
	memory_setup(&flash);
	Serial.begin(1000000);
	delay(1000);
	
	double data_converted[] = {4.22344,1.34345454,4.4566};
		//2.7787,3.22344,5.34345454,7.4566,6.7787,4.22344,1.34345454,4.4566,2.7787,3.22344,5.34345454,7.4566,6.7787,
			//					4.22344,1.34345454,4.4566,2.7787,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9};

	int size_data = 3;
	double dataOut[size_data];
	int gain  = 1000;
	
	memory_addresses_linkedlist.Clear();
	for (int i = 0; i < 40 ; i++)
	{
		//creating array
		Serial.print("Array to be stored ");
		Serial.print(i);
		Serial.print(" : ");
		double random_number = ((rand() % 5 + 1)/10.00);
		for (int i=0; i<size_data; i++)
		{
			data_converted[i] = data_converted[i]*0.1 + random_number;
			Serial.print(data_converted[i],5);
			Serial.print(DELIMITER);
			Serial.print(" ");
		}
		Serial.println();
		
		save_array_in_memory(&flash,data_converted,size_data);

		}
		
		print_all_arrays_in_memory(&flash, size_data);
  
 Serial.println("Done");
  
  

}

void loop()
{

}
 

