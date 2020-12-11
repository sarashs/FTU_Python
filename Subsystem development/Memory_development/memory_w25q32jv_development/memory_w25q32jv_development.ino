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
	
	double data_converted[] = {4.22344,1.34345454,4.4566,2.7787,3.22344,5.34345454,7.4566,6.7787,4.22344,1.34345454,4.4566,2.7787,3.22344,5.34345454,7.4566,6.7787,
								4.22344,1.34345454,4.4566,2.7787};

	int size_data = 21;
	double dataOut[size_data];
	int gain  = 1000;
	
	memory_addresses_linkedlist.Clear();
	for (int i = 0; i < 4000 ; i++)
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
		
		memory_store_array_function(&flash,data_converted,size_data, &memory_addresses_linkedlist,ARRAY_GAIN,DELIMITER);

		
		if (memory_addresses_linkedlist.GetSize() == MEM_CLUSTER_SIZE) //above 1500, the micro controller runs out of memory
		{
			//save the linked list as an array into memory
			int linkedlist_size = memory_addresses_linkedlist.GetSize();
			uint32_t *array = memory_addresses_linkedlist.ToArray();
			uint32_t linkedList_as_array[linkedlist_size] = {0}; //change from uint32 to double
			
			 for (int j = 0; j < linkedlist_size ; j++)
			 {
				 linkedList_as_array[j] = *(array + j);
			 }	 
			//convert array to string
			String linkedList_as_string = array_to_string(linkedList_as_array,linkedlist_size,ADDRESS_GAIN,DELIMITER);

			Serial.println(linkedList_as_string);
			Serial.print("String size : ");
			Serial.print(sizeof(linkedList_as_string));
			Serial.println(" Bytes");
			
			//store_list_in_memory
			memory_store_array_function(&flash, linkedList_as_array, linkedlist_size, &linkedlist_of_mem_addresses_of_other_linkedlists, ADDRESS_GAIN, DELIMITER); //gain is one so that addresses are not manipulated

			//clear linked list
			memory_addresses_linkedlist.Clear(); //reset so that RAM is not affected
			
		}
			}
		
	
  // print out all the memory addresses stored
  for (int current_index = 0 ; current_index < linkedlist_of_mem_addresses_of_other_linkedlists.GetSize(); current_index++)
  {
	 Serial.print("Size ");
	 Serial.println(linkedlist_of_mem_addresses_of_other_linkedlists.GetSize());
	 Serial.print("Loop ");
	 Serial.println(current_index);

	 
	 Serial.print("Address ");
	 Serial.print(current_index);
	 Serial.print(" : ");
	 uint32_t string_address = linkedlist_of_mem_addresses_of_other_linkedlists.GetAt(current_index);
	 Serial.println( string_address, HEX);
	 
	 //array the addresses for each value in the array
	 uint32_t address_array[MEM_CLUSTER_SIZE] = {0};
		 
	 memory_retrieve_array_function(&flash, address_array, MEM_CLUSTER_SIZE, ADDRESS_GAIN, DELIMITER, &linkedlist_of_mem_addresses_of_other_linkedlists, current_index);
	 //Memory retrieve function is not useful, it only fills in the last array

	 LinkedList<uint32_t> mem_addresses;
	 mem_addresses.Clear(); //clear the new linked list
	 
	//adding the double values in address_array into unint32
	 for (int g = 0; g < MEM_CLUSTER_SIZE; g++)
	 {
		 mem_addresses.InsertTail(address_array[g]);
		 //Serial.println( address_array[g] );
	 }
	 
	for (int g = 0; g < mem_addresses.GetSize() ; g++)
	{
		//retrieve each of the arrays now
		memory_retrieve_array_function(&flash, dataOut,size_data, ARRAY_GAIN, DELIMITER, &mem_addresses, g);
		//print data out
		Serial.print("Data out ");
		Serial.print(g);
		Serial.print(" : ");
		 
		//print the array out
		for (int f = 0; f < size_data; f++)
		{
			Serial.print(dataOut[f],5);
			Serial.print(", ");
		}
		Serial.println();	
				
	}
}
//retrieve the rest of the addresses in the original addresses array
for (int i = 0; i < memory_addresses_linkedlist.GetSize(); i++)
{
	memory_retrieve_array_function(&flash, dataOut,size_data, ARRAY_GAIN, DELIMITER, &memory_addresses_linkedlist, i);
	//print data out
	Serial.print("Data out ");
	Serial.print(i);
	Serial.print(" : ");
	//
			
	//print the array out
	for (int f = 0; f < size_data; f++)
	{
		Serial.print(dataOut[f],5);
		Serial.print(", ");
	}
	Serial.println();
}
  
 Serial.println("Done");
  
  

}

void loop()
{

}
 
 //____________________________________________________SENDING_TO_SERIAL_______________________________________


