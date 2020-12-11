#pragma once
/************************************************************************/
/*      MEMORY CODE                                                    */
/************************************************************************/

#include <LinkedList.h> //https://github.com/luisllamasbinaburo/Arduino-LinkedList
#include <SPIMemory.h>    //See https://github.com/Marzogh/SPIMemory




//DEFINITIONS
#define DELIMITER ',' //A delimiter is one or more characters that separate text strings.
#define ARRAY_GAIN 1000.0 //Since double to string can only have 2 decimal places, the array is
//multiplied by this value before converting it to a string and then divided
//by this value for restoration
#define ADDRESS_GAIN 1   //addresses stored in the memory have no gain so use 1
#define MEM_CLUSTER_SIZE 20 //number of addresses in every node of linked list pointing to addresses

// Create LinkedList
LinkedList<uint32_t> memory_addresses_linkedlist;
LinkedList<uint32_t> linkedlist_of_mem_addresses_of_other_linkedlists; //each value in this linked list points to an address_1 in
// memory that contains an array addresses_2,
//each address_2 contains an array of data
SPIFlash flash(_csPin_memory);                                          //Initialize the memory


//        PART 1 -> STORAGE
//Build a linked list to store addresses
//initialize a linked list
//check if the memory is full, if full -> erase chip and try to get an address again
//convert the adc double array into a string
//store the data_String in memory and add the string address to a linked list
//this is repeated until the ADC has stopped reading data
/**
 * \brief Sets up memory and checks if we have enough space for test
 * 
 * 
 * \return void
 */
void memory_setup(SPIFlash *flash_object){
	delay(1000);
	flash_object->begin();
	if (flash_object->getAddress((230*15000*sizeof(byte))) == false) //check if we can store at least 1.1 MB of data otherwise erase the chip
	{
		flash_object->eraseChip(); //if chip is full,erase it
	}
}
/**
 * \brief converts an array to string and separates the values using a delimiter
 * 
 * \param array_to_convert
 * \param array_size
 * \param gain ,factor to scale up the number. This is done because this conversion strips off all decimal points so you can scale up the number and scale it down later
 * \param delimeter
 * 
 * \return String
 */
String array_to_string (double array_to_convert[], int array_size,double gain, char delimeter){
	String string_converted;
	for (int i = 0; i < array_size; i++)
	{
		string_converted = string_converted + array_to_convert[i]*gain + delimeter;
	}
	return string_converted;
}
String array_to_string (uint32_t array_to_convert[], int array_size,int gain, char delimeter){
	String string_converted;
	for (int i = 0; i < array_size; i++)
	{
		string_converted = string_converted + array_to_convert[i]*gain + delimeter;
	}
	return string_converted;
}
/**
 * \brief Stores an array into memory, adds the array address in memory to the tail of the linked list
 * 
 * \param array_to_store (passed by address so no need to return)
 * \param array_size
 * \param linkedlist_of_addresses (passed by address so no need to return)
 * 
 * \return void
 */
void memory_store_array_function (SPIFlash *flash_mem ,double array_to_store[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses, double gain, char delimiter){
	//convert array to string
	String string_to_store = array_to_string(array_to_store,array_size,gain,delimiter);
	//store the data_String in memory and add the string address to a linked list
		  
	while(true){
		uint32_t _address = flash_mem->getAddress(sizeof(string_to_store));
		// This function should be changed depending on the type of data being written to the flash memory
		if (flash_mem->writeStr(_address,string_to_store)) {
				  
			//Serial.println(F("successful"));
				  
			linkedlist_of_addresses->InsertTail(_address); //save the string address at the end of the linked list
			break; //get out of while loop
		}
		else {
			//raise an error! and try to get the next address
		}
	}
}
void memory_store_array_function (SPIFlash *flash_mem ,uint32_t array_to_store[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses, int gain, char delimiter){
	//convert array to string
	String string_to_store = array_to_string(array_to_store,array_size,gain,delimiter);
	//store the data_String in memory and add the string address to a linked list
	
	while(true){
		uint32_t _address = flash_mem->getAddress(sizeof(string_to_store));
		// This function should be changed depending on the type of data being written to the flash memory
		if (flash_mem->writeStr(_address,string_to_store)) {
			
			//Serial.println(F("successful"));
			
			linkedlist_of_addresses->InsertTail(_address); //save the string address at the end of the linked list
			break; //get out of while loop
		}
		else {
			//raise an error! and try to get the next address
		}
	}
}


//______________________________________________PART 2 RETRIEVING MEMORY_____________________________________________________________________________
//iterate through the linked list, popping off the top item until the list is done
//read back the string from the memory
//convert the string back to a double array
//convert the array into a json doc
//send the read back data to the serial port
//clear the linked lists

/**
 * \brief converts a string with values separated by a delimeter into a double array
 * example "422344.00,134345.45,445660.00,277870.00," to [422344.00,134345.45,445660.00,277870.00] where "," is the delimeter 
 * 
 * \param string_array
 * \param storage_array
 * \param string_array_size
 * \param gain ,factor to scale down the number. This is done because this conversion strips off all decimal points so you can scale up the number and scale it down later
 * \param delimeter
 * 
 * \return void
 */
void string_to_array (String string_array, double storage_array[], int string_array_size, double gain, char delimeter){
	 
	  for (int i = 0; i < string_array_size ; i++)
	  {
		  String string_value = string_array.substring(0,string_array.indexOf(delimeter)); //getting first value in array

		  double value = string_value.toDouble(); //store the double value
		  storage_array[i] = value/gain; //store the value into the array


		  string_array.remove(0, (string_array.indexOf(delimeter) + 1 ) );

	  }
	
}
//function overload to support int array
void string_to_array (String string_array, uint32_t storage_array[], int string_array_size, int gain, char delimeter){
	 
	  for (int i = 0; i < string_array_size ; i++)
	  {
		  String string_value = string_array.substring(0,string_array.indexOf(delimeter)); //getting first value in array

		  uint32_t value = string_value.toInt(); //store the double value
		  storage_array[i] = value/gain; //store the value into the array


		  string_array.remove(0, (string_array.indexOf(delimeter) + 1 ) );

	  }
	
}
/**
 * \brief Stores an array in "array_to_hold_data",  pointed to by the position listed (default = head) a linked list of addresses in memory
 * 
 * \param array_to_hold_data, passed by address so no need to return
 * \param array_size
 * \param gain
 * \param delimiter
 * \param linkedlist_of_addresses, passed by address so no need to return
 * \param linked_list_index
 * 
 * \return void
 */
void memory_retrieve_array_function(SPIFlash *flash_mem, double array_to_hold_data[], int array_size,double gain, char delimiter, 
	LinkedList<uint32_t> *linkedlist_of_addresses, int linked_list_index = 0){
	
		 //if linked list is empty or invalid index, return 
		if (linkedlist_of_addresses->GetSize() == 0 || linked_list_index >= linkedlist_of_addresses->GetSize())
		{
			return;
		}
		
		
		//get head and read it from memory
		String adc_array_string;
		flash_mem->readStr(linkedlist_of_addresses->GetAt(linked_list_index), adc_array_string); //Default position is head

		//convert string back to double array
		string_to_array(adc_array_string, array_to_hold_data, array_size, gain, delimiter);
		//convert the array into a json doc
		//send json serial data to serial port
}
//overloading function
void memory_retrieve_array_function(SPIFlash *flash_mem, uint32_t array_to_hold_data[], int array_size, int gain, char delimiter, 
	LinkedList<uint32_t> *linkedlist_of_addresses, int linked_list_index = 0){
	
		 //if linked list is empty or invalid index, return
		 if (linkedlist_of_addresses->GetSize() == 0 || linked_list_index >= linkedlist_of_addresses->GetSize())
		 {
			 return;
		 }
		 
		//get head and read it from memory
		String adc_array_string;
		flash_mem->readStr(linkedlist_of_addresses->GetAt(linked_list_index), adc_array_string); //Default position is head
		//convert string back to double array
		string_to_array(adc_array_string, array_to_hold_data, array_size, gain, delimiter);
		//convert the array into a json doc
		//send json serial data to serial port	
}





