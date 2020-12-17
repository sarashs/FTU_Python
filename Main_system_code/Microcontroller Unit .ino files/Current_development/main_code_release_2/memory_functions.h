#pragma once
/**
 * \file memory_functions.h
 *
 * \brief This file contains functions used to communicate with SPI Flash Memory
 *
 * \author Valentine Ssebuyungo 
 *
 * \version Revision: 1.0 
 *
 * \date  2020/12/16 
 *
 * \note Linked list library used https://github.com/luisllamasbinaburo/Arduino-LinkedList
 
 * \note SPI Memory Library used https://github.com/Marzogh/SPIMemory
 *
 * \details How the memory works \n\n
 *
 *PART 1 -> Storing into memory \n
 *1. Build a linked list to store addresses \n
 *2. initialize a linked list \n
 *3. check if the memory is full, if full -> erase chip and try to get an address again \n
 *4. convert the adc double array into a string \n
 *5. store the data_String in memory and add the string address to a linked list \n
 *6.this is repeated until the ADC has stopped reading data \n\n
 *
 *Part 2 Retrieving Memory \n
 *1. iterate through the linked list, popping off the top item until the list is done \n
 *2. read back the string from the memory \n
 *3. convert the string back to a double array \n
 *4. convert the array into a json doc \n
 *5. send the read back data to the serial port \n
 *6. clear the linked lists \n
 *
 *
 */

/************************************************************************/
/*      MEMORY CODE                                                    */
/************************************************************************/

#include <LinkedListLib.h>  
#include <SPIMemory.h>    
#include "Arduino.h"



//DEFINITIONS
///1. A delimiter is one or more characters that separate text strings.
#define DELIMITER ',' 
///Since double to string can only have 2 decimal places, the array is
///multiplied by this value before converting it to a string and then divided
///by this value for restoration
#define ARRAY_GAIN 1000.0	
///number of addresses in every node of linked list pointing to addresses
#define MEM_CLUSTER_SIZE 20 


// Create LinkedList
LinkedList<uint32_t> memory_addresses_linkedlist;
LinkedList<uint32_t> linkedlist_of_mem_addresses_of_other_linkedlists;  ///each value in this linked list points to an address_1 in
																		/// memory that contains an array addresses_2,
																		///each address_2 contains an array of data
SPIFlash flash(_csPin_memory);                                          //Initialize the memory



/**
 * \brief Sets up memory and ensures that we have at least 2MB enough space for test
 *		  
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
 * \brief Converts a double array to string and separates the values using a delimiter
 * 
 * \param array_to_convert 
 * \param array_size
 * \param gain factor to scale up the number. This is done because this conversion strips off all decimal points so you can scale up the number and scale it down later
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
/**
 * \brief Converts a uint32_t array to string and separates the values using a delimiter
 * 
 * \param array_to_convert 
 * \param array_size
 * \param gain factor to scale up the number. This is done because this conversion strips off all decimal points so you can scale up the number and scale it down later
 * \param delimeter 
 * 
 * \return String
 */
String array_to_string (uint32_t array_to_convert[], int array_size,int gain, char delimeter){
	String string_converted;
	for (int i = 0; i < array_size; i++)
	{
		string_converted = string_converted + array_to_convert[i]*gain + delimeter;
	}
	return string_converted;
}

/**
 * \brief Stores a double array into memory, adds the array address in memory as the tail of the linked list
 * 
 * \param array_to_store passed by address so no need to return
 * \param array_size
 * \param linkedlist_of_addresses passed by address so no need to return
 * 
 * \return void
 */
void __memory_store_array (SPIFlash *flash_mem ,double array_to_store[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses, double gain, char delimiter){
	//convert array to string
	String string_to_store = array_to_string(array_to_store,array_size,gain,delimiter);
	//store the data_String in memory and add the string address to a linked list
		  
	while(true){
		uint32_t _address = flash_mem->getAddress(sizeof(string_to_store));
		// This function should be changed depending on the type of data being written to the flash memory
		if (flash_mem->writeStr(_address,string_to_store,false)) { //error check is set to false, this could cause errors but increases reading time because we are trying to get as much data as possible
				  
			//Serial.println(F("successful"));
				  
			linkedlist_of_addresses->InsertTail(_address); //save the string address at the end of the linked list
			break; //get out of while loop
		}
		else {
			//raise an error! and try to get the next address
		}
	}
}

/**
 * \brief Stores a uint32_t array into memory, adds the array address in memory as the tail of the linked list
 * 
 * \param array_to_store passed by address so no need to return
 * \param array_size
 * \param linkedlist_of_addresses passed by address so no need to return
 * 
 * \return void
 */
void __memory_store_array (SPIFlash *flash_mem ,uint32_t array_to_store[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses, int gain, char delimiter){
	//convert array to string
	String string_to_store = array_to_string(array_to_store,array_size,gain,delimiter);
	//store the data_String in memory and add the string address to a linked list
	
	while(true){
		uint32_t _address = flash_mem->getAddress(sizeof(string_to_store));
		// This function should be changed depending on the type of data being written to the flash memory
		if (flash_mem->writeStr(_address,string_to_store,false)) { //error check is set to false, this could cause errors but increases reading time because we are trying to get as much data as possible
			
			//Serial.println(F("successful"));
			
			linkedlist_of_addresses->InsertTail(_address); //save the string address at the end of the linked list
			break; //get out of while loop
		}
		else {
			//raise an error! and try to get the next address
		}
	}
}




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
///Function overload to support uint32_t array
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
///Function overload to support uint32_t array
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


/**
 * \brief Stores and array in memory and saves links to the addresses stored, variables are passed by address so no need to return!
 * 
 * \param flash_memory Flash memory object
 * \param array_to_store
 * \param array_size
 * \param gain
 * \param delimiter
 * \param mem_cluster_size
 * \param addresses_linked_list
 * \param pointer_to_addresses_linkedlist
 * 
 * \return void
 */
void save_array_in_memory(SPIFlash *flash_memory, double array_to_store[], int array_size, int gain = ARRAY_GAIN,
char delimiter = DELIMITER, int mem_cluster_size = MEM_CLUSTER_SIZE,
LinkedList<uint32_t> *addresses_linked_list = &memory_addresses_linkedlist,
LinkedList<uint32_t> *pointer_to_addresses_linkedlist = &linkedlist_of_mem_addresses_of_other_linkedlists){
	
	#define ADDRESS_GAIN 1   //addresses stored in the memory have no gain so use 1
	__memory_store_array(flash_memory,array_to_store,array_size, addresses_linked_list,gain,delimiter);
	
	if (addresses_linked_list->GetSize() == mem_cluster_size){ //above 1500 values in a linked list,
		//the RAM runs out so the addresses linked lists are divided into
		//MEM_CLUSTERS of max size 20
		//
		//Each memory cluster has addresses of arrays in memory
		//Each memory cluster is then stored in the memory
		//Another linked list is used to store the addresses of the memory clusters
		
		
		//save the linked list as an array into memory
		int linkedlist_size = addresses_linked_list->GetSize();
		uint32_t *array = addresses_linked_list->ToArray();
		uint32_t linkedList_as_array[linkedlist_size] = {0}; //change from uint32 to double
		
		for (int j = 0; j < linkedlist_size ; j++) //transferring the data into another array
		{
			linkedList_as_array[j] = *(array + j);
		}

		//store_list_in_memory
		__memory_store_array(flash_memory, linkedList_as_array, linkedlist_size, pointer_to_addresses_linkedlist, ADDRESS_GAIN, delimiter); //gain is one so that addresses are not manipulated

		//clear linked list
		addresses_linked_list->Clear(); //reset so that RAM is not affected
		
	}
}

/**
 * \brief This functions prints out all the arrays stored in the flash memory for the current test
 * 
 * \param flash_memory 						flash memory object
 * \param array_size   						sizes of the arrays in memory
 * \param gain         						the gain that was used to store arrays, this has a default value of 100,000
 * \param delimiter    						char used to separate arrays in memory
 * \param mem_cluster_size					size of each chunk of memory, max size is about 20
 * \param addresses_linked_list				LinkedList object where each value is a memory address, containing an array of data
 * \param pointer_to_addresses_linkedlist	Each value in this linked list -> is an address in memory -> This address in memory pointed to is another linkedList_2 data
 *											Linkedlist2 values of size mem_cluster are addresses in memory that point to an array of data
 * 
 * \return void
 */
void print_all_arrays_in_memory(SPIFlash *flash_memory, int array_size, int gain = ARRAY_GAIN, //this is for double arrays in memory
char delimiter = DELIMITER, int mem_cluster_size = MEM_CLUSTER_SIZE,
LinkedList<uint32_t> *addresses_linked_list = &memory_addresses_linkedlist,
LinkedList<uint32_t> *pointer_to_addresses_linkedlist = &linkedlist_of_mem_addresses_of_other_linkedlists ){
	
	// print out all the memory addresses stored
	//1. First we iterate through the linked list which stores the addresses in memory for other linked list
	//	Each value in this linked list -> is an address in memory -> This address in memory pointed to is another linkedList_2 data
	//    Linkedlist2 values of size mem_cluster are addresses in memory that point to an array of data
	
	// So for each value in pointer_LinkedList, we decipher the Linked list of addresses
	// For each value in Linked list addresses, we read that value in memory to get the array
	
	double dataOut[array_size];
	
	for (int current_index = 0 ; current_index < pointer_to_addresses_linkedlist->GetSize(); current_index++)
	{
		
		//array the addresses for each value in the array
		uint32_t address_array[mem_cluster_size] = {0};
		
		memory_retrieve_array_function(flash_memory, address_array, mem_cluster_size, ADDRESS_GAIN, delimiter, pointer_to_addresses_linkedlist, current_index);
		//Memory retrieve function is not useful, it only fills in the last array

		LinkedList<uint32_t> mem_addresses; //linked list that will contain addresses to arrays in memory
		mem_addresses.Clear(); //clear the new linked list
		
		//storing the values in address_array into linked list because our functions take LinkedLists not arrays
		for (int i = 0; i < MEM_CLUSTER_SIZE; i++)
		{
			mem_addresses.InsertTail(address_array[i]);
		}			
		for (int mem_address_index = 0; mem_address_index < mem_addresses.GetSize() ; mem_address_index++) //Iterating through every address to read the array
		{
			//retrieve each of the arrays now, stored in dataOut
			memory_retrieve_array_function(flash_memory, dataOut,array_size, gain, delimiter, &mem_addresses, mem_address_index);
			//print out the data out
			update_json_doc(test_id,false,test_start,test_error,error_message,test_time_count,0,0,dataOut,array_size);
			send_data_to_serial();
			delay(5);
			
		}
	}
	
	//If the original addresses array did not get full up to a mem_cluster_size, it still contains some value so we read those out as well
	
	//retrieve the rest of the addresses in the original addresses array
	for (int linkedlist_current_index = 0; linkedlist_current_index < addresses_linked_list->GetSize(); linkedlist_current_index++)
	{
		memory_retrieve_array_function(flash_memory, dataOut,array_size, gain, delimiter, addresses_linked_list, linkedlist_current_index);

		update_json_doc(test_id,false,test_start,test_error,error_message,test_time_count,0,0,dataOut,array_size);
		send_data_to_serial();
		delay(5); //delay added to accommodate python script
	}
	
}

