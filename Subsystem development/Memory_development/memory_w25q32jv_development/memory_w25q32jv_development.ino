/*
 * memory_w25q32jv_development.ino
 *
 * Created: 12/1/2020 4:23:14 PM
 * Author: Valentine Ssebuyungo
 */ 
#include <LinkedListLib.h> //https://github.com/luisllamasbinaburo/Arduino-LinkedList
#include <SPIMemory.h>    //See https://github.com/Marzogh/SPIMemory
#include "memory_functions.h"

//DEFINES
#define DELIMITER ',' //A delimiter is one or more characters that separate text strings.
#define ARRAY_GAIN 1000.0 //Since double to string can only have 2 decimal places, the array is 
						  //multiplied by this value before converting it to a string and then divided 
						  //by this value for restoration
#define  csPin  0  
#define MEM_CLUSTER_SIZE 20 //number of addresses in every node of linkedlist pointing to addresses

// Create LinkedList
LinkedList<uint32_t> memory_addresses_linkedlist;
LinkedList<uint32_t> linkedlist_of_mem_addresses_of_other_linkedlists;

SPIFlash flash(csPin);
uint32_t _address;
// This data type should be changed depending on the type of data being write to the flash memory
String dataOut = "This is a test String!";

void setup()
{
		
	/* add setup code here, setup code runs once when the processor starts */
	memory_setup(&flash);
	Serial.begin(1000000);
	delay(1000);
	
	double data_converted[] = {4.22344,1.34345454,4.4566,2.7787,3.22344,5.34345454,7.4566,6.7787};
	int size_data = 8;
	double dataOut[size_data];
	int gain  = 1000;
	
	memory_addresses_linkedlist.Clear();
	for (int i = 0; i < 500; i++)
	{
		//creating array
		Serial.print("Array to be stored ");
		Serial.print(i);
		Serial.print(" : ");
		double random_number = ((rand() % 5 + 1)/10.00);
		for (int i=0; i<size_data; i++)
		{
			data_converted[i] = data_converted[i]*0.1 + random_number;
			Serial.print(data_converted[i]);
			Serial.print(DELIMITER);
			Serial.print(" ");
		}
		Serial.println();
		
		memory_store_array_function(data_converted,size_data, &memory_addresses_linkedlist,ARRAY_GAIN,DELIMITER);

// 
// 		String linkedList_as_string = double_array_to_string(data_converted,size_data,ARRAY_GAIN,DELIMITER);
// 		Serial.println(linkedList_as_string);
// 		Serial.print("String size : ");
// 		Serial.print(sizeof(linkedList_as_string));
// 		Serial.println(" Bytes");
// 		
// 		Serial.print("Array size : ");
// 		Serial.print(sizeof(data_converted));
// 		Serial.println(" Bytes");
// 		
// 		Serial.println();
		
		if (memory_addresses_linkedlist.GetSize() == MEM_CLUSTER_SIZE) //above 1500, the micro controller runs out of memory
		{
			//save the linked list as an array into memory
			int linkedlist_size = memory_addresses_linkedlist.GetSize();
			uint32_t *array = memory_addresses_linkedlist.ToArray();
			double linkedList_as_array[linkedlist_size] = {0}; //change from uint32 to double
			
			 for (int j = 0; j < linkedlist_size ; j++)
			 {
				 linkedList_as_array[j] = (double) *(array + j);
			 }	 
			 
			
			//convert array to string
			String linkedList_as_string = double_array_to_string(linkedList_as_array,linkedlist_size,1,DELIMITER);

			Serial.println(linkedList_as_string);
			Serial.print("String size : ");
			Serial.print(sizeof(linkedList_as_string));
			Serial.println(" Bytes");
			
			//store_list_in_memory
			memory_store_array_function(linkedList_as_array, linkedlist_size, &linkedlist_of_mem_addresses_of_other_linkedlists, 1, DELIMITER); //gain is one so that addresses are not manipulated
			
			//clear linked list
			memory_addresses_linkedlist.Clear(); //reset so that memory is not affected
			
		}
		
	}
	
  // print out all the memory addresses stored
  for (int i = 0 ; i < linkedlist_of_mem_addresses_of_other_linkedlists.GetSize(); i++)
  {
	 Serial.print("Address ");
	 Serial.print(i);
	 Serial.print(" : ");
	 uint32_t string_address = linkedlist_of_mem_addresses_of_other_linkedlists.GetAt(i);
	 Serial.println( string_address, HEX);
	 
	 //retreat the addresses for each value in the array
	 double address_array[MEM_CLUSTER_SIZE] = {0};
		 
	 memory_retrieve_array_function(address_array,MEM_CLUSTER_SIZE, &linkedlist_of_mem_addresses_of_other_linkedlists);
	 LinkedList<uint32_t> mem_addresses;
	
	//adding the double values in address_array into unint32
	 for (int g = 0; g<MEM_CLUSTER_SIZE; g++)
	 {
		 mem_addresses.InsertTail( (uint32_t)address_array[g] );
	 }
	 
	 for (int k = 0; k < MEM_CLUSTER_SIZE; k++)
	 {
		 //retrieve each of the arrays now
		 memory_retrieve_array_function(dataOut,size_data, &mem_addresses);
		 //print data out
		 Serial.print("Data out ");
		 Serial.print(k*i);
		 Serial.print(" : ");
		 
		 for (int f = 0; f < size_data; f++)
		 {
			 Serial.print(dataOut[f]);
			 Serial.print(", ");
		 }
		 Serial.println();
		 
	 }
	
  }
  
  //try to retrieve the addresses
  
  
// 	Serial.println("\n Retrieved arrays \n");
// 	
// 	//retrieving from memory
// 	memory_retrieve_array_function(dataOut,size_data, &memory_addresses_linkedlist);
// 	
// 	
// 	Serial.println("\n Done!!!!!");  

// 
//   // print out all the memory addresses stored
//   for (int i = 0 ; i < memory_addresses_linkedlist.GetSize(); i++)
//   {
// 	 Serial.print("Address ");
// 	 Serial.print(i);
// 	 Serial.print(" : ");
// 	 uint32_t string_address = memory_addresses_linkedlist.GetAt(i);
// 	 Serial.println( string_address, HEX);
// 	 
// 	 //print the data out
// 	 String data;
// 	 flash.readStr(string_address, data);
// 	 Serial.println(data);
//   }
//   
//   memory_addresses_linkedlist.Clear(); //clear the memory
//   
// 
}

void loop()
{

}
 


/************************************************************************/
/*      MEMORY CODE                                                    */
/************************************************************************/
//        PART 1 -> STORAGE
//Build a linked list to store addresses
//initialize a linked list
//check if the memory is full, if full -> erase chip and try to get an address again
//convert the adc double array into a string
//store the data_String in memory and add the string address to a linked list
//this is repeated until the ADC has stopped reading data

//       PART 2 -> RETRIEVING FROM MEMORY
//read back the string from the memory
//convert the string back to a double array
//convert the array into a json doc
//send the read back data to the serial port
//clear the linked lists


/**
 * \brief Sets up memory and checks if we have enough space for test
 * 
 * 
 * \return void
 */
void memory_setup(SPIFlash *flash_object){
	delay(1000);
	flash_object->begin();
	if (flash_object->getAddress((230*4500*sizeof(byte))) == false) //check if we can store at least 1MB of data otherwise cancel
	{
		flash.eraseChip(); //if chip is full,erase it
		
		//Testing
		Serial.println("Memory setup error");
	}
}

/**
 * \brief 
 * 
 * \param array_to_store (passed by address so no need to return)
 * \param array_size
 * \param linkedlist_of_addresses (passed by address so no need to return)
 * 
 * \return void
 */
void memory_store_array_function (double array_to_store[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses, int gain, char delimiter){
	//convert array to string
	String string_to_store = double_array_to_string(array_to_store,array_size,gain,delimiter);
	//store the data_String in memory and add the string address to a linked list
		  
	while(true){
		uint32_t _address = flash.getAddress(sizeof(string_to_store));
		// This function should be changed depending on the type of data being written to the flash memory
		if (flash.writeStr(_address,string_to_store)) {
				  
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
 * \brief converts an array to string and separates the values using a delimiter
 * 
 * \param array_to_convert
 * \param array_size
 * \param gain ,factor to scale up the number. This is done because this conversion strips off all decimal points so you can scale up the number and scale it down later
 * \param delimeter
 * 
 * \return String
 */
String double_array_to_string (double array_to_convert[], int array_size,int gain, char delimeter){
	String string_converted;
	for (int i = 0; i < array_size; i++)
	{
		string_converted = string_converted + array_to_convert[i]*gain + delimeter;
	}
	return string_converted;
}

//______________________________________________RETRIEVING MEMORY_____________________________________________________________________________
//iterate through the linked list, popping off the top item until the list is done
//read back the string from the memory
//convert the string back to a double array
//convert the array into a json doc
//send the read back data to the serial port
//clear the linked lists

/**
 * \brief Reads the arrays stored in memory one by one
 *        The memory addresses are store in the linked list
 *        Reads of the head of the linked list(stack) and pops it off
 * 
 * \param array_to_hold_data (passed by address so no need to return)
 * \param array_size
 * \param linkedlist_of_addresses, passed by address so no need to return
 * 
 * \return void
 */
void memory_retrieve_array_function(double array_to_hold_data[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses){
	
	for (int i = 0; i <linkedlist_of_addresses->GetSize(); i++)
	{
		//get head and read it from memory
		String adc_array_string;
		flash.readStr(linkedlist_of_addresses->GetAt(i), adc_array_string);
		//pop off head as it has already been used
		//linkedlist_of_addresses->RemoveHead();
		//convert string back to double array
		string_to_doublearray(adc_array_string, array_to_hold_data, array_size, ARRAY_GAIN, DELIMITER);
		//convert the array into a json doc
		//send json serial data to serial port
		
		
// 		//Testing
// 		Serial.print("Array read from memory ");
// 		Serial.print(i);
// 		Serial.print(" : ");
// 		for (int i=0; i<array_size; i++)
// 		{
// 			Serial.print(array_to_hold_data[i]);
// 			Serial.print(DELIMITER);
// 			Serial.print(" ");
// 		}
// 		Serial.println();
		
	}
	
	//clear linked list
	linkedlist_of_addresses->Clear();
	
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
void string_to_doublearray (String string_array, double storage_array[], int string_array_size, int gain, char delimeter){
	 
	  for (int i = 0; i < string_array_size ; i++)
	  {
		  String string_value = string_array.substring(0,string_array.indexOf(delimeter)); //getting first value in array

		  double value = string_value.toDouble(); //store the double value
		  storage_array[i] = value/gain; //store the value into the array


		  string_array.remove(0, (string_array.indexOf(delimeter) + 1 ) );

	  }
	
}
