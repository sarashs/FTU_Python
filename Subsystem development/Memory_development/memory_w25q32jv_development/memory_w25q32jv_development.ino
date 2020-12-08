#include <LinkedListLib.h> //https://github.com/luisllamasbinaburo/Arduino-LinkedList
#include <SPIMemory.h>    //See https://github.com/Marzogh/SPIMemory
/*
 * memory_w25q32jv_development.ino
 *
 * Created: 12/1/2020 4:23:14 PM
 * Author: Valentine Ssebuyungo
 */ 


#define  csPin  0  
// Create LinkedList
LinkedList<uint32_t> memory_addresses_linkedlist;

SPIFlash flash(csPin);
uint32_t _address;
// This data type should be changed depending on the type of data being write to the flash memory
String dataOut = "This is a test String!";

void setup()
{
		
	/* add setup code here, setup code runs once when the processor starts */
	Serial.begin(9600);
	delay(1000);
	
	double data_converted[] = {4.22344,1.34345454,4.4566,2.7787};
	char delimeter = ',';
	int size_data = 4;
	double dataOut[size_data];
	int gain  = 100000;
	
	trial( &memory_addresses_linkedlist );
	Serial.print("Size : ");
	Serial.println(memory_addresses_linkedlist.GetSize());
	
	Serial.println("Values :");
	for (int i = 0; i < memory_addresses_linkedlist.GetSize(); i++)
	{
		Serial.println(memory_addresses_linkedlist.GetAt(i));
		
	}
	memory_addresses_linkedlist.Clear();


	  

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
	flash.begin();
	if (flash.getAddress(20000*sizeof(double)) == false) //check if we can store at least 20,000 values
	{
		//flash.eraseChip(); //if chip is full,erase it
	}
}

void memory_store_array_function (double array_to_store[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses){
	//convert array to string
	char delimeter = ','; //randomly chose commas to separate each value
	int gain  = 100000; //10^5 so that we can store up to 5 decimal places
	
	
	String string_to_store = double_array_to_string(array_to_store,array_size,gain,delimeter);
	//store the data_String in memory and add the string address to a linked list
		  
	while(true){
		_address = flash.getAddress(sizeof(string_to_store));
		// This function should be changed depending on the type of data being written to the flash memory
		if (flash.writeStr(_address,string_to_store)) {
				  
			//Serial.println(F("successful"));
				  
			linkedlist_of_addresses->InsertTail(_address); //save the string address at the end of the linked list
			break; //get out of while loop
		}
		else {
			//raise an error!
		}
	}
}

/**
 * \brief Reads the arrays stored in memory one by one
 *        The memory addresses are store in the linked list
 *        Reads of the head of the linked list(stack) and pops it off
 * 
 * \param array_to_hold_data
 * \param array_size
 * \param linkedlist_of_addresses
 * 
 * \return void
 */
void memory_retrieve_array_function(double array_to_hold_data[], int array_size, LinkedList<uint32_t> *linkedlist_of_addresses){
	
}

void trial(LinkedList<uint32_t> *trial){
	for (int i=0; i<100; i++)
	{
		trial->InsertTail(i);
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