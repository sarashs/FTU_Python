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
	  Serial.println("Flash begins!");
	  flash.begin();
	  Serial.println("Flash begins!");
	  
	  for (int i = 0; i<10; i++)
	  {
		  	  _address = flash.getAddress(sizeof(dataOut));
		  	  Serial.print(F("Address = "));
		  	  Serial.println(_address,HEX);

		  	  Serial.print(F("Data write "));
		  	  // This function should be changed depending on the type of data being written to the flash memory
		  	  if (flash.writeStr(_address, dataOut+i)) {
			  	  Serial.println(F("successful"));
				  
				//add to memory array
				memory_addresses_linkedlist.InsertTail(_address);
				
		  	  }
		  	  else {
			  	  Serial.println(F("failed"));
				  i--;
		  	  }
	  }

  // print out all the memory addresses stored
  for (int i = 0 ; i < memory_addresses_linkedlist.GetSize(); i++)
  {
	 Serial.print("Address ");
	 Serial.print(i);
	 Serial.print(" : ");
	 uint32_t string_address = memory_addresses_linkedlist.GetAt(i);
	 Serial.println( string_address, HEX);
	 
	 //print the data out
	 String data;
	 flash.readStr(string_address, data);
	 Serial.println(data);
  }
  
  memory_addresses_linkedlist.Clear();
  

}

void loop()
{

}


/************************************************************************/
/*                PLAN                                                  */
/*
1. Create a linked list of that will have the addresses of the stored data/ or just have an empty array and as soon as an address of -1 is reached, you'll know that it is empty
2. Store the arrays into memory as I save their address into the linked list
3. Read the arrays from the memory, one by one as I clear the linked list
4. Once list is empty
*/

/************************************************************************/
/*      MEMORY STUFF                                                    */
/************************************************************************/
//Build a linked list to store addresses

//initialize a linked list
//check if the memory is full, if full -> erase chip
//store the arrays into memory as the test runs, save the addresses simultaneously
//read back the arrays into the serial port like a normal test
//clear the linked lists

