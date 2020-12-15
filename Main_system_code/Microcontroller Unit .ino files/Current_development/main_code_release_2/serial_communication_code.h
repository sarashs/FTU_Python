#pragma once

#define BAUD_RATE 2000000 //9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000, 1000000, 2000000
#define number_of_adc_arrays 1
/************************************************************************/
/* USING JSON LIBRARY TO SEND COMMUNICATE WITH PYTHON SCRIPT            */
/************************************************************************/
/**
* @see Original source : "https://arduinojson.org/"
* preparing the JSON document to be used in the test
* Allocate the JSON document

* StaticJsonObject allocates memory on the stack, it can be
* replaced by DynamicJsonDocument which allocates in the heap.
*1024 is the RAM dedicated to this document 
*/
DynamicJsonDocument  doc(800*number_of_adc_arrays); 

/**
 * \@brief Function to receive test instructions from CPU 
 * 
 * \@param 
 * 
 * \@return void
 */
void receive_test_instructions(void){
	String receieved_instruction;
	String instruction = "instruction";  //This will be a json instruction in string format from the CPU Python script
	
	while (Serial.available()==0){ //wait for something at serial
	}
	receieved_instruction = Serial.readStringUntil('\n'); //read till end of the line

	while (Serial.available()==0){ //wait for instruction json at serial
		if (receieved_instruction.equals(instruction)){
			Serial.println("ready");
			delay(20);
		}
	}
	
	receieved_instruction = Serial.readStringUntil('\n'); //
	Serial.println(receieved_instruction); //send json back as string to to check if it is correct
	
	//change string to char array for JSON buffer simplicity
	char charBuf[receieved_instruction.length() + 1];
	receieved_instruction.toCharArray(charBuf, receieved_instruction.length()+1);

	// De serialize the JSON document
	DeserializationError error = deserializeJson(doc, charBuf);

	// Test if parsing succeeds.
	if (error) {
		//raise error
		raise_mcu_error(error.c_str());
		return;
	}

	test_id = doc["id"]; // 0
	const char* description = doc["description"]; // "Apply required stress for 2 hours, etcetera"

	//Setting test parameters
	JsonObject test_values = doc["test_values"];
	desired_temperature = test_values["temperature"]; // 120
	desired_fpga_voltage = test_values["v_stress"]; // -400
	desired_time_for_test = test_values["test_time"]; // 5
	desired_magnetic_field = test_values["magnetic_field"]; // 5
	if (test_values["Test_start"] == 1) test_start= true; // 1
	if (test_values["Test_stop"]==1) test_stop = true; // 0
	serial_output_rate = test_values["serial_rate"]; // 1500
	
	if (test_values["High speed test"]==1) high_speed_test = true; // 0

	JsonObject measurement_params = doc["measurement_params"];

	const char* measurement_params_temperature_unit = measurement_params["temperature"]["unit"]; // "C"
	const char* measurement_params_v_stress_unit = measurement_params["v_stress"]["unit"]; // "mV"
	const char* measurement_params_test_time_unit = measurement_params["test_time"]["unit"]; // "seconds"
	const char* measurement_params_magnetic_field_unit = measurement_params["magnetic_field"]["unit"]; // "mT"
	const char* measurement_params_serial_rate_unit = measurement_params["serial_rate"]["unit"]; // "milliseconds"
	
}


/**
 * \@brief This function prepares the JSON document that is to be sent to the serial
 * 
 * \@param test_id
 * \@param test_stop
 * \@param test_start
 * \@param test_error
 * \@param error_message
 * \@param adc_data
 * \@param test_time
 * \@param temperature
 * \@param measured_magnetic_field
 * 
 * \@return void
 */
void update_json_doc(int test_id, bool test_stop, bool test_start, bool test_error, String error_message, float test_time, float temperature, float magnetic_field,
double adc_data[],int array_size = ADC_ARRAY_SIZE, int num_adc_array = number_of_adc_arrays){
	//Preparing json file
	String str_testid = "test id";
	String str_teststop = "test stop";
	String str_testeror = "test error";
	String str_errormessage = "error message";
	String str_ADCdatastring = "ADC data";
	String str_testdata = "test data";
				
	doc.clear(); //clear the document as this frees up the memory
	// Add values to the document

	
	doc[str_testid] = test_id;
	//doc["test stop"] = TEST_STOP; //1 test is stopped, 0 test is running
	if (test_stop) doc[str_teststop] = 1;
	else doc[str_teststop] = 0;
	//doc["test error"] = TEST_ERROR; //0 no error, 1 there was an error
	if (test_error) doc[str_testeror]=1;
	else doc[str_testeror]=0;
				
	doc[str_errormessage] = error_message;
	doc["number of adc arrays"] = num_adc_array;
	
	for (int i=0;i<num_adc_array;i++){
		//send multiple arrays in adc doc
		str_ADCdatastring.concat(String(i));
		str_testdata.concat(String(i));
			
		// Add an array.
		JsonArray ADCdata = doc.createNestedArray(str_ADCdatastring);
		for (int i =8; i < array_size ; i++){ //starting from index 8 as that is how much we use in the ADC array, this is to save space
			ADCdata.add(adc_data[i]);
		}
		//add another array
		JsonArray TESTdata = doc.createNestedArray(str_testdata);

		TESTdata.add(test_time); //current test time
		TESTdata.add(temperature); //temperature (C)
		TESTdata.add(magnetic_field); //magnetic field (mT)
		//out current_test_timer, ADC converted data at intervals
		
		str_testid.remove(str_testid.lastIndexOf(String(i)));
		str_teststop.remove(str_teststop.lastIndexOf(String(i)));
		str_testeror.remove(str_testeror.lastIndexOf(String(i)));
		str_errormessage.remove(str_errormessage.lastIndexOf(String(i)));
		str_ADCdatastring.remove(str_ADCdatastring.lastIndexOf(String(i)));
		str_testdata.remove(str_testdata.lastIndexOf(String(i)));
	}
	
	return;
}

/**
 * \@brief  Function to send data to Python script
 *			tell python there is ready data
 *			send the data
 *			wait for confirmation
 * 
 * 
 * \@return void
 */
void send_data_to_serial(){
	char userInput;
	Serial.begin(BAUD_RATE); //9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000
	//printing the document
	serializeJson(doc, Serial);
	Serial.flush();
	Serial.write("\n");
	Serial.flush();
	Serial.end();

}