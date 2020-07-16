#include <ArduinoJson.h> 
String received_string;
const int capacity = JSON_ARRAY_SIZE(3) + 3*JSON_OBJECT_SIZE(2);
//DynamicJsonDocument input_doc(2048); //we use dynamic allocation as the documention recommends this for 1kb+ sized files
StaticJsonDocument<capacity> output_doc; //since we know the size of our output, we statically allocate
  
void setup() {
  // put your setup code here, to run once:
  //add json in memory
  Serial.begin(9600);
  while (!Serial) continue; // wait for serial port to connect. Needed for native USB port only
                            //this line allows us to not miss the serial output from the setup
  
  //char json[] = "[{\"id\": 0,\"description\": \"Apply required stress for 2 hours, etcetera\",\"apply_params\": {\"chip_temp\": 120, \"v_stress\": -0.4},\"meas_params\": {\"chip_temp\": {\"units\": \"seconds\", \"freq\": 1},\"v_th\": {\"units\": \"milliseconds\", \"freq\": 10}},\"next_cond\": {\"units\": \"hours\", \"time\": 2},\"goto_cond\": {\"id\": 2, \"cond\": \"iterations>=10\"}},{\"id\": 1,\"description\": \"Note that the description and goto_cond fields should be optional, and that the next_cond nested dict must contain either cond or units and time, exclusive of each other, and that this format should be validated.\",\"apply_params\": {\"chip_temp\": 0, \"v_stress\": 0},\"meas_params\": {\"chip_temp\": {\"units\": \"seconds\", \"freq\": 1}, \"v_th\": {\"units\": \"seconds\", \"freq\": 1}},\"next_cond\": {\"cond\": \"chip_temp<=25\"},\"goto_cond\": {\"id\": 2, \"cond\": \"iterations>=10\"}}]";
  DynamicJsonDocument input_doc(2048); //we use dynamic allocation as the documention recommends this for 1kb+ sized files

  while(!Serial.available()){} 

  while(Serial.available()){
    if(Serial.available()>0){ //if there's any characters, read it
      received_string = Serial.readString(); 
    }
  if(received_string.length() > 0){
    Serial.print("Arduino received: "); //send received string back to python
    Serial.println(received_string);
  }
  }
  Serial.flush(); //Waits for the transmission of outgoing serial data to complete
  
  //Serial.setTimeout(5000);
  Serial.println("r_string is:");
  Serial.println(received_string);
  
  DeserializationError err = deserializeJson(input_doc, received_string); //parse the json
  
  //check if the parse succeeded
  if (err) {
    Serial.print(F("deserializeJson() returned: ")); //save string to flash
    Serial.println(err.c_str()); //return string representation of the error
    return;
  }

  Serial.print("parsing success");
  //extract params
  JsonObject first_input_element = input_doc[0];
  int id = first_input_element["id"];
  const char* description = first_input_element["description"];
  int applied_temp = first_input_element["meas_params"]["chip_temp"]["freq"];
  float applied_stress = first_input_element["apply_params"]["v_stress"];

  //use params
  Serial.println(id);
  Serial.println(description);
  Serial.println(applied_temp);
  Serial.println(applied_stress);
  
//  //MCU does the conversions
//  
//  //create output nested objects and append them to the array
//  JsonObject first_output_element = output_doc.createNestedObject();
//  JsonObject second_output_element = output_doc.createNestedObject();
//  JsonObject third_output_element = output_doc.createNestedObject();
//
//  first_output_element["key"] = "timestamp";
//  first_output_element["value"] = 12; //change these pins to the proper measuring pin
//                                                  //this is just a placeholder. change later.
//  second_output_element["key"] = "chip_temp";
//  second_output_element["value"] = 13; //change these pins to the proper measuring pin
//  
//  third_output_element["key"] = "threshold_volt";
//  third_output_element["value"] = 0.5; //change these pins to the proper measuring pin
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!Serial.available()){} 

  while(Serial.available()){
    if(Serial.available()>0){ //if there's any characters, read it
      received_string = Serial.readString(); 
    }
  if(received_string.length() > 0){
    Serial.print("Arduino received: "); //send received string back to python
    Serial.println(received_string);
  }
  }
  Serial.flush(); //Waits for the transmission of outgoing serial data to complete
}
  
