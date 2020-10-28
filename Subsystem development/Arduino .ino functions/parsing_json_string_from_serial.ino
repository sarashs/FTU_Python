//Author : Valentine Ssebuyungo
//Date   : 12th Aug 2020
#include <ArduinoJson.h>
#include <string.h>

// Allocate the JSON document
//
// StaticJsonObject allocates memory on the stack, it can be
// replaced by DynamicJsonDocument which allocates in the heap.
//
// DynamicJsonDocument  doc(200);
//StaticJsonDocument<200> doc;
// Inside the brackets, 200 is the RAM allocated to this document.
// Don't forget to change this value to match your requirement.
// Use arduinojson.org/v6/assistant to compute the capacity.
DynamicJsonDocument  doc(1024);

//Testing variables
String message;

void setup() {
  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial) continue; 
  Serial.println("connected");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while (Serial.available()==0){ //wait for something at serial
    }
    
  message = Serial.readStringUntil('\n');
  
  //change string to char array for json buffer simplicity
  char charBuf[message.length() + 1];
  message.toCharArray(charBuf, message.length()+1);
  
  Serial.println(charBuf);

  // Deserialize the JSON document
   DeserializationError error = deserializeJson(doc, charBuf);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    //raise error
    return;
  }

  // Fetch values.
  //you can use this to test if it is the same object 
  //serializeJson(doc,Serial);
  
//  String Test_ID = doc["Test_ID"];
//  String Test_Description = doc["Test_Description"];
//  int Test_start = doc["data"][0];
//  int Test_stop = doc["data"][1];
//  int FPGA_voltage = doc["data"][2];
//  float magnetic_field = doc["data"][3];
//  int temperature = doc["data"][4];
//  int test_time_minutes = doc["data"][5];
//  int serial_rate = doc["data"][6];
//
//  // Print values.
//  Serial.println(Test_ID);
//  Serial.println(Test_Description);
//  Serial.println(Test_start);
//  Serial.println(Test_stop);
//  Serial.println(FPGA_voltage);
//  Serial.println(magnetic_field,4);
//  Serial.println(temperature);
//  Serial.println(test_time_minutes);
//  Serial.println(serial_rate);

int id = doc["id"]; // 0
const char* description = doc["description"]; // "Apply required stress for 2 hours, etcetera"

JsonObject test_values = doc["test_values"];
int test_values_temperature = test_values["temperature"]; // 120
int test_values_v_stress = test_values["v_stress"]; // -400
int test_values_test_time = test_values["test_time"]; // 5
int test_values_magnetic_field = test_values["magnetic_field"]; // 5
int test_values_Test_start = test_values["Test_start"]; // 1
int test_values_Test_stop = test_values["Test_stop"]; // 0
int test_values_serial_rate = test_values["serial_rate"]; // 1500

JsonObject measurement_params = doc["measurement_params"];

const char* measurement_params_temperature_unit = measurement_params["temperature"]["unit"]; // "C"

const char* measurement_params_v_stress_unit = measurement_params["v_stress"]["unit"]; // "mV"

const char* measurement_params_test_time_unit = measurement_params["test_time"]["unit"]; // "seconds"

const char* measurement_params_magnetic_field_unit = measurement_params["magnetic_field"]["unit"]; // "mT"

const char* measurement_params_serial_rate_unit = measurement_params["serial_rate"]["unit"]; // "milliseconds"

Serial.print("test_values_temperature: ");
Serial.println(test_values_temperature);

Serial.print("test_values_v_stress: ");
Serial.println(test_values_v_stress);

Serial.print("test_values_test_time: ");
Serial.println(test_values_test_time);

Serial.print("test_values_magnetic_field: ");
Serial.println(test_values_magnetic_field);

Serial.print("test_values_Test_start: ");
Serial.println(test_values_Test_start);

Serial.print("test_values_Test_stop: ");
Serial.println(test_values_Test_stop);

Serial.print("test_values_serial_rate: ");
Serial.println(test_values_serial_rate);

}


//file sent over serial
//{"id": 0,"description": "Apply required stress for 2 hours, etcetera","test_values": {"temperature": 120, "v_stress": -400, "test_time": 5, "magnetic_field": 5, "Test_start": 1, "Test_stop":0, "serial_rate": 1500},"measurement_params": {"temperature": {"unit": "C"},"v_stress": {"unit": "mV"},"test_time": {"unit": "seconds"},"magnetic_field": {"unit": "mT"},"serial_rate": {"unit": "milliseconds"}}}
//{"Test_ID":"ABC123456","Test_Description":"testing_FUT","data":[1,0,-1000,5,55,20,1500]}
