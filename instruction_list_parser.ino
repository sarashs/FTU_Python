#include <ArduinoJson.h> //make sure the ArduinoJSON library is installed
#include <RTCZero.h> //this is the RTCZero library which will be used to track time
#include <SPI.h>
#include <WiFi101.h>
//#include <WiFiNINA.h> //Include this instead of WiFi101.h as needed
#include <WiFiUdp.h>

String received_string;
const int output_json_capacity = JSON_ARRAY_SIZE(3) + 3*JSON_OBJECT_SIZE(2);
char output[256]; 
volatile int adc_data[30]; //array containing 29 adc values + 1 timestamp
RTCZero rtc; //create an rtc object

//#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "";        // your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                           // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

const int GMT = -8; //change this to adapt it to your time zone

void setup() {
  // put your setup code here, to run once:
  //add json in memory
  Serial.begin(9600);

  // check if the WiFi module works
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the status:
  rtc.begin();

  unsigned long epoch;
  int numberOfTries = 0, maxTries = 6;
  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  }
  while ((epoch == 0) && (numberOfTries < maxTries));

  if (numberOfTries == maxTries) {
    Serial.print("NTP unreachable!!");
    while (1);
  }
  else {
    Serial.print("Epoch received: ");
    Serial.println(epoch);
    rtc.setEpoch(epoch);

    Serial.println();
  }
  
  while (!Serial) continue; // wait for serial port to connect. Needed for native USB port only
                            //this line allows us to not miss the serial output from the setup
  
  //char json[] = "[{\"id\": 0,\"description\": \"Apply required stress for 2 hours, etcetera\",\"apply_params\": {\"chip_temp\": 120, \"v_stress\": -0.4},\"meas_params\": {\"chip_temp\": {\"units\": \"seconds\", \"freq\": 1},\"v_th\": {\"units\": \"milliseconds\", \"freq\": 10}},\"next_cond\": {\"units\": \"hours\", \"time\": 2},\"goto_cond\": {\"id\": 2, \"cond\": \"iterations>=10\"}},{\"id\": 1,\"description\": \"Note that the description and goto_cond fields should be optional, and that the next_cond nested dict must contain either cond or units and time, exclusive of each other, and that this format should be validated.\",\"apply_params\": {\"chip_temp\": 0, \"v_stress\": 0},\"meas_params\": {\"chip_temp\": {\"units\": \"seconds\", \"freq\": 1}, \"v_th\": {\"units\": \"seconds\", \"freq\": 1}},\"next_cond\": {\"cond\": \"chip_temp<=25\"},\"goto_cond\": {\"id\": 2, \"cond\": \"iterations>=10\"}}]";
  DynamicJsonDocument input_doc(2048); //we use dynamic allocation as the documention recommends this for 1kb+ sized files
  StaticJsonDocument<output_json_capacity> output_doc; //since we know the size of our output, we statically allocate
  
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
  
  DeserializationError err = deserializeJson(input_doc, received_string); //parse the json. this throws an error if the parsing 
                                                                          //was unsuccessful
 
  //check if the parse succeeded
  if (err) {
    Serial.print(F("deserializeJson() returned: ")); //save string to flash
    Serial.println(err.c_str()); //return string representation of the error
    return;
  }
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
  JsonObject first_output_element = output_doc.createNestedObject();
  JsonObject second_output_element = output_doc.createNestedObject();
  JsonObject third_output_element = output_doc.createNestedObject();

  first_output_element["key"] = "timestamp";
  first_output_element["value"] = rtc.getEpoch(); //change these pins to the proper measuring pin
                                                 
  second_output_element["key"] = "chip_temp";
  second_output_element["value"] = 1; //change these pins to the proper measuring pin
  
  third_output_element["key"] = "threshold_volt";
  third_output_element["value"] = 0.5; //change these pins to the proper measuring pin

  serializeJson(output_doc,Serial);
}

void loop() {

}
  
