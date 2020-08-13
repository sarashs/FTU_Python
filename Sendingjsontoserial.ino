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
  String clearjson="clear";
  String displayjson="data";
  String message;
  float ADCarray[29] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28};
  int test_data[3] = {135,50,2};

  //test
  String TEST_ID = "Test52";
  int TEST_RUN = 0;
  int TEST_ERROR = 0;


void setup() {
  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial) continue; 
  Serial.println("connected");
}

void loop() {
// not used in this example 
while (Serial.available()==0){ //wait for something at serial
  }
message = Serial.readStringUntil('\n');
Serial.print("The command you have entered is : '");
Serial.print(message); 
Serial.println("'");

if (message.equals(clearjson)){
doc.clear();
doc["message"] = "document is cleared";
Serial.println();
serializeJson(doc,Serial);
Serial.println();

}
if (message.equals(displayjson)){
  doc.clear(); //clear the document as this frees up the memory
  // Add values to the document
  doc["test id"] = TEST_ID;
  doc["test run"] = TEST_RUN; //0 test is not running, 1 test is running
  doc["test error"] = TEST_ERROR; //0 no error, 1 there was an error

  // Add an array.
  JsonArray ADCdata = doc.createNestedArray("ADC data");
  for (int i =0; i < 29; i++){
    ADCdata.add(ADCarray[i]);
    }
  //add another array
  JsonArray TESTdata = doc.createNestedArray("test data");
  for (int i =0; i < 3; i++){
    TESTdata.add(test_data[i]);
    }
  //data.add(48.756080);
  //data.add(2.302038);

  // Generate the minified JSON and send it to the Serial port.
  //
  serializeJson(doc, Serial);
  // The above line prints:
  // {"sensor":"gps","time":1351824120,"data":[48.756080,2.302038]}
  //serializeJsonPretty(doc, Serial);
  

  // Start a new line
  Serial.println();
  // Generate the prettified JSON and send it to the Serial port.
  //
  Serial.print("The memory used is ");
  Serial.println(doc.memoryUsage());

      }
}
