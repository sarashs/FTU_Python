
//sensor test programme initializations
int sensorPin = A0;    // select the input pin for the sensor 
int sensorValue = 0;   // variable to store the value coming from the sensor 
float temperature = 0.0;  
float voltage = 0.0; 
void setup() { 
Serial.begin(9600); 
analogReadResolution(10); //10 bit resolution
} 
void loop() { 
// read the value from the sensor: 
sensorValue = analogRead(sensorPin); 
voltage = sensorValue * (3300/1024); // in milliVolt 
//Serial.print(" voltage = "); 
//Serial.print(voltage); 
temperature = (voltage - 500 ) / 10; 
//Serial.print(" temperature(C) = "); 
Serial.println(temperature); 
delay(1000); //adjust sampling rate
} 
