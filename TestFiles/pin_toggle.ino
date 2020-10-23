//voltage control toggle of pin connecting to DAC output
//the user enters a command and it will toggle the
String ser_chars = "" ; //initialize for incoming serial strings 
int togglepin = 7; //placeholder for pin number

void setup() {
  // put your setup code here, to run once:
  pinMode(togglepin,OUTPUT); //this will be the pin we will toggle 
  Serial.begin(9600); //baud rate is place holder
}

void serialEvent(){
  if(Serial.available()){
    ser_chars = Serial.readString();
    Serial.println(ser_chars);
  }
  
}
void loop() {
  // put your main code here, to run repeatedly:
  //upon user command toggle pin
  serialEvent();
  
  bool pinstate = digitalRead(togglepin);
  
  if(ser_chars.indexOf("TOGGLEPIN") != -1){ //if this string is entered, it won't return -1
    
    
    digitalWrite(togglepin, !pinstate); //write it to the pin
    Serial.println("Current state: " );
    Serial.println(pinstate);
    
    ser_chars.replace("TOGGLEPIN",""); //empty the string buffer to enter a new string
  }
}
