#include "FunctionFSM.h"

const float thresh_temp = 50.0;
float current_temp; //get this temp from the sensor

enum Trigger {
  //check temp???
  START_HEAT;
  CHECK_TEMP;
};

void heater_idle(){
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDR,LOW);
  digitalWrite(LEDB,HIGH);
}

void heater_heating_safe(){ //pwm starts
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDR,HIGH);
  digitalWrite(LEDB,HIGH);
}

void heater_heating_unsafe(){
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDR,HIGH);
  digitalWrite(LEDB,LOW);
}
 

void heater_max_idle(){
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDR,HIGH);
  digitalWrite(LEDB,LOW);
}

void heater_cooling_unsafe(){ //pwm stops
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDR,LOW);
  digitalWrite(LEDB,LOW);
}


//fsm state declaration
FunctionState heat_idle(&heater_idle, nullptr, nullptr);
FunctionState heat_heating_safe(&heater_heating_safe, nullptr, nullptr);
FunctionState heat_heating_unsafe(&heater_heating_unsafe, nullptr, nullptr);
FunctionState heat_max_idle(&heater_max_idle, nullptr, nullptr);
FunctionState heat_cooling_unsafe(&heater_cooling_unsafe, nullptr, nullptr);

//first state
FunctionFsm fsm(&heat_idle);

//state transition logic
void initfsm(){
  fsm.add_transition(&heat_idle, &heat_heating_safe, START_HEAT, nullptr);
  if(current_temp < thresh_temp){ 
    fsm.add_transition(&heat_heating_safe, &heat_heating_unsafe, CHECK_TEMP, nullptr);
  }
  if(current_temp > thresh_temp){
    fsm.add_transition(&heat_heating_unsafe, &heat_max_idle, CHECK_TEMP, nullptr);
  }

  while(current_temp != max_temp){
    fsm.add_transition(&heat_max_idle, &heat_cooling_unsafe, SWITCH, nullptr);
  }

  while(current_temp >= max_temp){
    fsm.add_transition(&heat_max_idle, &heat_cooling_unsafe, SWITCH, nullptr);
  }

  if(current_temp < thresh_temp){
    fsm.add_transition(&heat_cooling_unsafe, &heat_idle, SWITCH, nullptr);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.flush();
  
  pinmode(LEDG,OUTPUT);
  pinmode(LEDB,OUTPUT);
  pinmode(LEDR,OUTPUT);
  //init fsm
  initfsm();
}


void loop() {
  // put your main code here, to run repeatedly:
  fsm.run_machine();
  if(Serial.available()){
    Serial.println("\r\n");
    Serial.read();
  }
}
