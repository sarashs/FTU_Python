const float thresh_temp = 50.0;
const float max_temp = 150.0;

int incomingByte;
int LEDG = 8;
int LEDR = 9;
int LEDB = 11; 
enum State_enum {HEATER_IDLE,HEATER_HEATING_SAFE,HEATER_HEATING_UNSAFE,HEATER_MAX_IDLE,HEATER_COOLING_UNSAFE};

//function declaration
void heater_fsm(uint16_t current_temp);
void heater_idle();
void heater_heating_safe();
void heater_heating_unsafe();
void heater_max_idle();
void heater_cooling_unsafe();

uint16_t read_temp();
uint16_t apply_pwm();
uint16_t stop_pwm();

//state initialize
uint8_t state = HEATER_IDLE;

//state transition
void heater_fsm(uint16_t current_temp) {
  
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
  } 
  
  switch(state){
    case HEATER_IDLE:
      if(incomingByte == 'S'){                  //user input from GUI or serial
        state = HEATER_HEATING_SAFE;
      }
      else{
        state = HEATER_IDLE;
      }
      break;
      
    case HEATER_HEATING_SAFE:
      if(current_temp < thresh_temp){                  
        state = HEATER_HEATING_SAFE;
      }
      else{
        state = HEATER_HEATING_UNSAFE;
      }
      break;
      
    case HEATER_HEATING_UNSAFE:
      if(current_temp == max_temp){                  
        state = HEATER_MAX_IDLE;
      }
      else{
        state = HEATER_HEATING_UNSAFE;
      }
      break;
      
    case HEATER_MAX_IDLE:
      if(incomingByte == 'G'){    
        state = HEATER_COOLING_UNSAFE;
      }
      else{
        state = HEATER_MAX_IDLE;
      }
      break;
      
    case HEATER_COOLING_UNSAFE:
      if(current_temp < thresh_temp){                  
        state = HEATER_IDLE;
      }
      else{
        state = HEATER_COOLING_UNSAFE;
      }
      break;
  }
}

  
  void heater_idle(){
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,LOW);
    digitalWrite(LEDB,HIGH);
  }
  
  void heater_heating_safe(){ //pwm starts
    apply_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDB,HIGH);
  }
  
  void heater_heating_unsafe(){
    apply_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDB,LOW);
  }
   
  
  void heater_max_idle(){
    apply_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDB,LOW);
  }
  
  void heater_cooling_unsafe(){ //pwm stops
    stop_pwm();
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDR,LOW);
    digitalWrite(LEDB,LOW);
  }
  
  uint16_t read_temp(){
    //read temp sensor
    //0.5V/50C
    analogReadResolution(16);
    volt = analogRead(A0);
    mvolt = volt*1000/65535
    temp = (mvolt-500)/10;
    return temp;
  }


  uint16_t apply_pwm(){
    //apply pwm signal
  }
  
  uint16_t stop_pwm(){
    //stop pwm signal
  }
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  pinMode(LEDG,OUTPUT);
  pinMode(LEDB,OUTPUT);
  pinMode(LEDR,OUTPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  heater_fsm(read_temp()); //check_temp for fsm changes
  delay(10);
}
