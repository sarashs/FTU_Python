int ledPin = A1;
int incomingByte;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600); 
pinMode(ledPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
   // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H') {
      digitalWrite(ledPin, HIGH);
    }
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == 'L') {
      digitalWrite(ledPin, LOW);
    }
  }
}
