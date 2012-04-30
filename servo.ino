#include <Servo.h> 
 
Servo servo;
int inByte = 0;
int proceeding = 0;

void setup() {
//  pinMode(5, OUTPUT);
//  analogWrite(5, 200);
  servo.attach(3);
  servo.write(0);
  Serial.begin(9600);
} 
  
void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (inByte == 'G' && !proceeding) {
      proceeding = 1;
      servo.write(165);
      delay(500);
      servo.write(0);
      delay(500);
      proceeding = 0;
    }
  }
}
