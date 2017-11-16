#include <Servo.h>
#define SERVOINIT 0             // initial angle for servo to sit at (the empty hole)
#define SERVOSTEP 34            // angle to rotate servo by in order to move to next hole
#define SERVOPIN 10             // servo pin
#define PIEZOPIN 9
#define LEDPIN 8
 
Servo myservo;

void setup() 
{ 
  delay(10000);
  myservo.attach(SERVOPIN);
  myservo.write(SERVOINIT);
  delay(1000);
  myservo.write(SERVOSTEP * 2);
  delay(10000);
  myservo.write(SERVOSTEP);
  delay(10000);
  myservo.write(SERVOINIT);
  delay(20000);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  tone(PIEZOPIN, 1000, 200);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
