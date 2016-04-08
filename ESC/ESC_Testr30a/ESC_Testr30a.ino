// Test for the ESC

#include <Servo.h>
Servo esc;
int throttlePin = 0;
char dir=0;
int throttle=0;


void setup() {
  Serial.begin(9600);
  esc.attach(3);
}

void loop() {
  throttle=1460;
  Serial.println(throttle);
  esc.writeMicroseconds(throttle);
  delay(2000);
  throttle=1860;
  Serial.println(throttle);
  esc.writeMicroseconds(throttle);
  delay(5000);
  throttle=1460;
  Serial.println(throttle);
  esc.writeMicroseconds(throttle);
  delay(2000);
  throttle=1060;
  Serial.println(throttle);
  esc.writeMicroseconds(throttle);
  delay(5000);

}
