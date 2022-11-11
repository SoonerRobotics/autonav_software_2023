#include <Servo.h>

// For some reason, there is a weird min/max of 40 and 140. Probably due to some issue with using Pico's
// https://github.com/irishpatrick/pico-servo

Servo motor;
void setup() {
  motor.attach(2, 1000, 2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.write(40);
  delay(500);
  motor.write(90);
  delay(500);
  motor.write(140);
  delay(500);
}