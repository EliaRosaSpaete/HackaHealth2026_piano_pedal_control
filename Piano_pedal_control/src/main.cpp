#include <Arduino.h>
#include <Servo.h>

Servo motor;
int angle = 90;
int pin_motor = 5;

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  // motor.attach(pin_motor, 1000, 2000);
  // motor.writeMicroseconds(1000);   // arm: minimum throttle
  Serial.begin(115200);
  motor.attach(pin_motor);
  motor.write(angle);
  delay(3000);
  Serial.println("Ready. Type an angle (0-180) and press Enter.");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // motor.writeMicroseconds(1100);   // low throttle
  // delay(5000);
  // motor.writeMicroseconds(1000);   // stop
  // delay(3000);
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int newAngle = input.toInt();

      if (newAngle >= 0 && newAngle <= 180) {
        angle = newAngle;
        Serial.print("New angle: ");
        Serial.println(angle);
      } else {
        Serial.println("Invalid. Enter a number from 0 to 180.");
      }
    }
  }

  motor.write(angle);
  delay(15);
}
// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}