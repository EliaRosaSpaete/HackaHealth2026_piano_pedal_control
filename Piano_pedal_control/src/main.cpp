#include <Arduino.h>
#include <Servo.h>
#include "Trigger.h"

// ---- Wiring ----
// Motor (BLS3235, 3 wires):
//   GND    (black/brown) -> GND (must be shared with the Arduino GND)
//   Power  (red)         -> external supply as per motor datasheet (do NOT power
//                           from the Arduino pin if the motor draws a lot of current)
//   Data   (yellow/white)-> PIN_MOTOR (5)
//
// Endstop switch (B59, 3 terminals: COM, NC, NO) - used with SwitchTrigger:
//   COM (common) -> GND
//   NO (normally open)   -> PIN_SWITCH (2)   pressed = pin pulled LOW
//   NC (normally closed) -> not connected
//   (Using NC instead would invert the logic: pin LOW while NOT pressed.)
//
// Force sensor (FSR402, 2 connectors, polarity does not matter) - used with FsrTrigger:
//   Pin 1 -> 5V (3.3V on 3.3V boards)
//   Pin 2 -> PIN_FSR (A0) AND through a ~10k resistor to GND (voltage divider)

// ---- Configuration ----
const uint8_t PIN_MOTOR = 5;
const uint8_t PIN_SWITCH = 2;   // endstop switch (B59), see wiring above
const uint8_t PIN_FSR = A0;     // FSR402 divider output, see wiring above

const int NEUTRAL_ANGLE = 0;   // motor position when not triggered
const int PRESSED_ANGLE = 180;  // predefined angle X when triggered

// ---- Trigger selection: swap this line to change the sensor ----
// SwitchTrigger switchTrigger(PIN_SWITCH);
const int FSR_PRESSED_BELOW = 20;    // filtered reading below this = pressed (0-1023)
const int FSR_RELEASED_ABOVE = 50;   // filtered reading above this = released (keep higher than PRESSED_BELOW)
const float FSR_CUTOFF_HZ = 20.0;    // low-pass filter cutoff frequency
const unsigned long FSR_WINDOW_MS = 200;  // filter uses the samples of the last 200 ms
FsrTrigger fsrTrigger(PIN_FSR, FSR_PRESSED_BELOW, FSR_RELEASED_ABOVE, FSR_CUTOFF_HZ, FSR_WINDOW_MS);
// Trigger &trigger = switchTrigger;
Trigger &trigger = fsrTrigger;

Servo motor;

void setup() {
  Serial.begin(115200);
  trigger.begin();
  motor.attach(PIN_MOTOR);
  motor.write(NEUTRAL_ANGLE);
  delay(3000);  // let the motor settle
  Serial.println("Ready.");
}

void loop() {
  static unsigned long lastPrint = 0;
  
  bool pressed = trigger.isActive();
  int targetAngle = pressed ? PRESSED_ANGLE : NEUTRAL_ANGLE;
  motor.write(targetAngle);

  // Print status every 800 ms
  if (millis() - lastPrint >= 800) {
    lastPrint = millis();
    Serial.print("FSR raw: ");
    Serial.print(analogRead(PIN_FSR));
    Serial.print(" | FSR filtered: ");
    Serial.print(fsrTrigger.filtered());
    Serial.print(" | Trigger: ");
    Serial.print(pressed ? "PRESSED" : "RELEASED");
    Serial.print(" | Motor target angle: ");
    Serial.print(targetAngle);
    Serial.print(" | Motor position: ");
    Serial.println(motor.read());
  }

  delay(15);
}
