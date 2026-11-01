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
//
// Potentiometers (3 pins each, wired the same way):
//   Outer pin 1 -> GND
//   Outer pin 2 -> 5V (3.3V on 3.3V boards)
//   Middle pin (wiper) -> PIN_POT_FSR (A1) or PIN_POT_ANGLE (A2)
//   Reading: 0-1023 (swapping the outer pins reverses the direction)
//   Both signals are low-pass filtered (POT_CUTOFF_HZ) before use.
//   PIN_POT_FSR (A1):   sets the centre of the FSR threshold band (pressed above / released below)
//                       linearly between FSR_CENTER_MIN (100) and FSR_CENTER_MAX (400); the gap
//                       between the two thresholds stays fixed.
//   PIN_POT_ANGLE (A2): sets the motor angle used when pressed, linearly between
//                       PRESSED_ANGLE_MIN (90) and PRESSED_ANGLE_MAX (180) degrees.
//
// Serial output (115200 baud, every 800 ms, 3 lines + blank line):
//   1: FSR raw/filtered, trigger state, current FSR thresholds
//   2: raw/filtered value of the FSR potentiometer and of the angle potentiometer
//   3: resulting pressed angle, motor target angle, motor position

// ---- Configuration ----
const uint8_t PIN_MOTOR = 5;
const uint8_t PIN_SWITCH = 2;   // endstop switch (B59), see wiring above
const uint8_t PIN_FSR = A0;     // FSR402 divider output, see wiring above
const uint8_t PIN_POT_FSR = A1;   // potentiometer wiper for the FSR thresholds
const uint8_t PIN_POT_ANGLE = A2; // potentiometer wiper for the pressed angle

const int NEUTRAL_ANGLE = 0;   // motor position when not triggered
// Pressed angle X (when triggered) is set by the angle potentiometer: full left = MIN, full right = MAX
const int PRESSED_ANGLE_MIN = 90;
const int PRESSED_ANGLE_MAX = 180;

// ---- Trigger selection: swap this line to change the sensor ----
// SwitchTrigger switchTrigger(PIN_SWITCH);
const int FSR_PRESSED_ABOVE = 130;  // filtered reading above this = pressed (0-1023)
const int FSR_RELEASED_BELOW = 80;  // filtered reading below this = released (keep lower than PRESSED_ABOVE)
const float FSR_CUTOFF_HZ = 20.0;    // low-pass filter cutoff frequency
const unsigned long FSR_WINDOW_MS = 200;  // filter uses the samples of the last 200 ms
FsrTrigger fsrTrigger(PIN_FSR, FSR_PRESSED_ABOVE, FSR_RELEASED_BELOW, FSR_CUTOFF_HZ, FSR_WINDOW_MS);
// Trigger &trigger = switchTrigger;
Trigger &trigger = fsrTrigger;

// The potentiometer sets the centre of the FSR hysteresis band directly (absolute value, 0-1023 scale).
// Full left = FSR_CENTER_MIN, full right = FSR_CENTER_MAX. The gap between the two thresholds stays
// the one defined above (FSR_PRESSED_ABOVE - FSR_RELEASED_BELOW).
const int FSR_CENTER_MIN = 100;
const int FSR_CENTER_MAX = 400;
const float POT_CUTOFF_HZ = 2.0;  // low-pass cutoff for the potentiometer signals (removes ADC noise)

// Reads an analog pin through a first-order, time-based low-pass filter (like the FSR filter).
struct FilteredPot {
  uint8_t pin;
  float filtered = -1;  // < 0 = not initialised yet
  unsigned long lastTime = 0;
  int raw = 0;

  explicit FilteredPot(uint8_t p) : pin(p) {}

  float update() {
    unsigned long now = millis();
    raw = analogRead(pin);
    if (filtered < 0) {
      filtered = raw;
    } else {
      float dt = (now - lastTime) / 1000.0f;
      float rc = 1.0f / (2.0f * PI * POT_CUTOFF_HZ);
      filtered += dt / (rc + dt) * (raw - filtered);
    }
    lastTime = now;
    return filtered;
  }
};

FilteredPot potFsr(PIN_POT_FSR);
FilteredPot potAngle(PIN_POT_ANGLE);

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

  // Set the FSR threshold band from the filtered potentiometer value
  const int fsrGap = FSR_PRESSED_ABOVE - FSR_RELEASED_BELOW;
  int center = map((long)potFsr.update(), 0, 1023, FSR_CENTER_MIN, FSR_CENTER_MAX);
  int releasedBelow = constrain(center - fsrGap / 2, 0, 1023 - fsrGap);  // keep the band within 0-1023
  int pressedAbove = releasedBelow + fsrGap;
  fsrTrigger.setThresholds(pressedAbove, releasedBelow);

  // Set the pressed angle from the filtered angle potentiometer value
  int pressedAngle = map((long)potAngle.update(), 0, 1023, PRESSED_ANGLE_MIN, PRESSED_ANGLE_MAX);

  bool pressed = trigger.isActive();
  int targetAngle = pressed ? pressedAngle : NEUTRAL_ANGLE;
  motor.write(targetAngle);

  // // Debugging: Print status every 800 ms
  // if (millis() - lastPrint >= 800) {
  //   lastPrint = millis();
  //   // Line 1: force sensor and thresholds (set by the FSR potentiometer)
  //   Serial.print("FSR raw: ");
  //   Serial.print(analogRead(PIN_FSR));
  //   Serial.print(" | filtered: ");
  //   Serial.print(fsrTrigger.filtered());
  //   Serial.print(" | Trigger: ");
  //   Serial.print(pressed ? "PRESSED" : "RELEASED");
  //   Serial.print(" | Pressed above: ");
  //   Serial.print(pressedAbove);
  //   Serial.print(" | Released below: ");
  //   Serial.println(releasedBelow);

  //   // Line 2: both potentiometers
  //   Serial.print("Pot FSR raw: ");
  //   Serial.print(potFsr.raw);
  //   Serial.print(" | filtered: ");
  //   Serial.print(potFsr.filtered);
  //   Serial.print(" || Pot angle raw: ");
  //   Serial.print(potAngle.raw);
  //   Serial.print(" | filtered: ");
  //   Serial.println(potAngle.filtered);

  //   // Line 3: motor (pressed angle is set by the angle potentiometer)
  //   Serial.print("Pressed angle: ");
  //   Serial.print(pressedAngle);
  //   Serial.print(" | Motor target angle: ");
  //   Serial.print(targetAngle);
  //   Serial.print(" | Motor position: ");
  //   Serial.println(motor.read());
  //   Serial.println();
  // }

  delay(15);
}
