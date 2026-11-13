#include <Arduino.h>
#include <Servo.h>
#include "Trigger.h"

// Piano pedal control: a force sensor (FSR) under the foot triggers a servo motor
// that presses the pedal. Two potentiometers tune the system:
//   A1 (PIN_POT_FSR):   sensitivity, i.e. the press/release thresholds of the FSR
//   A2 (PIN_POT_ANGLE): motor angle when pressed
// Wiring, tuning and serial output are described in README.md.

// ---- Configuration ----
const uint8_t PIN_MOTOR = 5;
const uint8_t PIN_FSR = A0;     // FSR402 divider output, see README.md
const uint8_t PIN_POT_FSR = A1;   // potentiometer wiper for the FSR thresholds
const uint8_t PIN_POT_ANGLE = A2; // potentiometer wiper for the pressed angle

const int NEUTRAL_ANGLE = 0;   // motor position when not triggered
// Pressed angle X (when triggered) is set by the angle potentiometer: full left = MIN, full right = MAX
const int PRESSED_ANGLE_MIN = 90;
const int PRESSED_ANGLE_MAX = 180;

// Hysteresis band of the FSR (0-1023 scale). The pot (A1) moves the band's centre between
// FSR_CENTER_MIN and FSR_CENTER_MAX; FSR_GAP is the distance between the two thresholds:
//   released below = centre - FSR_GAP/2,  pressed above = centre + FSR_GAP/2
// -> Tune FSR_CENTER_MIN / FSR_CENTER_MAX per mechanical construction (see README.md).
const int FSR_CENTER_MIN = 200;
const int FSR_CENTER_MAX = 700;
const int FSR_GAP = 40;
const float FSR_CUTOFF_HZ = 20.0;    // low-pass filter cutoff frequency
const unsigned long FSR_WINDOW_MS = 200;  // filter uses the samples of the last 200 ms
// Initial thresholds only; overwritten in loop() from the pot
FsrTrigger fsrTrigger(PIN_FSR, FSR_CENTER_MIN + FSR_GAP / 2, FSR_CENTER_MIN - FSR_GAP / 2, FSR_CUTOFF_HZ, FSR_WINDOW_MS);
Trigger &trigger = fsrTrigger;

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
  int center = map((long)potFsr.update(), 0, 1023, FSR_CENTER_MIN, FSR_CENTER_MAX);
  int releasedBelow = constrain(center - FSR_GAP / 2, 0, 1023 - FSR_GAP);  // keep the band within 0-1023
  int pressedAbove = releasedBelow + FSR_GAP;
  fsrTrigger.setThresholds(pressedAbove, releasedBelow);

  // Set the pressed angle from the filtered angle potentiometer value
  int pressedAngle = map((long)potAngle.update(), 0, 1023, PRESSED_ANGLE_MIN, PRESSED_ANGLE_MAX);

  bool pressed = trigger.isActive();
  int targetAngle = pressed ? pressedAngle : NEUTRAL_ANGLE;
  motor.write(targetAngle);

  // Debugging: Print status every 800 ms
  if (millis() - lastPrint >= 800) {
    lastPrint = millis();

    // Line 1: System State
    Serial.print("SYSTEM: ");
    Serial.print("Trigger: ");
    Serial.print(pressed ? "PRESSED" : "RELEASED");
    Serial.print(" | Motor target angle: ");
    Serial.print(targetAngle);
    Serial.print(" | Motor position: ");
    Serial.println(motor.read());
    
    
    // Line 2: Force sensor and thresholds (set by the FSR potentiometer)
    Serial.print("SENSOR: ");
    Serial.print("FSR raw: ");
    Serial.print(analogRead(PIN_FSR));
    Serial.print(" | filtered: ");
    Serial.println(fsrTrigger.filtered());


    // Line 3: potentiometer for pedal sensitivity
    Serial.print("PEDAL SENS.: ");
    Serial.print("Pot raw: ");
    Serial.print(potFsr.raw);
    Serial.print(" | filtered: ");
    Serial.print(potFsr.filtered);
    Serial.print(" | Pressed above: ");
    Serial.print(pressedAbove);
    Serial.print(" | Released below: ");
    Serial.println(releasedBelow);


    // Line 4: potentiometer for motor angle 
    Serial.print("MOT. ANGEL: ");
    Serial.print("Pot raw: ");
    Serial.print(potAngle.raw);
    Serial.print(" | filtered: ");
    Serial.print(potAngle.filtered);
    Serial.print(" | pressed angle: ");
    Serial.println(pressedAngle);

    Serial.println();
  }

  delay(15);
}
