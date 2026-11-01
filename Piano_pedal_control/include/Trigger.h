#pragma once
#include <Arduino.h>

// Common interface for anything that can trigger the pedal motor.
// To use a different sensor, implement this interface and pass it to the controller.
class Trigger {
public:
  virtual ~Trigger() {}
  virtual void begin() = 0;
  virtual bool isActive() = 0;  // true = pedal should be pressed
};

// Endstop switch (B59). Wired between pin and GND, uses the internal pull-up,
// so a closed (pressed) switch reads LOW.
class SwitchTrigger : public Trigger {
public:
  explicit SwitchTrigger(uint8_t pin) : pin_(pin) {}

  void begin() override { pinMode(pin_, INPUT_PULLUP); }
  bool isActive() override { return digitalRead(pin_) == LOW; }

private:
  uint8_t pin_;
};

// Force sensitive resistor (FSR402) in a voltage divider: the pedal counts as pressed when
// the filtered reading is HIGH. The raw reading is low-pass filtered (first-order, cutoff
// cutoffHz) over the samples of the last windowMs milliseconds.
// Hysteresis: pressed when filtered > pressedAbove, released when filtered < releasedBelow
// (pressedAbove must be higher than releasedBelow).
class FsrTrigger : public Trigger {
public:
  FsrTrigger(uint8_t pin, int pressedAbove, int releasedBelow,
             float cutoffHz = 20.0f, unsigned long windowMs = 200)
      : pin_(pin), pressedAbove_(pressedAbove), releasedBelow_(releasedBelow),
        rc_(1.0f / (2.0f * PI * cutoffHz)), windowMs_(windowMs) {}

  void begin() override { pinMode(pin_, INPUT); }

  bool isActive() override {
    addSample(millis(), analogRead(pin_));
    filtered_ = filter();
    if (!active_ && filtered_ > pressedAbove_) active_ = true;
    else if (active_ && filtered_ < releasedBelow_) active_ = false;
    return active_;
  }

  float filtered() const { return filtered_; }  // last filtered value, for debugging

  // Change the hysteresis thresholds at runtime (e.g. from a potentiometer).
  void setThresholds(int pressedAbove, int releasedBelow) {
    pressedAbove_ = pressedAbove;
    releasedBelow_ = releasedBelow;
  }

private:
  static const uint8_t MAX_SAMPLES = 32;

  void addSample(unsigned long t, int v) {
    if (count_ == MAX_SAMPLES) drop();
    uint8_t idx = (head_ + count_) % MAX_SAMPLES;
    times_[idx] = t;
    values_[idx] = v;
    count_++;
    while (count_ > 1 && t - times_[head_] > windowMs_) drop();  // keep only the last windowMs
  }

  void drop() {
    head_ = (head_ + 1) % MAX_SAMPLES;
    count_--;
  }

  // Run the low-pass filter over the stored window, oldest to newest.
  float filter() const {
    float y = values_[head_];
    for (uint8_t i = 1; i < count_; i++) {
      uint8_t cur = (head_ + i) % MAX_SAMPLES;
      uint8_t prev = (head_ + i - 1) % MAX_SAMPLES;
      float dt = (times_[cur] - times_[prev]) / 1000.0f;
      float alpha = dt / (rc_ + dt);
      y += alpha * (values_[cur] - y);
    }
    return y;
  }

  uint8_t pin_;
  int pressedAbove_, releasedBelow_;
  float rc_;
  unsigned long windowMs_;
  unsigned long times_[MAX_SAMPLES];
  int values_[MAX_SAMPLES];
  uint8_t head_ = 0, count_ = 0;
  float filtered_ = 0;
  bool active_ = false;
};
