# Piano Pedal Control

A force sensor (FSR) under the foot triggers a servo motor that presses the piano pedal.
Two potentiometers tune the system while it runs. Firmware: [Piano_pedal_control/src/main.cpp](Piano_pedal_control/src/main.cpp).

## Wiring

**Motor (BLS3235, 3 wires)**
- GND (black/brown) -> GND (shared with the Arduino GND)
- Power (red) -> external supply as per datasheet (do not power from the Arduino pin if the motor draws a lot of current)
- Data (yellow/white) -> `PIN_MOTOR` (5)

**Force sensor (FSR402, polarity does not matter)**
- Pin 1 -> 5V (3.3V on 3.3V boards)
- Pin 2 -> `PIN_FSR` (A0) and through a ~10k resistor to GND (voltage divider)

**Potentiometers (3 pins each)**
- Outer pin 1 -> GND, outer pin 2 -> 5V (3.3V on 3.3V boards)
- Wiper (middle) -> A1 (`PIN_POT_FSR`) or A2 (`PIN_POT_ANGLE`)
- Swapping the outer pins reverses the direction
- Both readings are low-pass filtered (`POT_CUTOFF_HZ`)

## Potentiometers

### A1: sensitivity (FSR thresholds)

The foot rests on the sensor and pushes on it to play. The FSR has two thresholds:

- **pressed above**: the foot pushes hard enough -> motor presses the pedal
- **released below**: the foot only rests (or lifts) -> motor returns to neutral

The pot shifts both thresholds together, keeping the distance `FSR_GAP` between them:

```
centre         = FSR_CENTER_MIN ... FSR_CENTER_MAX   (pot full left ... full right)
released below = centre - FSR_GAP / 2
pressed above  = centre + FSR_GAP / 2
```

Direction depends on the pot wiring. Check line 3 of the serial output for the current thresholds.

### A2: motor angle

Sets the angle used when pressed, linearly from `PRESSED_ANGLE_MIN` (90°, full left) to
`PRESSED_ANGLE_MAX` (180°, full right). When released the motor goes to `NEUTRAL_ANGLE` (0°).

## Tuning for a new mechanical construction

Only `FSR_CENTER_MIN` and `FSR_CENTER_MAX` need to be adapted. `FSR_GAP` only sets the distance
between the thresholds and does not affect the range.

1. Open the serial monitor (115200 baud) and read `FSR filtered` (line 2) with
   - the foot only resting on the sensor (`F_rest`)
   - the foot pushing as for playing (`F_press`)
2. Set `FSR_CENTER_MIN` / `FSR_CENTER_MAX` so that `F_rest` lies inside the range the
   thresholds can reach and `F_press` lies above it. Choose a range that is not much wider than needed, so the pot stays fine to adjust.
3. Fine-tune by feel with the pot while playing.

`FSR_GAP` should be larger than the noise of `FSR filtered` and smaller than `F_press - F_rest`.

## Serial output

115200 baud, every 800 ms:

1. `SYSTEM`: trigger state, motor target angle, motor position
2. `SENSOR`: FSR raw / filtered
3. `PEDAL SENS.`: pot A1 raw / filtered, current thresholds
4. `MOT. ANGEL`: pot A2 raw / filtered, pressed angle

## Configuration (main.cpp)

| Constant | Meaning |
|---|---|
| `FSR_CENTER_MIN`, `FSR_CENTER_MAX` | range of the threshold centre (tune per construction) |
| `FSR_GAP` | distance between pressed and released threshold |
| `FSR_CUTOFF_HZ`, `FSR_WINDOW_MS` | FSR low-pass filter |
| `POT_CUTOFF_HZ` | low-pass filter for both pots |
| `PRESSED_ANGLE_MIN/MAX`, `NEUTRAL_ANGLE` | motor angles |
