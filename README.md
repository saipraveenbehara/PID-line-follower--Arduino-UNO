# Line-Following Robot — Arduino PID Controller

**Created by Sai Praveen Behara**

---

## Overview

A two-wheeled differential-drive robot that autonomously follows a black line on a white surface. A 5-channel IR sensor array feeds positional error into a PID (Proportional-Integral-Derivative) controller, which continuously adjusts the left and right motor speeds to keep the robot centred on the line. No line-tracking library is used — the PID algorithm is implemented from scratch for full transparency and tunability.

---

## Hardware Bill of Materials

| Qty | Component | Notes |
|-----|-----------|-------|
| 1 | Arduino Uno (ATmega328P) | Or any 5V compatible board |
| 1 | 5-channel IR sensor array | TCRT5000-based or Pololu QTRX-5A |
| 1 | L298N dual H-bridge module | Includes onboard 5V regulator |
| 2 | DC gear motors (6V, 150–300 RPM) | N20 or TT motor format |
| 2 | Motor wheels + 1 caster | Rear caster for balance |
| 1 | 7.4V LiPo (2S) or 2x 18650 | Minimum 1000 mAh recommended |
| 1 | Robot chassis (2WD) | 3D print or acrylic cut |
| 1 | Tactile push-button | Optional start trigger |
| 1 | 10 uF capacitor | Across motor power rails to reduce noise |
| — | Jumper wires, breadboard | — |

---

## Pin Mapping

```
Arduino Uno
│
├── D2  ──> IR Sensor 0  (leftmost)
├── D3  ──> IR Sensor 1
├── D4  ──> IR Sensor 2  (centre)
├── D5  ──> IR Sensor 3
├── D6  ──> IR Sensor 4  (rightmost)
│
├── D7  ──> L298N IN1   (Left motor direction A)
├── D8  ──> L298N IN2   (Left motor direction B)
├── D9  ──> L298N ENA   (Left motor PWM speed)   [Timer1]
│
├── D10 ──> L298N ENB   (Right motor PWM speed)  [Timer1]
├── D11 ──> L298N IN3   (Right motor direction A)
├── D12 ──> L298N IN4   (Right motor direction B)
│
└── D13 ──> Start button (active LOW, uses internal pull-up)
```

---

## Software Setup

1. Install the **Arduino IDE** (version 1.8.x or 2.x).
2. No additional libraries are required. The sketch uses only built-in Arduino functions (`analogWrite`, `digitalRead`, `constrain`).
3. Open `LineFollowerPID.ino` in the Arduino IDE.
4. Select **Board: Arduino Uno** and the correct **COM port**.
5. Click **Upload**.

---

## PID Tuning Procedure

All tuning constants are grouped at the top of `LineFollowerPID.ino` for convenience.

```cpp
const float KP = 20.0;
const float KI = 0.0;
const float KD = 5.0;
```

Follow these steps in order:

### Step 1 — Set starting values
```
KP = 5    KI = 0    KD = 0
```

### Step 2 — Tune KP (Proportional)
Increase KP gradually. The robot will start correcting towards the line. Stop when the robot oscillates with a consistent left-right weave. Back off KP by about 20%.

| KP too low | KP correct | KP too high |
|------------|-----------|-------------|
| Robot drifts off line | Follows line with mild weave | Fast, wide oscillation |

### Step 3 — Tune KD (Derivative)
With KP set, increase KD. This dampens the oscillation. Stop when the weave disappears and the robot tracks smoothly.

| KD too low | KD correct | KD too high |
|------------|-----------|-------------|
| Oscillation remains | Smooth tracking | Jerky, twitchy response |

### Step 4 — Tune KI (Integral) — usually optional
Add a small KI only if the robot consistently drifts to one side even on a straight line. Keep it very small (0.001–0.1 range).

### Typical starting values by track type

| Track type | KP | KI | KD | BASE_SPEED |
|------------|----|----|----|------------|
| Wide curves, slow | 15 | 0 | 3 | 120 |
| Mixed curves | 20 | 0 | 5 | 160 |
| Tight curves, fast | 30 | 0.01 | 8 | 180 |

---

## Control Flow

```
[Power ON]
    |
[Setup: pin config, serial init]
    |
[Wait for start button press]
    |
    v
+------------------------------------------+
|              MAIN LOOP                   |
|                                          |
|  readSensors()                           |
|    -> sensorValues[0..4] = 0 or 1       |
|                                          |
|  computeError()                          |
|    -> weighted average of active sensors |
|    -> returns float in [-2.0 .. +2.0]   |
|                                          |
|  computePID(error)                       |
|    -> P = KP * error                    |
|    -> I = KI * integral (clamped)       |
|    -> D = KD * (error - lastError)      |
|    -> returns correction signal          |
|                                          |
|  applyMotorOutput(correction)            |
|    -> leftSpeed  = BASE_SPEED + corr    |
|    -> rightSpeed = BASE_SPEED - corr    |
|    -> clamp to [MIN_SPEED, MAX_SPEED]   |
|    -> drive motors                       |
+------------------------------------------+
```

---

## Edge Case Handling

| Situation | Sensor state | Behaviour |
|-----------|-------------|-----------|
| Line centred | Sensor 2 ON | Zero error, straight ahead |
| Line to the left | Sensors 0-1 ON | Negative error, steer left |
| Line to the right | Sensors 3-4 ON | Positive error, steer right |
| Line lost | All sensors OFF | Last known error held; robot curves to search |
| Intersection | All sensors ON | Error forced to 0; robot drives straight through |

---

## Serial Debug Output

Uncomment the `debugPrint()` call inside `loop()` to stream live PID data:

```
ERR:-1.00 OUT:-20.00 L:140 R:180 SNS:11000
ERR:-0.50 OUT:-10.00 L:150 R:170 SNS:01000
ERR: 0.00 OUT:  0.00 L:160 R:160 SNS:00100
```

Open **Tools > Serial Monitor** at **115200 baud**, or **Tools > Serial Plotter** for a live graph. The Serial Plotter view is the fastest way to visualise PID oscillation during tuning.

---

## Known Limitations and Future Work

- Sensor calibration is hardcoded. A startup calibration routine that sweeps the robot over the line to auto-detect thresholds would make the system more robust to different floor colours and lighting conditions.
- No acceleration ramp at startup. Adding a gradual speed ramp prevents wheel slip and improves initial tracking.
- Intersection detection is minimal. A full intersection handler would count how long all sensors are simultaneously ON and branch into a turn or straight-through decision based on a predefined route map.
- No battery monitoring. A low-voltage cutoff via the ADC would prevent motor brownout at the end of a battery discharge.

---

## License

MIT License. Free to use, modify, and distribute with attribution.
