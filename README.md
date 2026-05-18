# Reaction Wheel Inverted Pendulum

A self-balancing inverted pendulum that swings up from rest and balances upright using a reaction wheel for torque. Built with an Arduino UNO R4 WiFi, a brushed DC motor, and a high-resolution quadrature encoder.

---

## How It Works

The pendulum starts hanging at the bottom. It swings itself upright by spinning the reaction wheel back and forth — the torque reaction on the pendulum pumps energy into each swing. Once it reaches the upright position, a PID controller takes over and keeps it balanced indefinitely.

**Swing-up:** The motor runs in whichever direction the pendulum is currently moving. At each peak, the motor hard-reverses — this continuous `dω/dt` of the wheel creates maximum reaction torque at every moment of the swing. Near the top, an approach decelerator bleeds excess energy so the pendulum arrives with just enough to catch.

**Balance:** A PID loop with velocity feedforward, predictive correction, and adaptive gains drives the reaction wheel to keep the pendulum upright. A wheel desaturation algorithm shifts the balance setpoint slightly to oppose accumulated wheel spin, preventing the wheel from building up speed until it can no longer correct in one direction.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino UNO R4 WiFi |
| Motor | RS-550, 24V brushed DC, 35,000 RPM no-load |
| Motor driver | BTS7960 (IBT-2) 43A H-bridge |
| Power supply | 24V 600W switching PSU with inline E-stop |
| Encoder | CALT GHS38, 2500 PPR incremental, ABZ line driver, 5V |
| Pendulum arm | 3D printed, ~228mm, Hyper PLA |
| Reaction wheel | 3D printed, ~200mm OD, 20mm thick, 27× M8 nut ballast (~121.5g) |
| Drive | Direct drive (motor shaft → reaction wheel) |

### Pin Map

```
Motor driver:
  R_EN  → pin 8
  L_EN  → pin 11
  RPWM  → pin 9
  LPWM  → pin 10

Encoder:
  ENC_A → pin 2  (interrupt)
  ENC_B → pin 3  (interrupt)
  ENC_Z → pin 4  (zero index)
```

### Wiring Notes

- **Encoder:** Red→5V, Black→GND, Green(A)→pin 2, White(B)→pin 3, Yellow(Z)→pin 4. Differential lines (A-/B-/Z-) left floating; shield to GND.
- **BTS7960:** VCC/GND from 5V rail; logic pins to Arduino as above. PSU V+/V− → BTS7960 B+/B− (14AWG, through E-stop). M+/M− → motor (20AWG).

---

## Software

### Encoder

Full 4× quadrature decode via `CHANGE` interrupt on both A and B channels — 2500 PPR × 4 edges = **10,000 counts/revolution**. A state table (`QUAD_TABLE`) handles direction from the two-channel transition pattern.

The Z-pulse zeros the encoder exactly once at bottom dead center, then `detachInterrupt` disables it to prevent mid-run re-zeroing.

> **Note:** This runs on the Renesas RA4M1 (not AVR). ISRs use `digitalRead`, not `PIND` — port reads don't work on this chip.

### Coordinate System

- **0°** = pendulum hanging straight down (Z-pulse zero point)
- **180°** = upright

### State Machine

Each loop iteration selects one of two modes:

| Mode | Condition |
|---|---|
| **Swing-up** | More than 35° from vertical, and not slow-and-close |
| **Stabilization** | Within 35° of vertical, or within 60° and moving slower than 300°/s |

### Swing-Up Algorithm

1. **Pre-spin** — spins the wheel at full PWM for 800ms before the first swing to establish a momentum baseline.
2. **Hard-reverse pumping** — motor direction always matches the pendulum's current motion direction. At each velocity sign change (peak), the motor direction reverses instantly (no coast, no brake), keeping `|dω/dt|` high at all times for maximum reaction torque.
3. **Coast zone** — motor stops when within 45° of vertical; pendulum glides into the catch window.
4. **Approach decelerator** — between 60° and 35° from vertical, if the pendulum is approaching faster than 200°/s, the motor briefly brakes to bleed excess energy.
5. **Near-miss management** — if a swing reaches within 50° of vertical without catching, the next pump is skipped and the energy multiplier drops by ×0.75 (floor: 0.4). On non-near-miss peaks, energy recovers +0.05.
6. **Rotation detection** — if `|velocity| > 400°/s` while more than 90° from vertical, the pendulum is fully rotating; motor stops and friction bleeds energy back down.

### Stabilization (PID + Extras)

- **Predictive error:** Error is computed from the predicted angle 30ms into the future: `angle + vel·dt + ½·accel·dt²`
- **Velocity feedforward:** `−Kff × velocity` added to output, directly opposing motion for fast disturbance response
- **Adaptive Kd:** Multiplied by 1.6× when within 5° of vertical for tighter damping near the setpoint
- **Three-tier output smoothing:** Low smoothing (0.35) when calm; higher smoothing (0.9) during light disturbance; no smoothing at all during hard disturbance (>80°/s)
- **Adaptive deadband and direction hold:** Deadband and min direction hold time reduce during disturbances so the motor reacts faster
- **Stiction compensation:** Minimum PWM of 75 when the motor is commanded to move, ensuring it actually overcomes static friction
- **Wheel desaturation:** Integrates PID output over time as a proxy for accumulated wheel speed. Shifts `targetAngle` by up to ±4.5° so gravity partially opposes the wheel spin, passively slowing it down during steady balance

### Current Tuning Values

```cpp
// PID
Kp = 55.0,  Ki = 5.0,  Kd = 10.0,  Kff = 0.6
PID_SAMPLE_TIME = 15 ms
PREDICTION_HORIZON_MS = 30 ms

// Motor (balance)
BALANCE_MIN_PWM = 75
BALANCE_DEADBAND = 12
DISTURBANCE_DEADBAND = 0
OUTPUT_SMOOTHING = 0.35
MIN_DIR_HOLD_MS = 60
DISTURBANCE_DIR_HOLD_MS = 10

// Swing-up
SWINGUP_PWM_MAX = 255,  SWINGUP_PWM_MIN = 200
COAST_THRESHOLD_DEG = 45°
DECEL_ANGLE_START = 60°,  DECEL_ANGLE_END = 35°
DECEL_VEL_THRESHOLD = 200°/s,  DECEL_PWM = 180

// Wheel desaturation
WHEEL_SPEED_DECAY_RATE = 0.3
WHEEL_ACCEL_GAIN = 0.015
DESAT_MAX_BIAS_DEG = 4.5°
```

---

## Setup & Upload

1. Open `RWIP.ino` in Arduino IDE
2. Select board: **Arduino UNO R4 WiFi**
3. Select the correct COM/serial port
4. Click **Verify** (compile), then **Upload**
5. Open Serial Monitor at **115200 baud**
6. Place the pendulum at the bottom dead center position
7. The firmware waits 3 seconds, pre-spins the wheel, then begins swing-up automatically

---

## Serial Output

```
SWING | ang: 5.2 | vel: -312 | fromVert: 174.8 | skip: 0 | E: 1.00
>>> PEAK: 428 | minFromVert: 170.1
BRAKE | ang: 145.3 | vel: 298
>>> ENTERING BALANCE MODE | entry ang: 179.1 | entry vel: 42
BAL | ang: 179.4 | vel: 12 | err: 0.6 | out: 87 | whl: 0.04
```

`whl` is the wheel speed estimate used for desaturation (range −1 to +1).

---

## Tuning Guide

### Swing-up
| Symptom | Fix |
|---|---|
| Too many swings to reach upright | Lower `COAST_THRESHOLD_DEG`, raise `SWINGUP_PWM_MIN` |
| Pendulum goes into full rotation | Lower `SWINGUP_PWM_MAX`, raise `COAST_THRESHOLD_DEG` |
| Arrives too fast, falls past vertical | Lower `DECEL_VEL_THRESHOLD`, raise `DECEL_PWM` |
| Near-misses don't recover well | Lower `ENERGY_REDUCE_FACTOR`, raise `SKIP_PUMPS_AFTER_NEAR_MISS` |

### Balance
| Symptom | Fix |
|---|---|
| No reaction to small tilt | Raise `BALANCE_MIN_PWM` |
| Rapid twitching without effective correction | Raise `MIN_DIR_HOLD_MS`, raise `BALANCE_DEADBAND` |
| Slow drift off-center | Raise `Ki` |
| Falls before motor responds | Lower `MIN_DIR_HOLD_MS`, lower `BALANCE_DEADBAND`, raise `OUTPUT_SMOOTHING` |
| Side-to-side oscillation | Raise `Kd`, lower `Kp` |
| Falls same direction every time | Adjust `vert` (line 97) by 1–3° toward the fall direction |
| Poor disturbance rejection | Raise `Kff`, raise `KD_BOOST_NEAR_VERTICAL`, raise `PREDICTION_HORIZON_MS` |
| Wheel keeps spinning in one direction | Raise `DESAT_MAX_BIAS_DEG`, raise `WHEEL_ACCEL_GAIN` |

---

## Known Limitations

- **Inertia ratio:** η ≈ 10%. Direct-drive means motor torque isn't amplified. This is the physical ceiling on disturbance rejection — a belt reduction would multiply torque but add complexity.
- **No slip ring:** Wiring wraps around the pivot; continuous rotation in one direction will eventually snag. The rotation detector stops pumping if this happens during swing-up, but long balance sessions with sustained wheel spin can cause issues.
- **MPU-6050 wired but unused:** An IMU is mounted on the pendulum arm but was unreliable during testing. The encoder alone gives sufficient angle and velocity for balance.
