# ESP-FLY Blimp

A vision-tracking, IMU-stabilized blimp controlled wirelessly from a keyboard, built on two ESP32 boards linked over **ESP-NOW** and bridged to a PC over **USB serial**.

```
 ┌──────────────┐   USB Serial    ┌────────────────┐   ESP-NOW (2.4GHz)   ┌──────────────┐
 │ base_station │ <────wired────> │  Base Station  │ <──────wireless─────>│    Blimp     │
 │     .py      │  (keyboard in,  │    (ESP32)     │   telemetry / ctrl   │   (ESP32)    │
 │   (your PC)  │  telemetry out) │base_station.ino│                      │  blimp.ino   │
 └──────────────┘                 └────────────────┘                      └──────┬───────┘
                                                                                 │
                                                    ┌────────────────┬───────────┴──────────┬─────────────────────┐
                                                    │                │                      │                     │
                                                Vision (cam)      IMU (MPU6050)         State Machine          4x Motors
                                                Vision.cpp/h      IMU.cpp/h           StateMachine.cpp/h      (analogWrite)
```

## How it works

1. **Blimp** runs an onboard camera + IMU + state machine + battery-voltage loop, packages the readings into a `TelemetryPacket`, and transmits it to the base station over ESP-NOW every loop iteration. It listens for `ControlPacket`s from the base station (carrying motor inputs and active control mode) and drives 4 motors via `analogWrite`.
2. **Base station (ESP32)** acts as a relay: it forwards every `TelemetryPacket` from the blimp to USB serial (framed with headers/footers), and forwards any `ControlPacket` read from serial over ESP-NOW to the blimp.
3. **base_station.py** runs on your PC, reads user control input, computes motor values at ~20 Hz, and writes them along with the control mode (`MANUAL` vs `AUTONOMOUS`) to the base station. Pressing `M` toggles control mode live. The script parses incoming telemetry, monitors frame timing, and displays live status and latency in the terminal.

## Repository contents

| File | Runs on | Purpose |
|---|---|---|
| `blimp.ino` | Blimp ESP32 | Main control loop, sensor sampling, ESP-NOW telemetry/control, and motor PWM updates |
| `StateMachine.h` / `StateMachine.cpp` | Blimp ESP32 | Autonomous state machine managing target tracking (yaw & pitch), waypoint turning, and searching |
| `Vision.h` / `Vision.cpp` | Blimp ESP32 | OV-series camera capture, Lab-color thresholding, blob tracking, pixel area accumulation, ROI locking |
| `IMU.h` / `IMU.cpp` | Blimp ESP32 | MPU6050 accelerometer/gyro readout |
| `Motor.h` / `Motor.cpp` | Blimp ESP32 | Defines `MotorData` outputs (M1–M4) and helper functions shared with telemetry |
| `BatteryMonitor.h` / `BatteryMonitor.cpp` | Blimp ESP32 | Reads battery voltage via resistive divider, samples ADC, and reports reconstructed pack voltage |
| `base_station.ino` | Base station ESP32 | Serial ⇄ ESP-NOW protocol bridge |
| `base_station.py` | PC | Keyboard/Xbox controller input, serial framing, live telemetry/latency dashboard |
| `calibrate.ino` | Blimp ESP32 | Standalone camera streamer — outputs JPEG frames over serial |
| `calibrate.py` | PC | Live LAB color-mask tuner for dialing in `THRESHOLD_BLIMP` |

## Hardware

**Blimp ESP32 (ESP-FLY board)**

| Motor | Function | GPIO | Propeller |
|---|---|---|---|
| M1 | Front Right | 7 | CCW |
| M2 | Rear Right | 4 | CCW |
| M3 | Rear Left | 3 | CW |
| M4 | Front Left | 1 | CW |

- Camera: OV-series module wired per `Vision.h` pin map (XCLK=10, PCLK=13, VSYNC=38, HREF=47, SIOD=40, SIOC=39, D0–D7 as defined), status LED on GPIO 21.
- IMU: MPU6050 on I²C, SDA=5, SCL=6, address `0x68`.
- Battery voltage sensing: resistive divider from `Vbat` to GPIO 2. Polled every `BATTERY_READ_INTERVAL_MS` (500 ms) and attached to `TelemetryPacket` as `battVoltage`.
- Balloon: 24-36 inch 2-sheet Foil Balloon.
- Frame: PLA printed structure.
- Current CAD Model: https://cad.onshape.com/documents/6abb47ea8df48ed88ff41eff/w/fc873cead17ca65b0af647cb/e/1d23eafa7f8f630fb687ec6b

**Base station ESP32**
- ESP32-S3 board connected via USB serial to PC.
  - Other ESP32 boards could be made to work with minor modification to the code base.

## Wireless protocol (ESP-NOW)

Two fixed-size `packed` structs are exchanged over ESP-NOW:

```cpp
// Vision Data returned from camera processing
typedef struct __attribute__((packed)) {
  uint16_t cx, cy, w, h;  // Blob centroid (cx, cy) and ROI bounding dimensions (w, h)
  uint32_t pixels;        // Total count of masked pixels in the detected blob
} VisionData;

// Blimp -> Base station
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h, pixels
  IMUData imu;       // ax, ay, az, tz (4x float)
  MotorData motors;  // actual M1-M4 outputs (4x int16_t)
  float yawError;    // degrees of yaw needed to center target (+ => right)
  float battVoltage; // battery voltage in volts
  uint8_t state;     // reported state (STATE_MANUAL, STATE_TRACKING, STATE_TURNING, STATE_SEARCHING)
} TelemetryPacket;

// Base station -> Blimp
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // M1, M2, M3, M4 inputs
  uint8_t mode;      // MODE_MANUAL (0) or MODE_AUTONOMOUS (1)
} ControlPacket;
```

`vision.pixels` reports the exact number of threshold-matching pixels in the detected target blob. This count is used directly by the state machine to determine proximity to target objects.

## Serial protocol (base station ⇄ PC)

```
Telemetry (Base Station -> PC):
  [ 4B header: 00 AA 55 FF ] [ TelemetryPacket Payload ] [ 2B footer: EE FF ]

Control (PC -> Base Station):
  [ 4B header: 00 BB 66 FF ] [ ControlPacket Payload ]
```

## Control modes & State Machine

Control mode is selectable live via the `mode` byte inside `ControlPacket`. Pressing `M` in `base_station.py` flips modes at runtime:

- `MODE_MANUAL` (`0`): Motor outputs are driven by the base station commands with IMU gyro-Z yaw damping (`STRAIGHT_KD * iData.tz`) on M1 and M4.
- `MODE_AUTONOMOUS` (`1`): Handled by the standalone `StateMachine` class (`StateMachine.h` / `StateMachine.cpp`). Manual stick inputs are bypassed while the state machine calculates motor commands based on vision tracking data and IMU telemetry.

### Autonomous State Machine Sub-States

1. **`STATE_TRACKING`**: Active when a target blob is detected (`w > 0 && h > 0`) and total blob size has not reached the turn threshold (`pixels <= TURNING_AREA`).
   - **Yaw Controller**: Calculates horizontal error (`yawError`) in degrees. If `|yawError| > YAW_DEADZONE_HALF_DEG` (2°), proportional thrust correction (`YAW_GAIN_PER_DEG = 10`) is applied across forward motors M1 and M4, combined with gyro rate damping (`TURN_KD = 5`).
   - **Pitch Controller**: Calculates vertical error (`pitchError`) in degrees using vertical FOV parameters (`VERTICAL_FOV_DEG = 44.6`). If `|pitchError| > PITCH_DEADZONE_HALF_DEG` (2°), proportional thrust correction (`PITCH_GAIN_PER_DEG = 2`) adjusts vertical thrust across M2 and M3 relative to `DEFAULT_UPWARD_POWER`.
2. **`STATE_TURNING`**: Triggered when `vData.pixels > TURNING_AREA` (15,000 pixels), indicating the blimp is close enough to a waypoint target. The blimp halts tracking and executes a closed-loop rotation sequence defined in `WAYPOINT_LIST` using IMU gyro yaw integration before advancing to the next waypoint.
3. **`STATE_SEARCHING`**: Active when no visual target is visible. Maintains default forward and upward baseline thrust levels while attempting to acquire a target.

## Setup & Calibration

### 1. Color Mask Calibration
1. Flash `calibrate.ino` onto the ESP32.
2. Run `python calibrate.py` on your PC.
3. Adjust LAB thresholds live using keyboard inputs (`1`–`6` / `Q`–`Y`).
4. Apply calibrated values to `THRESHOLD_BLIMP` in `Vision.h`.

### 2. Ground Control
```bash
pip install pyserial pygame
python base_station.py
```

### Key Controls
- **WASD / QE**: Manual flight movement.
- **M**: Toggle control mode (`MANUAL` ⇄ `AUTONOMOUS`).
- **Ctrl+C**: Stop and force safe zero-thrust shutdown.

## Safety & Failsafe

If no `ControlPacket` is received within `CONTROL_TIMEOUT_MS` (1000 ms), the control link is marked as stale. In both `MODE_MANUAL` and `MODE_AUTONOMOUS`, motor outputs are immediately forced to zero until communication is restored.
