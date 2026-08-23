# ESP-FLY Blimp

A vision-tracking, IMU-stabilized blimp controlled wirelessly from a keyboard, built on two ESP32 boards linked over **ESP-NOW** and bridged to a PC over **USB serial**.

```
 ┌──────────────┐   USB Serial    ┌────────────────┐   ESP-NOW (2.4GHz)   ┌──────────────┐
 │ base_station │ <────wired────> │  Base Station  │ <──────wireless─────>│    Blimp     │
 │     .py      │  (keyboard in,  │    (ESP32)     │   telemetry / ctrl   │   (ESP32)    │
 │   (your PC)  │  telemetry out) │base_station.ino│                      │  blimp.ino   │
 └──────────────┘                 └────────────────┘                      └──────┬───────┘
                                                                                 │
                                                                     ┌───────────┼────────────┐
                                                                     │           │            │
                                                                Vision (cam)  IMU (MPU6050)  4x Motors
                                                              Vision.cpp/h   IMU.cpp/h      (analogWrite)
```

## How it works

1. **Blimp** runs an onboard camera + IMU loop, packages the readings into a `TelemetryPacket`, and blasts it to the base station over ESP-NOW every loop iteration. It also listens for `ControlPacket`s from the base station, which carry both motor commands and the active control mode, and drives the 4 motors via `analogWrite`.
2. **Base station (ESP32)** is a dumb relay: it forwards every `TelemetryPacket` it receives from the blimp out over USB serial (framed with a header/footer), and forwards any `ControlPacket` it reads from serial out to the blimp over ESP-NOW. It doesn't interpret the mode byte at all — that's the blimp's job.
3. **base_station.py** runs on your PC, reads WASD-style key state, computes motor values at ~20 Hz, and writes them (plus the currently selected control mode) to the base station over serial. Pressing `M` toggles the mode live. The script also parses incoming telemetry frames, tracks round-trip frame timing, and prints a live status/latency readout to the terminal.

## Repository contents

| File | Runs on | Purpose |
|---|---|---|
| `blimp.ino` | Blimp ESP32 | Vision + IMU sensor loop, motor output, ESP-NOW telemetry/control |
| `Vision.h` / `Vision.cpp` | Blimp ESP32 | OV-series camera capture, Lab-color threshold blob tracking, ROI locking |
| `IMU.h` / `IMU.cpp` | Blimp ESP32 | MPU6050 accelerometer/gyro readout |
| `motor.h` / `motor.cpp` | Blimp ESP32 | Defines `MotorData` (the actual, post-constrain M1–M4 outputs) and a small constructor helper, shared by `TelemetryPacket` |
| `base_station.ino` | Base station ESP32 | Serial ⇄ ESP-NOW protocol bridge (no processing) |
| `base_station.py` | PC | Keyboard/Xbox-controller control, serial framing, live telemetry/latency dashboard |
| `calibrate.ino` | Blimp ESP32 (or standalone camera board) | Standalone camera streamer — grabs JPEG frames and writes them to serial with a `{FF AA 55 FF}` + length header, no ESP-NOW involved |
| `calibrate.py` | PC | Live LAB color-mask tuner — decodes the JPEG stream from `calibrate.ino`, applies the current L/A/B thresholds, and displays the resulting mask so you can dial in `THRESHOLD_BLIMP` |

## Hardware

**Blimp ESP32 (ESP-FLY board)**

| Motor | Function | GPIO | Propeller
|---|---|---| --- |
| M1 | Front Right | 7 | CCW
| M2 | Rear Right | 4 | CCW
| M3 | Rear Left | 3 | CW
| M4 | Front Left | 1 | CW

- Camera: OV-series module wired per `Vision.h` pin map (XCLK=10, PCLK=13, VSYNC=38, HREF=47, SIOD=40, SIOC=39, D0–D7 as defined), status LED on GPIO 21.
- IMU: MPU6050 on I²C, SDA=5, SCL=6, address `0x68`.
- Balloon: 30/36 inch 2-sheet Foil Balloon
  - Potential Sources:
    - https://www.balloonsdirect.com/36-inch-round-foil-balloons-gold
    - https://www.balloonsdirect.com/36-inch-round-foil-balloons-sapphire-blue
    - https://bargainballoons.com/products/36-inches-navy-blue-round-packaged-oaktree-brand-foil-balloon-ot-608320
    - https://bargainballoons.com/products/36-inches-pure-gold-round-packaged-oaktree-brand-foil-balloon-ot-608290
- Frame
  - Material: PLA
  - Current CAD Model: https://cad.onshape.com/documents/6abb47ea8df48ed88ff41eff/w/fc873cead17ca65b0af647cb/e/1d23eafa7f8f630fb687ec6b
- Notes:
  - PSRAM is required — the camera frame buffer lives in PSRAM; everything else (mask buffer, flood-fill stack, threshold LUT) lives in internal SRAM. See Memory layout below.
  - The rear motors are rotated clockwise in the +x-axis (AKA when looking from the back) relative to their PCB pads.


**Base station ESP32**

- Any ESP32 board with USB serial to the PC. No sensors — it's purely a bridge.

## Wireless protocol (ESP-NOW)

Two fixed-size, `packed` structs are exchanged directly as ESP-NOW payloads:

```cpp
// Blimp -> Base station
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4x uint16_t) — blob centroid + ROI box size
  IMUData imu;       // ax, ay, az, tz (4x float)  — accel XYZ + gyro Z
  MotorData motors;  // m1, m2, m3, m4 (4x int16_t) — actual, post-constrain motor outputs
  float yawError;    // degrees of yaw needed to center the target (computed on the blimp)
} TelemetryPacket;

// Base station -> Blimp
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // M1, M2, M3, M4
  uint8_t mode;       // 0 = MODE_MANUAL, 1 = MODE_PROPORTIONAL — set live from the keyboard ('M')
} ControlPacket;
```

`vision.cx`/`vision.cy` are the tracked blob's true centroid in frame coordinates (0–320, 0–240 on the QVGA camera); `vision.w`/`vision.h` are the padded tracking ROI's dimensions, useful for display but not for locating the target itself.

`telemetry.motors` (defined in `motor.h`/`motor.cpp` as `MotorData`) carries the *actual* M1–M4 values just written to `analogWrite()` for that loop iteration — after mode selection, the yaw controller (if active), and the `[0, 255]` clamp. This is filled in and the telemetry packet is sent only after motor outputs are computed each loop, so it's never a stale value from the previous iteration. It's distinct from the base station's `ControlPacket.motors`, which is the last *commanded* value from the PC and can differ from what's actually driving the motors (e.g. in `MODE_PROPORTIONAL`, where manual stick input is ignored, or when the watchdog has forced zero). Having the real output on hand is useful for autonomous operation downstream — e.g. logging, closed-loop tuning, or a PC-side controller that needs to know actual thrust rather than last-sent thrust.

Each side's `esp_now_add_peer` must point at the other's MAC address — set these in both `.ino` files before flashing:

Both sketches now check `esp_now_send`'s return value (whether the packet was successfully queued) and register an `OnDataSent` callback for the async `esp_now_send_status_t` (whether it was actually ACKed over the air), logging a message on either kind of failure instead of failing silently.

- `base_station.ino` → `blimpAddress[]`
- `blimp.ino` → `baseStationAddress[]`

## Serial protocol (base station ⇄ PC)

The base station re-frames each `TelemetryPacket` for transport over serial, and unwraps `ControlPacket`s the same way:

```
Telemetry (Base Station -> PC), 42 bytes total:
  [ 4B header: 00 AA 55 FF ] [ 36B payload: 4x uint16 + 4x float + 4x int16 + 1x float ] [ 2B footer: EE FF ]

Control (PC -> Base Station), 13 bytes total:
  [ 4B header: 00 BB 66 FF ] [ 9B payload: 4x int16 motor values + 1x uint8 mode ]
```

The 36-byte telemetry payload is `cx, cy, w, h` (4x `uint16`), `ax, ay, az, tz` (4x `float`), `m1, m2, m3, m4` — the blimp's actual motor outputs (4x `int16`) — then `yawError` (1x `float`), the degrees of yaw needed to center the target, computed on the blimp. `base_station.py` unpacks this with `struct.unpack("<4H4f4hf", raw_payload)` against a `PAYLOAD_SIZE` of 36 bytes.

`base_station.py` re-syncs to the header on every parse pass, so it tolerates dropped/partial bytes on the serial line.

## Calibrating Vision Model
**Calibrating the color mask**

1. Flash `calibrate/calibrate.ino` onto the Xiao ESP32S3.
2. Keep the board connected to the computer over USB.
3. Run `calibrate/calibrate.py` on the computer. A window should open showing the default mask applied live.
4. Adjust the mask with the following keys:
   | Parameter | Decrease | Increase |
   |---|---|---|
   | `L_min` | `1` | `Q` |
   | `L_max` | `2` | `W` |
   | `A_min` | `3` | `E` |
   | `A_max` | `4` | `R` |
   | `B_min` | `5` | `T` |
   | `B_max` | `6` | `Y` |
5. Every adjustment prints the current mask values to the terminal.
6. The default starting mask lives in `calibrate.py` (lines 14–16, `l_min`/`l_max`, `a_min`/`a_max`, `b_min`/`b_max`) —
   edit it there if you want a different starting point for future calibration runs.
7. `calibrate.py` requires `opencv-python`, `numpy`, and `pygame` in addition to `pyserial` (see [Setup](#1-flash-the-firmware-arduino-ide--arduino-cli) below) — install with `pip install opencv-python numpy pygame pyserial`.

**Applying a calibrated mask to the detector**

Edit `src\vision.h` Line 90:

```cpp
static const LabThreshold THRESHOLD_BLIMP = { l_min, l_max, a_min, a_max, b_min, b_max };
```

## Setup

### 1. Flash the firmware (Arduino IDE / arduino-cli)

Required libraries: `esp_now`, `WiFi`, `esp_camera` (ESP32 board package), `Adafruit_MPU6050`, `Adafruit_Sensor`, `Wire`.

> **Before compiling either sketch:** copy the `bidirectional` folder into your Arduino `libraries` folder (e.g. `~/Documents/Arduino/libraries/` on most OSes, `Documents\Arduino\libraries\` on Windows). Both `blimp.ino` and `base_station.ino` depend on the shared code in it, and the Arduino IDE/CLI won't find it otherwise.

1. Find each board's MAC address (e.g. `WiFi.macAddress()` in a throwaway sketch).
2. Set `blimpAddress[]` in `base_station.ino` and `baseStationAddress[]` in `blimp.ino`.
3. Flash `blimp.ino` (with `Vision.h/.cpp`, `IMU.h/.cpp` alongside it) to the blimp board.
4. Flash `base_station.ino` to the base station board and plug it into your PC via USB.

### 2. Run the ground control script

```bash
pip install pyserial pygame
python base_station.py
```

(`calibrate.py` has its own, separate dependency list — see [Calibrating Vision Model](#calibrating-vision-model) above.)

Edit `SERIAL_PORT` at the top of `base_station.py` first (e.g. `COM9` on Windows, `/dev/ttyUSB0` / `/dev/ttyACM0` on Linux/macOS) and confirm `BAUD_RATE` matches both `.ino` files (`115200`).

### Controls

`base_station.py` supports two input methods, and will use whichever is available: an Xbox (or XInput-compatible) controller if `pygame` detects one at launch, otherwise the keyboard. Both drive the same `ControlPacket` at ~20 Hz.

**Keyboard**

| Key | Effect |
|---|---|
| `W` | +50 to M1 and M4 |
| `A` | +50 to M4 |
| `D` | +50 to M1 |
| `Q` | +50 to M3 |
| `E` | +50 to M2 |
| `M` | Toggle control mode: `MANUAL` ⇄ `AUTONOMOUS` (yaw-only, see below) |
| `Ctrl+C` | Stop — sends an all-zero, forced-`MANUAL` motor command and exits |

M2 carries a constant idle offset (`10`) even with no keys held (see `compute_motors()`); every other motor idles at `0`. `M` toggles on the key-down edge only (holding it or OS key-repeat won't rapidly flip modes), and the script starts in `MANUAL` every time it launches, regardless of what mode the blimp was last left in.

You must click into the small "Base Station Controls" pygame window for keyboard input to register — it's what gives `pygame.key` focus, and it also mirrors the same status lines printed to the terminal.

**Xbox controller** (if one is detected — see `init_controller()`)

| Input | Effect |
|---|---|
| Left stick Y | Forward thrust on M1 + M4 (push up = forward) |
| Left stick X | Steering — biases M1 vs M4, mirroring the keyboard's `D`/`A` |
| Right stick Y | Lift — M2 above a `HOVER_BASELINE` of `20` when pushed up, M3 when pulled down |
| `A` button | Toggle control mode (same edge-detected toggle as `M`) |

All controller-derived motor values are capped at `CONTROLLER_MAX_POWER` (`80`) and pass through a `CONTROLLER_DEADZONE` (`0.15`) to ignore stick noise near center. Axis indices (`AXIS_LEFT_X`/`AXIS_LEFT_Y`/`AXIS_RIGHT_Y` = `0`/`1`/`4`) assume the common SDL2/XInput mapping; if your sticks move the wrong motors, run:

```bash
python base_station.py --calibrate
```

This prints live axis/button values to the terminal (no serial connection needed) so you can confirm or correct the indices at the top of `base_station.py`.

The terminal shows live commanded motor state and control mode, the blimp's tracked vision blob (center/box), IMU readings, the blimp's actual motor outputs (unpacked from telemetry into `actual_motors`), and round-trip telemetry latency/FPS. On exit it prints a benchmark summary (frame count, average delta, jitter, throughput).

`actual_motors` (a `[m1, m2, m3, m4]` list, parsed each frame from the new `TelemetryPacket` fields) reflects what the blimp is really doing, as opposed to `curr_motors`/`compute_motors()`, which is only what the PC last *commanded*. The two will diverge whenever `MODE_PROPORTIONAL` is active (manual input is ignored on the blimp side) or the blimp's own watchdog has zeroed the motors — exactly the cases where an autonomous or logging consumer of this script would want the real value.

## Control modes (switch live from the keyboard)

`blimp.ino` supports two motor-control modes. The active mode is **no longer a compile-time `#define`** — it travels in every `ControlPacket` as a `mode` byte, so you flip it live from `base_station.py` by pressing `M`, without reflashing anything:

```cpp
#define MODE_MANUAL       0
#define MODE_PROPORTIONAL 1
uint8_t currentMode = MODE_MANUAL;  // safe default until the first ControlPacket arrives
```

`currentMode` updates in `OnDataRecv` whenever a valid mode value arrives, and the main loop branches on it at runtime (`if (currentMode == MODE_MANUAL) { ... } else { ... }`) instead of the old preprocessor `#if`. `base_station.py` itself always starts in `MODE_MANUAL` on launch — you opt into autonomous flight explicitly each session by pressing `M`.

### `MODE_MANUAL`

Motors are driven from the base station's `ControlPacket` — i.e. whatever `base_station.py`'s keyboard or controller input computed (see [Controls](#controls) below) — with one addition: M1 and M4 each get a small yaw-rate correction from the IMU's gyro-Z reading (`TURN_KD * iData.tz`, `TURN_KD = 25`), subtracted from M1 and added to M4. This damps unwanted yaw rotation while flying manually; it isn't purely open-loop stick input passed straight to the motors.

### `MODE_PROPORTIONAL`

Incoming manual stick input is ignored. The blimp flies forward by default and a proportional (P) controller nudges **yaw only** (left/right turning) to keep the vision blob centered:

- **Default flight:** `M1 = M4 = DEFAULT_FORWARD_POWER` (`20`), `M2 = DEFAULT_UPWARD_POWER` (`50`), `M3 = 0`, before any yaw correction is applied.
- **Yaw error** is computed in *degrees*, not pixels: `yawError = (vData.cx - MAX_W/2) * DEG_PER_PIXEL`, where `DEG_PER_PIXEL = HORIZONTAL_FOV_DEG / MAX_W` and `HORIZONTAL_FOV_DEG = 57.4`. This is calculated every loop (in both modes) so it's always available in telemetry, not just when `MODE_PROPORTIONAL` is active.
- **Deadzone:** `YAW_DEADZONE_HALF_DEG` = `5°`. No correction is applied inside this window.
- **Gain:** `YAW_GAIN_PER_DEG` = `3` — for every degree of yaw error beyond the deadzone, the correction grows by 3 (PWM units).
- **Direction:** turning is done by *cutting* power to one motor rather than boosting the other. If `yawError > 0` (target right of center), `M1 -= correction`; if `yawError < 0` (target left of center), `M4 -= correction`. Each of these also gets the same gyro yaw-rate term used in `MODE_MANUAL` (`± TURN_KD * iData.tz`, `TURN_KD = 25`).
- **"Close enough":** once the tracked blob's box is at least `180×180` px (`vData.w > 180 && vData.h > 180`), the controller stops correcting yaw and just holds forward flight. (The commented-out code in `blimp.ino` sketches out a follow-on IMU waypoint-turn behavior once "close enough" is reached, but it isn't implemented yet.)
- If no target is visible (`vData.w == 0 || vData.h == 0`), no correction is applied and the blimp continues flying straight forward at the default power levels above — it does **not** sit at zero thrust, and it does **not** currently search for a lost target (a "wiggle search" sweep exists in `blimp.ino` as commented-out, unimplemented scaffolding).
- All motor outputs are clamped to `[0, 255]` (`MOTOR_MAX`) before `analogWrite`.

**This mode still respects the control-link watchdog** (see below) — losing contact with the base station zeroes the motors regardless of what the camera sees, so there's always a way to kill the blimp by cutting the base station's link, even in autonomous mode. This does mean `base_station.py` (or something) must still be actively sending `ControlPacket`s at ~20 Hz for the blimp to leave the "stale" state, even though those packets' motor values are discarded in this mode — if nothing is sending, the blimp stays at zero thrust indefinitely.

> **`base_station.ino` and `base_station.py` must both be running, in any mode.** `base_station.ino` doesn't care what control mode is active — it's a dumb relay either way — but it's still the source of the heartbeat `ControlPacket`s the blimp's watchdog needs to leave the "stale" state (see below), and only `base_station.py` actually sets the `mode` byte. Without both running, the blimp will sit at zero thrust even with a target locked in view or `M` pressed.

## Vision tracking

`Vision.cpp` converts each captured frame to CIELAB color space via a precomputed 64K-entry lookup table, thresholds against a configured `LabThreshold`, and flood-fills to find the largest connected blob above an area threshold. Once a target is found it locks onto a padded ROI around it (`AREA_THRESHOLD_LOCKED`) for cheaper tracking on subsequent frames, and falls back to a full-frame search (`AREA_THRESHOLD_SEARCH`) if the target is lost. Tune the color threshold in `Vision.h`'s `THRESHOLD_BLIMP` constant for your target's color under your lighting.

The blob's true centroid (`largest.cx`/`largest.cy` from `findLargestBlob`) is what gets transmitted as `VisionData.cx`/`cy` and is what the proportional yaw controller reacts to — it is not the same as the padded tracking ROI's corner or size (`w`/`h`), which exist for telemetry/display only.

## Safety: control-link failsafe

`blimp.ino` tracks `lastRecvTime` (updated in `OnDataRecv`, which fires on any received `ControlPacket`) and treats the link as **stale** if more than `CONTROL_TIMEOUT_MS` (currently 1000 ms) has passed since the last packet arrived:

```cpp
bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);
```

`stale` acts as a heartbeat check, independent of control mode:

- **`MODE_MANUAL`:** if stale, motor values are forced to zero instead of using the (possibly ancient) `incomingControl` values.
- **`MODE_PROPORTIONAL`:** if stale, the yaw controller doesn't run at all — motors stay at zero — even if the camera can still see a target. This guarantees a human always has a way to stop the blimp (by killing the base station link) even while it's flying itself.

If the base station crashes, loses power, goes out of range, or a packet is simply dropped over ESP-NOW, the blimp zeroes its own motors once the timeout elapses — it doesn't depend on a shutdown command actually arriving. `base_station.py`'s `Ctrl+C` handler (which flushes and resends an all-zero `ControlPacket` a few times) is a fast-path on top of this, not the only thing standing between "quit" and "still flying."

Note the 1000 ms timeout is fairly loose relative to the ~20 Hz (50 ms) control rate — worth tightening (e.g. 200–300 ms) if you want the blimp to fail-safe faster after a lost link.

## Known limitations / suggested hardening

- `CONTROL_TIMEOUT_MS` (1000 ms) is generous compared to the control loop's ~50 ms cadence; a tighter timeout would reduce how long the blimp can drift on a stale command before self-stopping.
- The proportional yaw controller's turn direction (which of `M1`/`M4` gets cut for a given error sign) hasn't been confirmed in flight.
- The `5°` yaw deadzone (`YAW_DEADZONE_HALF_DEG`) and gain of `3`/degree (`YAW_GAIN_PER_DEG`) are untuned starting values; expect to adjust both once you see how the blimp actually responds.
- `MODE_PROPORTIONAL` has no target-reacquisition behavior yet: if the tracked blob is lost, the blimp just keeps flying straight forward rather than searching. A "wiggle search" sweep and an IMU-based waypoint turn (for once the target is "close enough") both exist as commented-out scaffolding in `blimp.ino` but aren't wired up.
- `blimp.ino`'s `use_camera` compile-time flag (currently `1`) can bypass all camera code — `Vision` object, `setup()`, `processFrame()`/`buildVisionData()` — for bring-up/testing without a camera attached. With it set to `0`, telemetry always reports `[0,0,0,0]` for `cx, cy, w, h`, which will also disable yaw error and hold `MODE_PROPORTIONAL` at "no target visible" behavior. Worth checking this is `1` before expecting vision tracking to work.
- On `base_station.ino`, `esp_now_send`'s return value and the `OnDataSent` delivery-status callback are both now logged over the same `Serial` connection used for the framed telemetry/control protocol to `base_station.py`. `base_station.py` re-syncs on the next telemetry header, so a stray debug line is tolerated (a few bytes are dropped, not a crash), but it does briefly interrupt the binary stream — worth moving to a second UART or removing the prints if you see it cause noticeable frame loss.
