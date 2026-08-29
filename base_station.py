import os
import struct
import sys
import threading
import time
import pygame
import serial

# Enable VT100 ANSI escape sequence support on Windows terminal
if sys.platform == "win32":
    os.system("")

SERIAL_PORT = "COM9"  # Adjust for your OS ('/dev/ttyUSB0' or '/dev/ttyACM0')
BAUD_RATE = 115200

FLIGHT_LOG_PATH = "flight.log"  # wiped at the start of each run, then appended to

# Protocol Markers
TELEMETRY_HEADER = b"\x00\xAA\x55\xFF"
TELEMETRY_FOOTER = b"\xEE\xFF"
CONTROL_HEADER = b"\x00\xBB\x66\xFF"

PAYLOAD_SIZE = 41  # 4x uint16 (8B) + 4x float (16B) + 4x int16 actual motors (8B) + 1x float yaw error (4B) + 1x float batt voltage (4B) + 1x uint8 state (1B)
TOTAL_FRAME_SIZE = 4 + PAYLOAD_SIZE + 2  # 47 Bytes Total Frame

# Autonomous sub-state, reported in telemetry.state (must match blimp.ino's
# STATE_MANUAL / STATE_TRACKING / STATE_TURNING / STATE_SEARCHING). Always
# STATE_MANUAL while the blimp itself is in MODE_MANUAL.
STATE_MANUAL = 0
STATE_TRACKING = 1
STATE_TURNING = 2
STATE_SEARCHING = 3
STATE_NAMES = {
    STATE_MANUAL: "MANUAL",
    STATE_TRACKING: "TRACKING",
    STATE_TURNING: "TURNING",
    STATE_SEARCHING: "SEARCHING",
}

# Yaw-to-target is now computed on the blimp (see blimp.ino: HORIZONTAL_FOV_DEG /
# DEG_PER_PIXEL / yawError) and arrives as part of the telemetry payload below.

# Must match BatteryMonitor.h's default low-battery threshold on the blimp,
# used here only to color/flag the status display -- the blimp itself is the
# authority on when the battery is actually low (its LED, if wired, reflects that).
LOW_BATTERY_THRESHOLD_V = 3.3

# Control modes (must match blimp.ino's MODE_MANUAL / MODE_PROPORTIONAL)
MODE_MANUAL = 0
MODE_PROPORTIONAL = 1
MODE_NAMES = {MODE_MANUAL: "MANUAL", MODE_PROPORTIONAL: "AUTONOMOUS (yaw-only)"}

# Xbox controller tuning
CONTROLLER_MAX_POWER = 200      # absolute cap on any motor value from the controller
CONTROLLER_DEADZONE = 0.15      # ignore stick noise near center
HOVER_BASELINE = 20             # matches the keyboard path's idle m2 value

# Axis indices are for the common SDL2/XInput mapping (Xbox 360 / Xbox One
# controllers on Windows & most Linux setups via pygame 2.x). If your sticks
# don't move the right motors, run `python base_station.py --calibrate`
# to print live axis values and adjust the indices below.
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_Y = 4
BUTTON_MODE_TOGGLE = 0  # "A" button

# Maps pygame key constants to the same single-char tokens the rest of the
# code (compute_motors, MODE toggle, etc.) already expects.
KEY_MAP = {
    pygame.K_w: "w",
    pygame.K_a: "a",
    pygame.K_d: "d",
    pygame.K_q: "q",
    pygame.K_e: "e",
    pygame.K_m: "m",
}

# Keyboard state management
active_keys = set()
current_mode = MODE_MANUAL  # start safe: manual control until the pilot opts in

# Controller state
joystick = None
controller_button_prev = set()

# Display state
screen = None
status_font = None
STATUS_BG = (15, 15, 15)
STATUS_FG = (0, 230, 110)

# --- Telemetry reader thread state -----------------------------------------
# Telemetry is parsed on a dedicated background thread so that a burst of
# buffered serial data (e.g. a backlog of frames, or debug text the firmware
# writes straight onto the same serial line) can never block the main loop's
# keyboard/controller polling or 20 Hz control-command transmission. The two
# threads only share `latest_telemetry`, guarded by `telemetry_lock`.
telemetry_lock = threading.Lock()
latest_telemetry = {
    "actual_motors": [0, 0, 0, 0],
    "cx": 0, "cy": 0, "w": 0, "h": 0,
    "ax": 0.0, "ay": 0.0, "az": 0.0, "tz": 0.0,
    "yaw_err": 0.0,
    "batt_voltage": 0.0,
    "state": STATE_MANUAL,
    "delta_ms": 0.0,
    "avg_dt": 0.0,
    "fps": 0.0,
    "total_frames": 0,
    "queue_bytes": 0,
    "frame_deltas": [],
}
stop_event = threading.Event()


def init_keyboard_window():
    """
    pygame.key needs a display surface with window focus to receive keyboard
    events, so we open a small control window. Click into it to give it focus.
    This same window also mirrors whatever gets printed to the terminal.
    """
    global screen, status_font
    screen = pygame.display.set_mode((1500, 100))
    pygame.display.set_caption("Base Station Controls (click here for keyboard focus)")
    pygame.key.set_repeat(0)  # disabled: we want press edges, not OS key-repeat
    status_font = pygame.font.SysFont("consolas,couriernew,monospace", 16)
    return screen


def render_status(lines):
    """Draw the same status lines shown in the terminal onto the pygame window."""
    if screen is None:
        return
    screen.fill(STATUS_BG)
    y = 8
    for line in lines:
        surf = status_font.render(line, True, STATUS_FG)
        screen.blit(surf, (10, y))
        y += surf.get_height() + 4
    pygame.display.flip()


def process_keyboard_events():
    """
    Poll pygame's event queue and update active_keys / current_mode.
    Returns False if the window was closed (used as a shutdown signal).
    """
    global current_mode

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False

        elif event.type == pygame.KEYDOWN:
            c = KEY_MAP.get(event.key)
            if c is not None:
                # KEYDOWN only fires once per physical press (repeat is
                # disabled above), so this is naturally the press edge.
                if c == "m":
                    current_mode = MODE_PROPORTIONAL if current_mode == MODE_MANUAL else MODE_MANUAL
                active_keys.add(c)

        elif event.type == pygame.KEYUP:
            c = KEY_MAP.get(event.key)
            if c is not None:
                active_keys.discard(c)

    return True


def compute_motors():
    m1, m2, m3, m4 = 0, HOVER_BASELINE, 0, 0

    if "w" in active_keys:
        m1 += 50
        m4 += 50
    if "a" in active_keys:
        m4 += 50
    if "d" in active_keys:
        m1 += 50
    if "q" in active_keys:
        m3 += 50
    if "e" in active_keys:
        m2 += 50

    return [m1, m2, m3, m4]


def apply_deadzone(value, deadzone=CONTROLLER_DEADZONE):
    """Zero out stick noise near center and rescale the remaining range to 0-1."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def init_controller():
    """Attempt to detect and open an Xbox (or compatible) controller. Returns None if unavailable."""
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller detected — falling back to keyboard-only control.")
        return None

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller connected: {js.get_name()}")
    return js


def compute_motors_from_controller(js):

    """
    Left stick Y  -> forward thrust on m1 + m4 (push up = forward)
    Left stick X  -> steering, biases m1 vs m4 like the keyboard 'a'/'d' keys
    Right stick Y -> lift on m2 + m3 relative to hover baseline (push up = more lift)
    """
    pygame.event.pump()
    m1 = m2 = m3 = m4 = 0
    lx = apply_deadzone(js.get_axis(0))
    ly = apply_deadzone(js.get_axis(1))
    ry = apply_deadzone(js.get_axis(3))
    forward = max(0.0, -ly)  # pushing stick up is negative on the Y axis
    turn = lx                # -1 (left) .. +1 (right)
    vertical = -ry            # pushing stick up is negative on the Y axis

    m1 = forward * CONTROLLER_MAX_POWER
    m4 = forward * CONTROLLER_MAX_POWER

    if turn > 0:      # steer right: boost m1
        m1 += turn * CONTROLLER_MAX_POWER
    elif turn < 0:     # steer left: boost m4
        m4 += -turn * CONTROLLER_MAX_POWER

    power = vertical * CONTROLLER_MAX_POWER
    if vertical > -HOVER_BASELINE:
        m2 += power + (HOVER_BASELINE if vertical > 0 else 0)
    else:
        m3 -= power

    m1 = max(0, min(CONTROLLER_MAX_POWER, m1))
    m2 = max(0, min(CONTROLLER_MAX_POWER, m2))
    m3 = max(0, min(CONTROLLER_MAX_POWER, m3))
    m4 = max(0, min(CONTROLLER_MAX_POWER, m4))

    return [int(m1), int(m2), int(m3), int(m4)]


def handle_controller_mode_toggle(js):
    """Edge-detect the mode-toggle button so a held press doesn't rapidly flip modes."""
    global current_mode, controller_button_prev

    pressed_now = set()
    for i in range(js.get_numbuttons()):
        if js.get_button(i):
            pressed_now.add(i)

    if BUTTON_MODE_TOGGLE in pressed_now and BUTTON_MODE_TOGGLE not in controller_button_prev:
        current_mode = MODE_PROPORTIONAL if current_mode == MODE_MANUAL else MODE_MANUAL

    controller_button_prev = pressed_now


def calibrate_controller():
    """Utility mode: prints live axis/button values so you can confirm indices. Run with --calibrate."""
    pygame.init()
    js = init_controller()
    if js is None:
        sys.exit(1)
    print("Move sticks / press buttons. Ctrl+C to quit.\n")
    try:
        while True:
            pygame.event.pump()
            axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
            buttons = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            sys.stdout.write(f"\rAxes: {axes}  Buttons: {buttons}   \033[K")
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone.")


def pack_control(motors, mode):
    # 4x int16 motor values + 1x uint8 mode flag, matching blimp.ino's ControlPacket
    return struct.pack("<4hB", *motors, mode)


def telemetry_reader_loop(ser):
    """
    Runs on its own thread for the lifetime of the connection.

    All serial *reading* and frame parsing happens here, isolated from the
    main loop. Previously this work was interleaved with keyboard/controller
    polling and control-command transmission on a single thread, so any
    backlog of frames (or a chunk of debug text the firmware wrote straight
    onto the same serial line) had to be fully drained/resynced before the
    next control packet could go out -- stalling the whole program. Now the
    main loop only ever touches `latest_telemetry` (a plain dict guarded by
    `telemetry_lock`) and never waits on this thread.
    """
    buffer = bytearray()
    last_frame_time = None
    frame_deltas = []
    max_history = 200  # Rolling window size for averaging
    total_frames = 0

    while not stop_event.is_set():
        try:
            n = ser.in_waiting
            if n:
                buffer.extend(ser.read(n))
            else:
                time.sleep(0.001)
                continue
        except serial.SerialException:
            # Port closed/unplugged; let the main thread handle shutdown.
            break

        # Parse ALL complete telemetry frames currently sitting in the buffer.
        while len(buffer) >= TOTAL_FRAME_SIZE:
            idx = buffer.find(TELEMETRY_HEADER)
            if idx == -1:
                # No header at all: keep the last few bytes in case a header
                # is split across reads, discard the rest.
                del buffer[:-3]
                break
            if idx > 0:
                del buffer[:idx]

            if len(buffer) < TOTAL_FRAME_SIZE:
                break  # Header found but the rest of the frame hasn't arrived yet

            if buffer[4 + PAYLOAD_SIZE : TOTAL_FRAME_SIZE] == TELEMETRY_FOOTER:
                raw_payload = bytes(buffer[4 : 4 + PAYLOAD_SIZE])
                del buffer[:TOTAL_FRAME_SIZE]

                recv_time = time.perf_counter()
                delta_ms = 0.0
                if last_frame_time is not None:
                    total_frames += 1
                    delta_ms = (recv_time - last_frame_time) * 1000.0
                    frame_deltas.append(delta_ms)
                    if len(frame_deltas) > max_history:
                        frame_deltas.pop(0)
                last_frame_time = recv_time

                avg_dt = sum(frame_deltas) / len(frame_deltas) if frame_deltas else 0.0
                fps = 1000.0 / avg_dt if avg_dt > 0 else 0.0

                cx, cy, w, h, ax, ay, az, tz, m1, m2, m3, m4, yaw_err, batt_voltage, state = struct.unpack(
                    "<4H4f4hffB", raw_payload
                )

                with telemetry_lock:
                    latest_telemetry.update(
                        {
                            "actual_motors": [m1, m2, m3, m4],
                            "cx": cx, "cy": cy, "w": w, "h": h,
                            "ax": ax, "ay": ay, "az": az, "tz": tz,
                            "yaw_err": yaw_err,
                            "batt_voltage": batt_voltage,
                            "state": state,
                            "delta_ms": delta_ms,
                            "avg_dt": avg_dt,
                            "fps": fps,
                            "total_frames": total_frames,
                            "queue_bytes": ser.in_waiting,
                            "frame_deltas": list(frame_deltas),
                        }
                    )
            else:
                # Header matched but footer didn't line up (false-positive
                # header bytes, e.g. from interleaved debug text) -- resync
                # one byte at a time.
                del buffer[:1]


def main():
    global joystick

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Connected to Base Station on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        sys.exit(1)

    # Clear OS buffer queue lag on launch
    ser.reset_input_buffer()

    # Wipe flight.log at the start of each run, then keep it open for
    # appending timestamped telemetry/command/latency lines for the
    # duration of this run.
    flight_log = open(FLIGHT_LOG_PATH, "w")

    # Telemetry reading/parsing runs on its own thread so it can never stall
    # the keyboard/controller polling or control-command transmission below.
    reader_thread = threading.Thread(target=telemetry_reader_loop, args=(ser,), daemon=True)
    reader_thread.start()

    # pygame powers both keyboard capture and the Xbox controller
    pygame.init()
    init_keyboard_window()

    # Try to bring up an Xbox controller; keyboard still works either way
    joystick = init_controller()

    print("Control & Benchmark Active")
    print("Controls: Hold 'W' (M1+M2=25) | 'A' (+M1=25) | 'D' (+M2=25) | 'Q' (M3=25) | 'E' (M4=25)")
    print("Press 'M' to toggle MANUAL <-> AUTONOMOUS (yaw-only) mode")
    print("NOTE: click into the small 'Base Station Controls' window for keyboard input to register")
    if joystick is not None:
        print("Xbox controller: Left stick = forward/steer | Right stick Y = up/down | 'A' = toggle mode")
    print("Press Ctrl+C, or close the control window, to quit\n\n")

    last_control_time = 0
    status_printed = False  # tracks whether the 3-line status block exists yet

    # Tracks the last frame count we've already displayed, so the status
    # block only redraws when the reader thread has actually delivered a
    # new telemetry frame.
    last_seen_frame_count = 0
    start_bench_time = time.perf_counter()

    try:
        while True:
            # 1. Poll the keyboard window; a closed window shuts things down cleanly
            if not process_keyboard_events():
                raise KeyboardInterrupt

            # 2. Transmit Motor Control Commands (~20 Hz)
            now = time.perf_counter()
            if current_mode == MODE_MANUAL:
                if joystick is not None:
                    handle_controller_mode_toggle(joystick)
                    command_motors = compute_motors_from_controller(joystick)
                else:
                    command_motors = compute_motors()
            else:
                command_motors = [0, 0, 0, 0]

            if now - last_control_time >= 0.05:
                last_control_time = now
                payload = pack_control(command_motors, current_mode)
                ser.write(CONTROL_HEADER + payload)

            # 3. Pull the latest telemetry snapshot from the reader thread.
            # This is just a dict copy under a lock -- never a serial read --
            # so it can't stall this loop even if the reader thread is busy
            # working through a large backlog of buffered bytes.
            with telemetry_lock:
                telemetry_snapshot = dict(latest_telemetry)

            if telemetry_snapshot["total_frames"] != last_seen_frame_count:
                last_seen_frame_count = telemetry_snapshot["total_frames"]

                actual_motors = telemetry_snapshot["actual_motors"]
                cx, cy, w, h = (telemetry_snapshot[k] for k in ("cx", "cy", "w", "h"))
                ax, ay, az, tz = (telemetry_snapshot[k] for k in ("ax", "ay", "az", "tz"))
                yaw_err = telemetry_snapshot["yaw_err"]
                batt_voltage = telemetry_snapshot["batt_voltage"]
                state = telemetry_snapshot["state"]
                delta_ms = telemetry_snapshot["delta_ms"]
                avg_dt = telemetry_snapshot["avg_dt"]
                fps = telemetry_snapshot["fps"]

                # Build the status text once, then mirror it to both
                # the terminal and the pygame window.
                batt_flag = " LOW!" if batt_voltage <= LOW_BATTERY_THRESHOLD_V else ""
                telemetry_line = (
                    f"[TELEMETRY] Motors: {actual_motors} || "
                    f"Vision: CX:{cx:3d} CY:{cy:3d} W:{w:3d} H:{h:3d} Yaw:{yaw_err:+5.1f}deg || "
                    f"IMU: AX:{ax:5.1f} AY:{ay:5.1f} AZ:{az:5.1f} TZ:{tz:5.1f} || "
                    f"Batt: {batt_voltage:4.2f}V {batt_flag} || "
                    f"State: {STATE_NAMES.get(state, f'UNKNOWN({state})'):<9}"
                )
                command_line = (
                    f"[COMMAND] Mode: {MODE_NAMES[current_mode]:<21} || Motors: {command_motors}"
                )
                latency_line = (
                    f"[LATENCY] Delta: {delta_ms:5.1f}ms | Avg: {avg_dt:5.1f}ms | "
                    f"Rate: {fps:4.1f} FPS | Queue: {telemetry_snapshot['queue_bytes']}B"
                )

                # Terminal display: redraw the 3-line block in place.
                # Once the block has been printed once, jump the
                # cursor back up to its top-left corner before
                # rewriting all three lines so nothing scrolls.
                if status_printed:
                    sys.stdout.write("\033[3A")
                sys.stdout.write(
                    f"\r\033[K{telemetry_line}\n"
                    f"\r\033[K{command_line}\n"
                    f"\r\033[K{latency_line}\n"
                )
                sys.stdout.flush()
                status_printed = True

                # pygame window display (same three lines)
                render_status([telemetry_line, command_line, latency_line])

                # Flight log: timestamped copy of the same three lines,
                # flushed immediately so the file is current if the program
                # is killed rather than exited cleanly.
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"
                flight_log.write(
                    f"[{timestamp}]\n{telemetry_line}\n{command_line}\n{latency_line}\n\n"
                )
                flight_log.flush()

            time.sleep(0.001)

    except KeyboardInterrupt:
        motors = [0, 0, 0, 0]
        # Force MANUAL mode here so a zero ControlPacket actually zeroes thrust,
        # even if AUTONOMOUS mode was active when Ctrl+C was hit.
        payload = pack_control(motors, MODE_MANUAL)
        ser.write(CONTROL_HEADER + payload)
        print("\n\n\n--- Benchmark Summary ---")
        with telemetry_lock:
            final_snapshot = dict(latest_telemetry)
        frame_deltas = final_snapshot["frame_deltas"]
        total_frames = final_snapshot["total_frames"]
        if frame_deltas:
            total_time = time.perf_counter() - start_bench_time
            avg_ms = sum(frame_deltas) / len(frame_deltas)
            print(f"Total Frames Received : {total_frames}")
            print(f"Total Test Duration   : {total_time:.2f} s")
            print(f"Average Frame Delta   : {avg_ms:.2f} ms")
            print(f"Min / Max Delta Jitter: {min(frame_deltas):.2f} ms / {max(frame_deltas):.2f} ms")
            print(f"Average Throughput    : {total_frames / total_time:.2f} FPS")
        print("Exiting...")
    finally:
        motors = [0, 0, 0, 0]
        payload = pack_control(motors, MODE_MANUAL)
        ser.write(CONTROL_HEADER + payload)
        for _ in range(5):
            ser.write(CONTROL_HEADER + payload)
            ser.flush()
            time.sleep(0.02)

        # Stop the reader thread before closing the port out from under it.
        stop_event.set()
        reader_thread.join(timeout=1.0)

        ser.close()
        flight_log.close()
        if joystick is not None:
            pygame.joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    if "--calibrate" in sys.argv:
            calibrate_controller()
    else:
        main()
