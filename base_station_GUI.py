#!/usr/bin/env python3
"""
ESP-FLY Blimp Ground Control -- GUI edition
============================================

A drop-in graphical replacement for base_station.py's tiny 3-line status
window. Same serial protocol, same keyboard/Xbox controller scheme, same
flight.log output -- the only thing that changes is the display: instead
of a 1500x100 text window + terminal printout, telemetry is rendered
live with the full panel set from blimp_dashboard.py (motor thrust,
IMU attitude, yaw error, vision/target tracking, state machine, battery,
link latency), plus a live control-status strip (active keys, controller
state, control mode).

This file does NOT reimplement the serial protocol, motor-mixing math,
or controller handling -- it imports all of that straight from
base_station.py so the two stay in lockstep and there is exactly one
place that ever needs to change if the wire protocol changes.

Requires (same directory):
    base_station.py       - serial protocol, control mixing, telemetry reader
    blimp_dashboard.py     - drawing primitives shared with the log-replay tool

Usage:
    python base_station_GUI.py                        # uses base_station.SERIAL_PORT
    python base_station_GUI.py --port /dev/ttyUSB0
    python base_station_GUI.py --port COM5 --baud 115200

Controls: identical to base_station.py --
    WASD / QE   manual flight movement
    M           toggle control mode (MANUAL <-> AUTONOMOUS)
    Xbox pad    left stick = forward/steer, right stick Y = up/down, A = mode toggle
    ESC / close window   safe zero-thrust shutdown and quit
"""

import argparse
import os
import sys
import threading
import time

import pygame
import serial

# Make sure sibling modules import regardless of cwd.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import base_station as bs          # noqa: E402  (serial protocol / control logic)
import blimp_dashboard as bd       # noqa: E402  (drawing primitives / panels)


# --------------------------------------------------------------------------
# Live control-status strip (keys / controller / mode) -- the one bit of UI
# base_station.py didn't have an equivalent panel for.
# --------------------------------------------------------------------------

DISPLAY_KEYS = ["w", "a", "d", "q", "e"]
KEY_CAPTIONS = {"w": "W", "a": "A", "d": "D", "q": "Q", "e": "E"}


def draw_controls_strip(surface, rect, joystick, font, font_small):
    pygame.draw.rect(surface, bd.PANEL_BG, rect, border_radius=8)
    pygame.draw.rect(surface, bd.PANEL_BORDER, rect, width=1, border_radius=8)

    x = rect.x + 14
    y = rect.y + rect.height // 2

    bd.draw_text(surface, "KEYS", (x, rect.y + 6), font_small, bd.TEXT_DIM)
    key_w = 30
    for i, k in enumerate(DISPLAY_KEYS):
        kx = x + i * (key_w + 6)
        active = k in bs.active_keys
        box = pygame.Rect(kx, y - 12, key_w, 26)
        color = bd.GREEN if active else (32, 37, 45)
        border = bd.GREEN if active else bd.PANEL_BORDER
        pygame.draw.rect(surface, color, box, border_radius=4)
        pygame.draw.rect(surface, border, box, width=1, border_radius=4)
        txt_color = (10, 12, 16) if active else bd.TEXT_DIM
        bd.draw_text(surface, KEY_CAPTIONS[k], box.center, font_small, txt_color, align="center")

    cx = x + len(DISPLAY_KEYS) * (key_w + 6) + 24
    if joystick is not None:
        ctrl_txt = f"CONTROLLER: {joystick.get_name()}"
        ctrl_color = bd.GREEN
    else:
        ctrl_txt = "CONTROLLER: none (keyboard only)"
        ctrl_color = bd.TEXT_DIM
    bd.draw_text(surface, ctrl_txt, (cx, rect.y + 6), font_small, ctrl_color)
    bd.draw_text(surface, "M toggle mode  |  ESC / close window: safe shutdown & quit",
                 (cx, y + 2), font_small, bd.TEXT_DIM)

    mode_txt = bs.MODE_NAMES[bs.current_mode]
    mode_color = bd.GREEN if bs.current_mode == bs.MODE_PROPORTIONAL else bd.ACCENT
    bd.draw_text(surface, f"MODE: {mode_txt}", (rect.right - 14, rect.y + rect.height // 2),
                 font, mode_color, align="right")


# --------------------------------------------------------------------------
# Main application
# --------------------------------------------------------------------------

class GroundControlApp:
    def __init__(self, port, baud, width=1180, height=820):
        self.port = port
        self.baud = baud

        pygame.init()
        pygame.display.set_caption(f"ESP-FLY Blimp Ground Control (LIVE) - {port}")
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.key.set_repeat(0)  # press edges, not OS key-repeat -- matches base_station.py
        self.clock = pygame.time.Clock()

        self.font_big = pygame.font.SysFont("consolas,menlo,monospace", 22, bold=True)
        self.font = pygame.font.SysFont("consolas,menlo,monospace", 16)
        self.font_small = pygame.font.SysFont("consolas,menlo,monospace", 13)
        self.font_title = pygame.font.SysFont("consolas,menlo,monospace", 13, bold=True)

        self.batt_spark = bd.Sparkline()
        self.rate_spark = bd.Sparkline()
        self.delta_spark = bd.Sparkline()

        self.ser = None
        self.flight_log = None
        self.reader_thread = None
        self.joystick = None

        self.running = True
        self.last_control_time = 0.0
        self.last_seen_frame_count = 0
        self.start_bench_time = time.perf_counter()
        self.last_frame = bd.Frame()  # placeholder until first telemetry arrives

    # ---- setup / teardown -------------------------------------------------

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            print(f"Connected to Base Station on {self.port}")
        except Exception as e:
            print(f"Failed to open serial port {self.port}: {e}")
            pygame.quit()
            sys.exit(1)

        self.ser.reset_input_buffer()
        self.flight_log = open(bs.FLIGHT_LOG_PATH, "w")

        bs.stop_event.clear()
        self.reader_thread = threading.Thread(target=bs.telemetry_reader_loop, args=(self.ser,), daemon=True)
        self.reader_thread.start()

        self.joystick = bs.init_controller()

    def shutdown(self):
        if self.ser is not None and self.ser.is_open:
            zero_payload = bs.pack_control([0, 0, 0, 0], bs.MODE_MANUAL)
            for _ in range(5):
                try:
                    self.ser.write(bs.CONTROL_HEADER + zero_payload)
                    self.ser.flush()
                except Exception:
                    break
                time.sleep(0.02)

        bs.stop_event.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        if self.flight_log is not None:
            self.flight_log.close()
        if self.joystick is not None:
            pygame.joystick.quit()

        with bs.telemetry_lock:
            frame_deltas = list(bs.latest_telemetry["frame_deltas"])
            total_frames = bs.latest_telemetry["total_frames"]
        print("\n--- Benchmark Summary ---")
        if frame_deltas:
            total_time = time.perf_counter() - self.start_bench_time
            avg_ms = sum(frame_deltas) / len(frame_deltas)
            print(f"Total Frames Received : {total_frames}")
            print(f"Total Test Duration   : {total_time:.2f} s")
            print(f"Average Frame Delta   : {avg_ms:.2f} ms")
            print(f"Min / Max Delta Jitter: {min(frame_deltas):.2f} ms / {max(frame_deltas):.2f} ms")
            print(f"Average Throughput    : {total_frames / total_time:.2f} FPS")
        print("Exiting...")

        pygame.quit()

    # ---- per-frame steps ----------------------------------------------------

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    continue
                c = bs.KEY_MAP.get(event.key)
                if c is not None:
                    if c == "m":
                        bs.current_mode = (
                            bs.MODE_PROPORTIONAL if bs.current_mode == bs.MODE_MANUAL else bs.MODE_MANUAL
                        )
                    bs.active_keys.add(c)
            elif event.type == pygame.KEYUP:
                c = bs.KEY_MAP.get(event.key)
                if c is not None:
                    bs.active_keys.discard(c)

    def send_control(self):
        now = time.perf_counter()
        if bs.current_mode == bs.MODE_MANUAL:
            if self.joystick is not None:
                bs.handle_controller_mode_toggle(self.joystick)
                command_motors = bs.compute_motors_from_controller(self.joystick)
            else:
                command_motors = bs.compute_motors()
        else:
            command_motors = [0, 0, 0, 0]

        if now - self.last_control_time >= 0.05:  # ~20 Hz, matches base_station.py
            self.last_control_time = now
            payload = bs.pack_control(command_motors, bs.current_mode)
            try:
                self.ser.write(bs.CONTROL_HEADER + payload)
            except Exception as e:
                print(f"Serial write failed: {e}")
                self.running = False

        return command_motors

    def build_frame(self, command_motors):
        with bs.telemetry_lock:
            snap = dict(bs.latest_telemetry)

        f = bd.Frame()
        f.timestamp = time.strftime("%Y-%m-%d %H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"
        f.motors_actual = list(snap["actual_motors"])
        f.cx, f.cy, f.w, f.h, f.pixels = (snap[k] for k in ("cx", "cy", "w", "h", "pixels"))
        f.ax, f.ay, f.az, f.tz = (snap[k] for k in ("ax", "ay", "az", "tz"))
        f.yaw_error = snap["yaw_err"]
        f.batt = snap["batt_voltage"]
        f.state = bs.STATE_NAMES.get(snap["state"], "MANUAL")
        f.cmd_mode = "AUTONOMOUS" if bs.current_mode == bs.MODE_PROPORTIONAL else "MANUAL"
        f.cmd_motors = list(command_motors)
        f.lat_delta = snap["delta_ms"]
        f.lat_avg = snap["avg_dt"]
        f.rate_fps = snap["fps"]
        f.queue = snap["queue_bytes"]
        return f, snap["total_frames"]

    def log_frame(self, frame):
        """Same three-line text block base_station.py writes, so flight.log stays
        readable by blimp_dashboard.py's replay parser."""
        batt_flag = " LOW!" if frame.batt <= bs.LOW_BATTERY_THRESHOLD_V else ""
        telemetry_line = (
            f"[TELEMETRY] Motors: {frame.motors_actual} || "
            f"Vision: CX:{frame.cx:3d} CY:{frame.cy:3d} W:{frame.w:3d} H:{frame.h:3d} Px:{frame.pixels:5d} || "
            f"IMU: AX:{frame.ax:5.1f} AY:{frame.ay:5.1f} AZ:{frame.az:5.1f} TZ:{frame.tz:5.1f} || "
            f"Yaw Error:{frame.yaw_error:+5.1f}deg || "
            f"Batt: {frame.batt:4.2f}V {batt_flag} || "
            f"State: {frame.state:<9}"
        )
        command_line = (
            f"[COMMAND] Mode: {bs.MODE_NAMES[bs.current_mode]:<21} || Motors: {frame.cmd_motors}"
        )
        latency_line = (
            f"[LATENCY] Delta: {frame.lat_delta:5.1f}ms | Avg: {frame.lat_avg:5.1f}ms | "
            f"Rate: {frame.rate_fps:4.1f} FPS | Queue: {frame.queue}B"
        )
        self.flight_log.write(f"[{frame.timestamp}]\n{telemetry_line}\n{command_line}\n{latency_line}\n\n")
        self.flight_log.flush()

    # ---- drawing ------------------------------------------------------------

    def layout(self, w, h):
        margin = 14
        header_h = 50
        controls_h = 54
        top = margin + header_h + 8 + controls_h + 8
        content_h = h - top - margin

        left_w = int(w * 0.42)
        right_w = w - left_w - margin * 3
        col1_x = margin
        col2_x = margin * 2 + left_w

        rects = {
            "header": pygame.Rect(margin, margin, w - margin * 2, header_h),
            "controls": pygame.Rect(margin, margin + header_h + 8, w - margin * 2, controls_h),
        }

        motor_h = int(content_h * 0.52)
        rects["motors"] = pygame.Rect(col1_x, top, left_w, motor_h)
        sub_w = left_w // 2 - 6
        rects["imu"] = pygame.Rect(col1_x, top + motor_h + 12, sub_w, content_h - motor_h - 12)
        rects["yaw"] = pygame.Rect(col1_x + sub_w + 12, top + motor_h + 12, sub_w, content_h - motor_h - 12)

        vision_h = int(content_h * 0.42)
        state_h = 92
        remain_h = content_h - vision_h - state_h - 24
        rects["vision"] = pygame.Rect(col2_x, top, right_w, vision_h)
        rects["state"] = pygame.Rect(col2_x, top + vision_h + 12, right_w, state_h)
        batt_w = right_w // 2 - 6
        rects["battery"] = pygame.Rect(col2_x, top + vision_h + state_h + 24, batt_w, remain_h)
        rects["latency"] = pygame.Rect(col2_x + batt_w + 12, top + vision_h + state_h + 24, batt_w, remain_h)
        return rects

    def draw_header(self, rect, frame):
        bd.draw_text(self.screen, "ESP-FLY BLIMP GROUND CONTROL", (rect.x, rect.y), self.font_big, bd.ACCENT)
        bd.draw_text(self.screen, frame.timestamp or "--", (rect.right, rect.y + 2), self.font, bd.TEXT_DIM,
                     align="right")

        connected = self.ser is not None and self.ser.is_open
        conn_color = bd.GREEN if connected else bd.RED
        conn_txt = f"LIVE  {self.port} @ {self.baud}  frames {self.last_seen_frame_count}"
        bd.draw_text(self.screen, conn_txt, (rect.x, rect.y + 26), self.font_small, conn_color)

        if frame.batt > 0 and frame.batt <= bs.LOW_BATTERY_THRESHOLD_V:
            bd.draw_text(self.screen, "LOW BATTERY", (rect.right, rect.y + 26), self.font_small, bd.RED,
                         align="right")

    def draw(self, frame):
        w, h = self.screen.get_size()
        self.screen.fill(bd.BG)
        rects = self.layout(w, h)

        self.draw_header(rects["header"], frame)
        draw_controls_strip(self.screen, rects["controls"], self.joystick, self.font, self.font_small)

        inner = bd.panel(self.screen, rects["motors"], "MOTORS (top-down)", self.font_title)
        bd.draw_motor_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = bd.panel(self.screen, rects["imu"], "IMU / ATTITUDE", self.font_title)
        bd.draw_imu_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = bd.panel(self.screen, rects["yaw"], "YAW ERROR", self.font_title)
        bd.draw_yaw_gauge(self.screen, inner, frame, self.font, self.font_small)

        inner = bd.panel(self.screen, rects["vision"], "VISION / TARGET TRACKING", self.font_title)
        bd.draw_vision_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = bd.panel(self.screen, rects["state"], "STATE MACHINE", self.font_title)
        bd.draw_state_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = bd.panel(self.screen, rects["battery"], "BATTERY", self.font_title)
        bd.draw_battery_panel(self.screen, inner, frame, self.batt_spark, self.font, self.font_small)

        inner = bd.panel(self.screen, rects["latency"], "LINK / LATENCY", self.font_title)
        bd.draw_latency_panel(self.screen, inner, frame, self.rate_spark, self.delta_spark,
                               self.font, self.font_small)

        pygame.display.flip()

    # ---- main loop ------------------------------------------------------------

    def run(self):
        self.connect()
        print("Control & Benchmark Active")
        print("Controls: Hold 'W'/'A'/'D'/'Q'/'E' to fly, 'M' to toggle MANUAL <-> AUTONOMOUS")
        if self.joystick is not None:
            print("Xbox controller: Left stick = forward/steer | Right stick Y = up/down | 'A' = toggle mode")
        print("Press ESC, or close the window, to quit\n")

        try:
            while self.running:
                self.clock.tick(60)
                self.handle_events()
                if not self.running:
                    break

                command_motors = self.send_control()
                frame, total_frames = self.build_frame(command_motors)
                self.last_frame = frame

                if total_frames != self.last_seen_frame_count:
                    self.last_seen_frame_count = total_frames
                    self.batt_spark.push(frame.batt)
                    self.rate_spark.push(frame.rate_fps)
                    self.delta_spark.push(frame.lat_delta)
                    self.log_frame(frame)

                self.draw(frame)
        finally:
            self.shutdown()


def main():
    ap = argparse.ArgumentParser(description="ESP-FLY blimp ground control (GUI)")
    ap.add_argument("--port", default=bs.SERIAL_PORT, help=f"serial port (default: {bs.SERIAL_PORT})")
    ap.add_argument("--baud", type=int, default=bs.BAUD_RATE, help=f"baud rate (default: {bs.BAUD_RATE})")
    args = ap.parse_args()

    app = GroundControlApp(args.port, args.baud)
    app.run()


if __name__ == "__main__":
    if "--calibrate" in sys.argv:
        bs.calibrate_controller()
    else:
        main()
