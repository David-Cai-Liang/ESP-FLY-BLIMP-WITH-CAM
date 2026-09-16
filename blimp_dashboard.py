#!/usr/bin/env python3
"""
ESP-FLY Blimp Telemetry Dashboard
==================================

A pygame-based ground-control visualizer for the ESP-FLY autonomous blimp.

It can run in two modes:
  1. REPLAY  - parses a flight.log file produced by base_station.py and
               plays it back frame by frame (play/pause/scrub/speed).
  2. LIVE    - reads newline-framed telemetry from a serial port using the
               same text format base_station.py prints, OR tails a growing
               log file (e.g. `tail -f flight.log`) in real time.

Usage:
    python blimp_dashboard.py flight.log            # replay a log file
    python blimp_dashboard.py --live COM5           # live serial (pyserial)
    python blimp_dashboard.py --tail flight.log      # tail a live-growing log

Controls (replay mode):
    SPACE       play / pause
    LEFT/RIGHT  step one frame back / forward (while paused)
    UP/DOWN     increase / decrease playback speed
    R           restart from beginning
    click bar   scrub to a point in the log
    ESC / Q     quit

Only the Python standard library + pygame are required for log replay.
`pyserial` is only needed for --live serial mode.
"""

import sys
import re
import math
import argparse
import time
from collections import deque

import pygame

# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------

# Example telemetry line (see base_station.py output / README):
# [TELEMETRY] Motors: [0, 0, 0, 0] || Vision: CX:  0 CY:  0 W:  0 H:  0 Px:    0
#   || IMU: AX: -0.4 AY: -0.7 AZ: -8.8 TZ: -0.0 || Yaw Error:-28.7deg
#   || Batt: 3.58V  || State: MANUAL
TELEMETRY_RE = re.compile(
    r"\[TELEMETRY\]\s*Motors:\s*\[([-\d,\s]+)\]\s*\|\|\s*"
    r"Vision:\s*CX:\s*(-?\d+)\s*CY:\s*(-?\d+)\s*W:\s*(-?\d+)\s*H:\s*(-?\d+)\s*Px:\s*(-?\d+)\s*\|\|\s*"
    r"IMU:\s*AX:\s*(-?[\d.]+)\s*AY:\s*(-?[\d.]+)\s*AZ:\s*(-?[\d.]+)\s*TZ:\s*(-?[\d.]+)\s*\|\|\s*"
    r"Yaw Error:\s*(-?[\d.]+)deg\s*\|\|\s*"
    r"Batt:\s*([\d.]+)V\s*\|\|\s*"
    r"State:\s*(\w+)"
)

# [COMMAND] Mode: MANUAL                || Motors: [0, 20, 0, 0]
COMMAND_RE = re.compile(
    r"\[COMMAND\]\s*Mode:\s*(\w+)\s*\|\|\s*Motors:\s*\[([-\d,\s]+)\]"
)

# [LATENCY] Delta:   4.0ms | Avg:   4.6ms | Rate: 219.5 FPS | Queue: 0B
LATENCY_RE = re.compile(
    r"\[LATENCY\]\s*Delta:\s*([\d.]+)ms\s*\|\s*Avg:\s*([\d.]+)ms\s*\|\s*"
    r"Rate:\s*([\d.]+)\s*FPS\s*\|\s*Queue:\s*(\d+)B"
)

TIMESTAMP_RE = re.compile(r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\]")


def _parse_motor_list(s):
    return [int(x.strip()) for x in s.split(",") if x.strip() != ""]


class Frame:
    """One fully-assembled telemetry/command/latency sample."""

    __slots__ = (
        "timestamp", "motors_actual", "cx", "cy", "w", "h", "pixels",
        "ax", "ay", "az", "tz", "yaw_error", "batt", "state",
        "cmd_mode", "cmd_motors",
        "lat_delta", "lat_avg", "rate_fps", "queue",
    )

    def __init__(self):
        self.timestamp = None
        self.motors_actual = [0, 0, 0, 0]
        self.cx = self.cy = self.w = self.h = self.pixels = 0
        self.ax = self.ay = self.az = self.tz = 0.0
        self.yaw_error = 0.0
        self.batt = 0.0
        self.state = "UNKNOWN"
        self.cmd_mode = "UNKNOWN"
        self.cmd_motors = [0, 0, 0, 0]
        self.lat_delta = self.lat_avg = self.rate_fps = 0.0
        self.queue = 0


def parse_log(path):
    """Parse a full flight.log file into a list of Frame objects."""
    frames = []
    current = None
    with open(path, "r", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            ts_match = TIMESTAMP_RE.fullmatch(line)
            if ts_match:
                if current is not None:
                    frames.append(current)
                current = Frame()
                current.timestamp = ts_match.group(1)
                continue

            if current is None:
                continue

            m = TELEMETRY_RE.search(line)
            if m:
                current.motors_actual = _parse_motor_list(m.group(1))
                current.cx = int(m.group(2))
                current.cy = int(m.group(3))
                current.w = int(m.group(4))
                current.h = int(m.group(5))
                current.pixels = int(m.group(6))
                current.ax = float(m.group(7))
                current.ay = float(m.group(8))
                current.az = float(m.group(9))
                current.tz = float(m.group(10))
                current.yaw_error = float(m.group(11))
                current.batt = float(m.group(12))
                current.state = m.group(13)
                continue

            m = COMMAND_RE.search(line)
            if m:
                current.cmd_mode = m.group(1)
                current.cmd_motors = _parse_motor_list(m.group(2))
                continue

            m = LATENCY_RE.search(line)
            if m:
                current.lat_delta = float(m.group(1))
                current.lat_avg = float(m.group(2))
                current.rate_fps = float(m.group(3))
                current.queue = int(m.group(4))
                continue

    if current is not None:
        frames.append(current)
    # drop frames that never got a TELEMETRY line (incomplete/trailing)
    return [fr for fr in frames if fr.state != "UNKNOWN" or fr.timestamp is not None]


class LiveLineAssembler:
    """Feed it raw lines one at a time (from a socket/serial/tail) and it
    yields completed Frame objects, same schema as parse_log()."""

    def __init__(self):
        self.current = None

    def feed_line(self, line):
        line = line.strip()
        if not line:
            return None
        ts_match = TIMESTAMP_RE.fullmatch(line)
        if ts_match:
            done = self.current
            self.current = Frame()
            self.current.timestamp = ts_match.group(1)
            return done

        if self.current is None:
            return None

        m = TELEMETRY_RE.search(line)
        if m:
            c = self.current
            c.motors_actual = _parse_motor_list(m.group(1))
            c.cx, c.cy, c.w, c.h, c.pixels = (int(m.group(i)) for i in (2, 3, 4, 5, 6))
            c.ax, c.ay, c.az, c.tz = (float(m.group(i)) for i in (7, 8, 9, 10))
            c.yaw_error = float(m.group(11))
            c.batt = float(m.group(12))
            c.state = m.group(13)
            return None

        m = COMMAND_RE.search(line)
        if m:
            self.current.cmd_mode = m.group(1)
            self.current.cmd_motors = _parse_motor_list(m.group(2))
            return None

        m = LATENCY_RE.search(line)
        if m:
            c = self.current
            c.lat_delta, c.lat_avg, c.rate_fps = (float(m.group(i)) for i in (1, 2, 3))
            c.queue = int(m.group(4))
            return None
        return None


# --------------------------------------------------------------------------
# Visual theme
# --------------------------------------------------------------------------

BG = (14, 17, 22)
PANEL_BG = (22, 26, 33)
PANEL_BORDER = (45, 52, 63)
TEXT = (222, 227, 233)
TEXT_DIM = (128, 138, 150)
ACCENT = (86, 182, 255)
GREEN = (94, 214, 130)
YELLOW = (230, 196, 70)
RED = (230, 90, 90)
ORANGE = (235, 150, 70)
MOTOR_COLOR = (86, 182, 255)
GRID_COLOR = (35, 40, 49)

STATE_COLORS = {
    "MANUAL": ACCENT,
    "STATE_TRACKING": GREEN,
    "TRACKING": GREEN,
    "STATE_TURNING": ORANGE,
    "TURNING": ORANGE,
    "STATE_SEARCHING": YELLOW,
    "SEARCHING": YELLOW,
}


def lerp_color(c1, c2, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


def batt_color(v):
    # rough single-cell LiPo curve: 4.2 full, 3.5 nominal-low, 3.2 critical
    if v >= 3.9:
        return GREEN
    if v >= 3.5:
        return lerp_color(YELLOW, GREEN, (v - 3.5) / 0.4)
    return lerp_color(RED, YELLOW, max(0.0, (v - 3.2) / 0.3))


# --------------------------------------------------------------------------
# Drawing helpers
# --------------------------------------------------------------------------

def panel(surface, rect, title, font_title):
    pygame.draw.rect(surface, PANEL_BG, rect, border_radius=8)
    pygame.draw.rect(surface, PANEL_BORDER, rect, width=1, border_radius=8)
    if title:
        label = font_title.render(title, True, TEXT_DIM)
        surface.blit(label, (rect.x + 12, rect.y + 8))
    return pygame.Rect(rect.x + 12, rect.y + 30, rect.width - 24, rect.height - 42)


def draw_text(surface, text, pos, font, color=TEXT, align="left"):
    img = font.render(text, True, color)
    r = img.get_rect()
    if align == "left":
        r.topleft = pos
    elif align == "center":
        r.midtop = pos
    elif align == "right":
        r.topright = pos
    surface.blit(img, r)
    return r


class Sparkline:
    """Rolling strip-chart for a scalar over time."""

    def __init__(self, maxlen=240):
        self.data = deque(maxlen=maxlen)

    def push(self, v):
        self.data.append(v)

    def draw(self, surface, rect, color, font, label="", lo=None, hi=None, unit=""):
        pygame.draw.rect(surface, (17, 20, 26), rect, border_radius=4)
        pygame.draw.rect(surface, GRID_COLOR, rect, width=1, border_radius=4)
        if len(self.data) < 2:
            return
        vals = list(self.data)
        vmin = lo if lo is not None else min(vals)
        vmax = hi if hi is not None else max(vals)
        if vmax - vmin < 1e-6:
            vmax = vmin + 1.0
        n = len(vals)
        # inset the plot area by 1px so the line's stroke width never draws
        # over the box's own border, then clip to it as a hard guarantee
        plot_rect = rect.inflate(-2, -2)
        pts = []
        for i, v in enumerate(vals):
            x = plot_rect.x + i / (n - 1) * plot_rect.width
            t = (v - vmin) / (vmax - vmin)
            t = max(0.0, min(1.0, t))
            y = plot_rect.bottom - t * plot_rect.height
            pts.append((x, y))
        prev_clip = surface.get_clip()
        surface.set_clip(plot_rect)
        if len(pts) >= 2:
            pygame.draw.lines(surface, color, False, pts, 2)
        surface.set_clip(prev_clip)
        cur = vals[-1]
        draw_text(surface, f"{label} {cur:.1f}{unit}", (rect.x + 6, rect.y + 4), font, TEXT_DIM)


# --------------------------------------------------------------------------
# Panels
# --------------------------------------------------------------------------

MOTOR_LAYOUT = {
    # index -> (label, grid position (col,row))  matches README GPIO table
    0: ("M1 FL", (0, 0), "CW"),
    1: ("M2 RT", (0, 1), "CW"),
    2: ("M3 RB", (1, 1), "CCW"),
    3: ("M4 FR", (1, 0), "CCW"),
}


def draw_motor_panel(surface, rect, frame, font, font_small):
    inner = rect
    cx, cy = inner.centerx, inner.centery
    size = min(inner.width, inner.height) * 0.42

    # blimp body outline (top-down view)
    body_w, body_h = size * 1.9, size * 1.15
    body_rect = pygame.Rect(0, 0, body_w, body_h)
    body_rect.center = (cx, cy)
    pygame.draw.ellipse(surface, (40, 46, 56), body_rect)
    pygame.draw.ellipse(surface, PANEL_BORDER, body_rect, width=2)
    # nose direction arrow (forward = up)
    pygame.draw.polygon(
        surface, TEXT_DIM,
        [(cx, cy - body_h / 2 - 14), (cx - 8, cy - body_h / 2 + 2), (cx + 8, cy - body_h / 2 + 2)]
    )

    max_pwm = max(255, max(abs(m) for m in frame.motors_actual) if frame.motors_actual else 255)
    max_pwm = max(max_pwm, 60)

    for idx, (label, (col, row), spin) in MOTOR_LAYOUT.items():
        mx = cx + (col * 2 - 1) * size * 0.62
        my = cy + (row * 2 - 1) * size * 0.5
        val = frame.motors_actual[idx] if idx < len(frame.motors_actual) else 0
        cmd = frame.cmd_motors[idx] if idx < len(frame.cmd_motors) else 0
        t = min(1.0, abs(val) / max_pwm)
        radius = 16 + t * 22
        color = MOTOR_COLOR if val >= 0 else RED
        pygame.draw.circle(surface, (0, 0, 0, 0), (int(mx), int(my)), int(radius) + 4)
        pygame.draw.circle(surface, color, (int(mx), int(my)), int(radius), width=0)
        pygame.draw.circle(surface, TEXT, (int(mx), int(my)), int(radius), width=2)
        # spin direction indicator
        spin_txt = font_small.render(spin, True, (10, 12, 16))
        surface.blit(spin_txt, spin_txt.get_rect(center=(mx, my)))
        draw_text(surface, label, (mx, my + radius + 6), font_small, TEXT_DIM, align="center")
        draw_text(surface, f"act {val}", (mx, my + radius + 20), font_small, TEXT, align="center")
        draw_text(surface, f"cmd {cmd}", (mx, my + radius + 34), font_small, TEXT_DIM, align="center")


def draw_imu_panel(surface, rect, frame, font, font_small):
    cx, cy = rect.centerx, rect.centery - 6
    radius = min(rect.width, rect.height) * 0.36

    # roll/pitch style horizon using AX (roll-ish) / AY (pitch-ish)
    roll = math.atan2(frame.ax, 9.8) if abs(frame.ax) < 20 else 0
    pitch = math.atan2(frame.ay, 9.8) if abs(frame.ay) < 20 else 0

    pygame.draw.circle(surface, (17, 20, 26), (cx, cy), radius)
    clip_rect = pygame.Rect(cx - radius, cy - radius, radius * 2, radius * 2)
    prev_clip = surface.get_clip()
    surface.set_clip(clip_rect)

    horizon_y = cy + pitch * radius * 1.4
    sky_poly = [(cx - radius * 2, horizon_y - radius * 3)]
    ground_poly_h = radius * 3

    surf_size = int(radius * 2.2)
    tmp = pygame.Surface((surf_size * 2, surf_size * 2), pygame.SRCALPHA)
    tsky = (52, 92, 140)
    tground = (76, 58, 40)
    tmp.fill(tsky)
    pygame.draw.rect(tmp, tground, (0, surf_size, surf_size * 2, surf_size))
    rotated = pygame.transform.rotate(tmp, -math.degrees(roll))
    rrect = rotated.get_rect(center=(cx, horizon_y))
    surface.blit(rotated, rrect)

    surface.set_clip(prev_clip)
    pygame.draw.circle(surface, PANEL_BORDER, (cx, cy), radius, width=2)
    pygame.draw.line(surface, TEXT, (cx - 18, cy), (cx + 18, cy), 2)
    pygame.draw.line(surface, TEXT, (cx, cy - 6), (cx, cy + 6), 2)

    # yaw torque (tz, rad) as a small needle dial below
    dial_cy = cy + radius + 34
    dial_r = 22
    pygame.draw.circle(surface, (17, 20, 26), (cx, dial_cy), dial_r)
    pygame.draw.circle(surface, PANEL_BORDER, (cx, dial_cy), dial_r, width=1)
    tz_clamped = max(-math.pi, min(math.pi, frame.tz))
    ang = tz_clamped - math.pi / 2
    nx = cx + math.cos(ang) * dial_r * 0.85
    ny = dial_cy + math.sin(ang) * dial_r * 0.85
    pygame.draw.line(surface, YELLOW, (cx, dial_cy), (nx, ny), 3)
    draw_text(surface, "yaw torque", (cx, dial_cy + dial_r + 4), font_small, TEXT_DIM, align="center")

    draw_text(surface, f"AX {frame.ax:+.1f}", (rect.x, rect.bottom - 44), font_small, TEXT)
    draw_text(surface, f"AY {frame.ay:+.1f}", (rect.x, rect.bottom - 30), font_small, TEXT)
    draw_text(surface, f"AZ {frame.az:+.1f}", (rect.x, rect.bottom - 16), font_small, TEXT)
    draw_text(surface, f"TZ {frame.tz:+.2f} rad", (rect.right, rect.bottom - 16), font_small, TEXT, align="right")


CAM_W, CAM_H = 320, 240  # assumed sensor working resolution for scaling the blob box


def draw_vision_panel(surface, rect, frame, font, font_small):
    pygame.draw.rect(surface, (10, 12, 16), rect)
    pygame.draw.rect(surface, PANEL_BORDER, rect, width=1)

    sx = rect.width / CAM_W
    sy = rect.height / CAM_H
    scale = min(sx, sy)
    ox = rect.x + (rect.width - CAM_W * scale) / 2
    oy = rect.y + (rect.height - CAM_H * scale) / 2

    # crosshair + deadzone (2 deg yaw / pitch deadzone visualized as a center box)
    ccx, ccy = ox + CAM_W / 2 * scale, oy + CAM_H / 2 * scale
    pygame.draw.line(surface, GRID_COLOR, (ccx, oy), (ccx, oy + CAM_H * scale), 1)
    pygame.draw.line(surface, GRID_COLOR, (ox, ccy), (ox + CAM_W * scale, ccy), 1)
    # StateMachine.h: YAW/PITCH_DEADZONE_HALF_DEG = 1deg each way (tiny; drawn
    # at a small fixed size since exact deg->pixel mapping needs horizontal FOV,
    # which isn't published alongside VERTICAL_FOV_DEG in the README)
    dz_w, dz_h = 0.04 * CAM_W * scale, 0.04 * CAM_H * scale
    pygame.draw.rect(surface, (60, 66, 40), (ccx - dz_w / 2, ccy - dz_h / 2, dz_w, dz_h), width=1)

    has_target = frame.w > 0 and frame.h > 0
    if has_target:
        bx = ox + max(0, frame.cx - frame.w / 2) * scale
        by = oy + max(0, frame.cy - frame.h / 2) * scale
        bw = frame.w * scale
        bh = frame.h * scale
        box_color = GREEN if frame.state.upper() in ("STATE_TRACKING", "TRACKING") else ORANGE
        pygame.draw.rect(surface, box_color, (bx, by, bw, bh), width=2)
        blob_cx = ox + frame.cx * scale
        blob_cy = oy + frame.cy * scale
        pygame.draw.circle(surface, box_color, (int(blob_cx), int(blob_cy)), 3)
        # line from center to blob showing yaw error direction
        pygame.draw.line(surface, box_color, (ccx, ccy), (blob_cx, blob_cy), 1)
    else:
        draw_text(surface, "NO TARGET", (rect.centerx, rect.centery - 8), font, TEXT_DIM, align="center")

    draw_text(surface, f"cx {frame.cx} cy {frame.cy}", (rect.x + 6, rect.bottom - 46), font_small, TEXT)
    draw_text(surface, f"w {frame.w} h {frame.h}", (rect.x + 6, rect.bottom - 32), font_small, TEXT)
    draw_text(surface, f"pixels {frame.pixels}", (rect.x + 6, rect.bottom - 18), font_small, TEXT)

    # pixel-area bar vs TURNING_AREA threshold (StateMachineConfig::TURNING_AREA = 29500)
    bar_x, bar_y, bar_w, bar_h = rect.right - 130, rect.bottom - 26, 118, 14
    turning_area = 29500
    frac = min(1.0, frame.pixels / turning_area)
    pygame.draw.rect(surface, (30, 34, 41), (bar_x, bar_y, bar_w, bar_h), border_radius=3)
    fill_color = ORANGE if frac >= 1.0 else ACCENT
    pygame.draw.rect(surface, fill_color, (bar_x, bar_y, bar_w * frac, bar_h), border_radius=3)
    pygame.draw.rect(surface, PANEL_BORDER, (bar_x, bar_y, bar_w, bar_h), width=1, border_radius=3)
    draw_text(surface, "area / turn thresh", (bar_x, bar_y - 14), font_small, TEXT_DIM)


def draw_yaw_gauge(surface, rect, frame, font, font_small):
    cx = rect.centerx
    cy = rect.y + rect.height * 0.62
    radius = min(rect.width * 0.48, rect.height * 0.55)

    pygame.draw.arc(surface, GRID_COLOR, (cx - radius, cy - radius, radius * 2, radius * 2),
                     math.pi, 2 * math.pi, 3)
    for deg in range(-90, 91, 30):
        a = math.pi + math.radians(deg + 90)
        x1 = cx + math.cos(a) * radius
        y1 = cy + math.sin(a) * radius
        x2 = cx + math.cos(a) * (radius - 8)
        y2 = cy + math.sin(a) * (radius - 8)
        pygame.draw.line(surface, TEXT_DIM, (x1, y1), (x2, y2), 2)

    err = max(-90, min(90, frame.yaw_error))
    a = math.pi + math.radians(err + 90)
    nx = cx + math.cos(a) * (radius - 4)
    ny = cy + math.sin(a) * (radius - 4)
    color = GREEN if abs(err) <= 2 else (YELLOW if abs(err) <= 15 else RED)
    pygame.draw.line(surface, color, (cx, cy), (nx, ny), 3)
    pygame.draw.circle(surface, TEXT, (cx, cy), 4)

    draw_text(surface, f"Yaw Error {frame.yaw_error:+.1f}\u00b0", (cx, cy - radius - 18), font, color, align="center")
    draw_text(surface, "L", (cx - radius - 4, cy - 6), font_small, TEXT_DIM, align="right")
    draw_text(surface, "R", (cx + radius + 4, cy - 6), font_small, TEXT_DIM)


def draw_state_panel(surface, rect, frame, font, font_small):
    states = ["MANUAL", "STATE_SEARCHING", "STATE_TRACKING", "STATE_TURNING"]
    display = {
        "MANUAL": "MANUAL",
        "STATE_SEARCHING": "SEARCHING",
        "STATE_TRACKING": "TRACKING",
        "STATE_TURNING": "TURNING",
    }
    cur = frame.state.upper()
    # normalize possible short forms
    alias = {"SEARCHING": "STATE_SEARCHING", "TRACKING": "STATE_TRACKING", "TURNING": "STATE_TURNING"}
    cur = alias.get(cur, cur)

    n = len(states)
    box_w = rect.width / n - 8
    box_h = min(rect.height * 0.78, rect.height - 26)
    y = rect.y + 4
    for i, s in enumerate(states):
        x = rect.x + i * (box_w + 8)
        active = (s == cur)
        color = STATE_COLORS.get(s, ACCENT) if active else (30, 34, 41)
        border = STATE_COLORS.get(s, ACCENT)
        r = pygame.Rect(x, y, box_w, box_h)
        pygame.draw.rect(surface, color, r, border_radius=6)
        pygame.draw.rect(surface, border, r, width=2, border_radius=6)
        txt_color = (10, 12, 16) if active else TEXT_DIM
        draw_text(surface, display[s], (r.centerx, r.y + 6), font_small, txt_color, align="center")

    mode_color = GREEN if frame.cmd_mode.upper() == "AUTONOMOUS" else ACCENT
    draw_text(surface, f"CMD MODE: {frame.cmd_mode}", (rect.x, y + box_h + 10), font, mode_color)


LOW_BATT_THRESHOLD = 3.3  # BatteryMonitor.h default setLowBatteryThreshold() (1S LiPo storage-safe level)


def draw_battery_panel(surface, rect, frame, spark, font, font_small):
    bar_rect = pygame.Rect(rect.x, rect.y, 26, rect.height - 46)
    pygame.draw.rect(surface, (30, 34, 41), bar_rect, border_radius=4)
    vmin, vmax = 3.0, 4.2
    frac = max(0.0, min(1.0, (frame.batt - vmin) / (vmax - vmin)))
    fill_h = bar_rect.height * frac
    fill_rect = pygame.Rect(bar_rect.x, bar_rect.bottom - fill_h, bar_rect.width, fill_h)
    color = batt_color(frame.batt)
    pygame.draw.rect(surface, color, fill_rect, border_radius=4)

    # low-battery cutoff tick — kept within the bar's width, not overhanging it
    low_frac = max(0.0, min(1.0, (LOW_BATT_THRESHOLD - vmin) / (vmax - vmin)))
    low_y = bar_rect.bottom - bar_rect.height * low_frac
    pygame.draw.line(surface, RED, (bar_rect.x, low_y), (bar_rect.right, low_y), 2)

    pygame.draw.rect(surface, PANEL_BORDER, bar_rect, width=1, border_radius=4)

    label_color = RED if frame.batt <= LOW_BATT_THRESHOLD else color
    draw_text(surface, f"{frame.batt:.2f}V", (bar_rect.right + 10, rect.y), font, label_color)
    if frame.batt <= LOW_BATT_THRESHOLD:
        draw_text(surface, "LOW BATT", (bar_rect.right + 10, rect.y + 18), font_small, RED)

    spark_rect = pygame.Rect(bar_rect.right + 10, rect.y + 20, rect.width - bar_rect.width - 16,
                              rect.height - 20)
    spark.draw(surface, spark_rect, color, font_small, label="batt", lo=3.0, hi=4.2, unit="V")


def draw_latency_panel(surface, rect, frame, spark_rate, spark_delta, font, font_small):
    top = pygame.Rect(rect.x, rect.y, rect.width, rect.height // 2 - 4)
    bot = pygame.Rect(rect.x, rect.y + rect.height // 2, rect.width, rect.height // 2 - 4)
    spark_rate.draw(surface, top, ACCENT, font_small, label="FPS", lo=0)
    spark_delta.draw(surface, bot, YELLOW, font_small, label="delta", unit="ms", lo=0)
    draw_text(surface, f"{frame.rate_fps:.1f} FPS  avg {frame.lat_avg:.1f}ms  q {frame.queue}B",
              (rect.x, rect.bottom - 2), font_small, TEXT_DIM)


# --------------------------------------------------------------------------
# Main application
# --------------------------------------------------------------------------

class Dashboard:
    def __init__(self, frames=None, live_source=None, width=1180, height=760):
        pygame.init()
        pygame.display.set_caption("ESP-FLY Blimp Ground Control")
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        self.font_big = pygame.font.SysFont("consolas,menlo,monospace", 22, bold=True)
        self.font = pygame.font.SysFont("consolas,menlo,monospace", 16)
        self.font_small = pygame.font.SysFont("consolas,menlo,monospace", 13)
        self.font_title = pygame.font.SysFont("consolas,menlo,monospace", 13, bold=True)

        self.frames = frames or []
        self.live_source = live_source  # generator yielding Frame objects, or None
        self.assembler = LiveLineAssembler() if live_source else None

        self.idx = 0
        self.playing = self.live_source is None  # live mode is always "playing"
        self.speed = 1.0
        self.last_advance = 0.0
        self.batt_spark = Sparkline()
        self.rate_spark = Sparkline()
        self.delta_spark = Sparkline()
        self.yaw_spark = Sparkline()

        # seed sparklines with history if replaying so charts aren't empty at start
        self.running = True

    def current_frame(self):
        if self.frames:
            return self.frames[min(self.idx, len(self.frames) - 1)]
        return None

    def advance_replay(self, dt):
        if not self.frames or not self.playing:
            return
        # advance roughly self.speed frames worth of "log time" per real second,
        # simplified to a fixed nominal rate for smooth scrubbing feel
        nominal_hz = 40.0
        self.last_advance += dt
        step_interval = 1.0 / (nominal_hz * self.speed) if self.speed > 0 else 999
        while self.last_advance >= step_interval and self.idx < len(self.frames) - 1:
            self.idx += 1
            self.last_advance -= step_interval
            self._push_sparklines(self.frames[self.idx])
        if self.idx >= len(self.frames) - 1:
            self.playing = False

    def _push_sparklines(self, frame):
        self.batt_spark.push(frame.batt)
        self.rate_spark.push(frame.rate_fps)
        self.delta_spark.push(frame.lat_delta)
        self.yaw_spark.push(frame.yaw_error)

    def poll_live(self):
        if not self.live_source:
            return
        try:
            for _ in range(200):  # drain a bounded batch per frame tick
                line = next(self.live_source)
                if line is None:
                    break
                done = self.assembler.feed_line(line)
                if done is not None:
                    self.frames.append(done)
                    self.idx = len(self.frames) - 1
                    self._push_sparklines(done)
        except StopIteration:
            pass

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                elif event.key == pygame.K_SPACE and self.frames and not self.live_source:
                    self.playing = not self.playing
                elif event.key == pygame.K_LEFT and self.frames:
                    self.idx = max(0, self.idx - 1)
                    self.playing = False
                elif event.key == pygame.K_RIGHT and self.frames:
                    self.idx = min(len(self.frames) - 1, self.idx + 1)
                    self.playing = False
                elif event.key == pygame.K_UP:
                    self.speed = min(16.0, self.speed * 2)
                elif event.key == pygame.K_DOWN:
                    self.speed = max(0.125, self.speed / 2)
                elif event.key == pygame.K_r and self.frames:
                    self.idx = 0
                    self.playing = True
            elif event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            elif event.type == pygame.MOUSEBUTTONDOWN and self.frames and not self.live_source:
                if hasattr(self, "scrub_rect") and self.scrub_rect.collidepoint(event.pos):
                    frac = (event.pos[0] - self.scrub_rect.x) / self.scrub_rect.width
                    self.idx = max(0, min(len(self.frames) - 1, int(frac * len(self.frames))))
                    self.playing = False

    def draw_scrubber(self, rect, frame):
        self.scrub_rect = rect
        pygame.draw.rect(self.screen, (30, 34, 41), rect, border_radius=4)
        if self.frames:
            frac = self.idx / max(1, len(self.frames) - 1)
            x = rect.x + frac * rect.width
            pygame.draw.rect(self.screen, ACCENT, (rect.x, rect.y, x - rect.x, rect.height), border_radius=4)
            pygame.draw.circle(self.screen, TEXT, (int(x), rect.centery), 7)
        pygame.draw.rect(self.screen, PANEL_BORDER, rect, width=1, border_radius=4)

    def draw_header(self, rect, frame):
        title = "ESP-FLY BLIMP GROUND CONTROL"
        draw_text(self.screen, title, (rect.x, rect.y), self.font_big, ACCENT)
        ts = frame.timestamp or "--"
        draw_text(self.screen, ts, (rect.right, rect.y + 2), self.font, TEXT_DIM, align="right")

        if self.live_source:
            status = "LIVE"
            status_color = RED
        else:
            status = "PLAY" if self.playing else "PAUSE"
            status_color = GREEN if self.playing else YELLOW
        info = f"{status}   speed x{self.speed:g}   frame {self.idx + 1}/{max(1, len(self.frames))}"
        draw_text(self.screen, info, (rect.x, rect.y + 26), self.font_small, status_color)

        help_txt = "SPACE play/pause  \u2190/\u2192 step  \u2191/\u2193 speed  R restart  ESC quit"
        draw_text(self.screen, help_txt, (rect.right, rect.y + 26), self.font_small, TEXT_DIM, align="right")

    def layout(self, w, h):
        margin = 14
        header_h = 50
        scrub_h = 18 if not self.live_source else 0
        top = margin + header_h + 8

        content_h = h - top - margin - (scrub_h + 8 if scrub_h else 0)
        left_w = int(w * 0.42)
        right_w = w - left_w - margin * 3

        col1_x = margin
        col2_x = margin * 2 + left_w

        rects = {}
        rects["header"] = pygame.Rect(margin, margin, w - margin * 2, header_h)

        # left column: motors (top) + IMU/yaw (bottom row)
        motor_h = int(content_h * 0.52)
        rects["motors"] = pygame.Rect(col1_x, top, left_w, motor_h)
        sub_w = left_w // 2 - 6
        rects["imu"] = pygame.Rect(col1_x, top + motor_h + 12, sub_w, content_h - motor_h - 12)
        rects["yaw"] = pygame.Rect(col1_x + sub_w + 12, top + motor_h + 12, sub_w, content_h - motor_h - 12)

        # right column: vision (top), state (mid), battery+latency (bottom)
        vision_h = int(content_h * 0.42)
        state_h = 92
        remain_h = content_h - vision_h - state_h - 24
        rects["vision"] = pygame.Rect(col2_x, top, right_w, vision_h)
        rects["state"] = pygame.Rect(col2_x, top + vision_h + 12, right_w, state_h)
        batt_w = right_w // 2 - 6
        rects["battery"] = pygame.Rect(col2_x, top + vision_h + state_h + 24, batt_w, remain_h)
        rects["latency"] = pygame.Rect(col2_x + batt_w + 12, top + vision_h + state_h + 24, batt_w, remain_h)

        if scrub_h:
            rects["scrub"] = pygame.Rect(margin, h - margin - scrub_h, w - margin * 2, scrub_h)
        return rects

    def draw(self):
        w, h = self.screen.get_size()
        self.screen.fill(BG)
        frame = self.current_frame()
        if frame is None:
            draw_text(self.screen, "waiting for telemetry...", (w // 2, h // 2), self.font_big,
                      TEXT_DIM, align="center")
            pygame.display.flip()
            return

        rects = self.layout(w, h)
        self.draw_header(rects["header"], frame)

        inner = panel(self.screen, rects["motors"], "MOTORS (top-down)", self.font_title)
        draw_motor_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = panel(self.screen, rects["imu"], "IMU / ATTITUDE", self.font_title)
        draw_imu_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = panel(self.screen, rects["yaw"], "YAW ERROR", self.font_title)
        draw_yaw_gauge(self.screen, inner, frame, self.font, self.font_small)

        inner = panel(self.screen, rects["vision"], "VISION / TARGET TRACKING", self.font_title)
        draw_vision_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = panel(self.screen, rects["state"], "STATE MACHINE", self.font_title)
        draw_state_panel(self.screen, inner, frame, self.font, self.font_small)

        inner = panel(self.screen, rects["battery"], "BATTERY", self.font_title)
        draw_battery_panel(self.screen, inner, frame, self.batt_spark, self.font, self.font_small)

        inner = panel(self.screen, rects["latency"], "LINK / LATENCY", self.font_title)
        draw_latency_panel(self.screen, inner, frame, self.rate_spark, self.delta_spark, self.font, self.font_small)

        if "scrub" in rects:
            self.draw_scrubber(rects["scrub"], frame)

        pygame.display.flip()

    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            self.handle_events()
            if self.live_source:
                self.poll_live()
            else:
                self.advance_replay(dt)
            self.draw()
        pygame.quit()


# --------------------------------------------------------------------------
# Live source helpers
# --------------------------------------------------------------------------

def tail_file_lines(path):
    """Generator that yields new lines appended to a growing log file, or
    None when there's nothing new yet (non-blocking-ish)."""
    f = open(path, "r", errors="replace")
    f.seek(0, 2)  # seek to end
    while True:
        line = f.readline()
        if line:
            yield line
        else:
            yield None
            time.sleep(0.01)


def serial_lines(port, baud=115200):
    import serial  # requires: pip install pyserial
    ser = serial.Serial(port, baud, timeout=0.05)
    while True:
        raw = ser.readline()
        if raw:
            yield raw.decode(errors="replace")
        else:
            yield None


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="ESP-FLY blimp telemetry dashboard")
    ap.add_argument("logfile", nargs="?", help="path to a flight.log file to replay")
    ap.add_argument("--tail", metavar="LOGFILE", help="tail a live-growing log file")
    ap.add_argument("--live", metavar="PORT", help="read live telemetry from a serial port")
    ap.add_argument("--baud", type=int, default=115200, help="serial baud rate (default 115200)")
    args = ap.parse_args()

    if args.live:
        dash = Dashboard(live_source=serial_lines(args.live, args.baud))
    elif args.tail:
        dash = Dashboard(live_source=tail_file_lines(args.tail))
    elif args.logfile:
        frames = parse_log(args.logfile)
        if not frames:
            print(f"No telemetry frames could be parsed from {args.logfile}")
            sys.exit(1)
        print(f"Loaded {len(frames)} frames from {args.logfile}")
        dash = Dashboard(frames=frames)
        # preload all sparkline history up to frame 0 for a clean start
        dash._push_sparklines(frames[0])
    else:
        ap.print_help()
        sys.exit(1)

    dash.run()


if __name__ == "__main__":
    main()