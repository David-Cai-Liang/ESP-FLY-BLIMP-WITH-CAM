import serial
import cv2
import numpy as np
import struct
import pygame
import sys

SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

MAGIC_HEADER = b'\xff\xaa\x55\xff'
HEADER_SIZE = 8

l_min, l_max = 20, 60
a_min, a_max = 10, 50
b_min, b_max = 1, 30

def print_thresholds():
    print(f"LAB Thresholds -> L:[{l_min}, {l_max}] A:[{a_min}, {a_max}] B:[{b_min}, {b_max}]")
    print(f"C++ Struct Copy: {{{l_min}, {l_max}, {a_min}, {a_max}, {b_min}, {b_max}}}\n")

def apply_color_mask(img):
    l_min_cv = int(np.clip(l_min * 255 / 100, 0, 255))
    l_max_cv = int(np.clip(l_max * 255 / 100, 0, 255))
    a_min_cv = int(np.clip(a_min + 128, 0, 255))
    a_max_cv = int(np.clip(a_max + 128, 0, 255))
    b_min_cv = int(np.clip(b_min + 128, 0, 255))
    b_max_cv = int(np.clip(b_max + 128, 0, 255))

    lower_bound = np.array([l_min_cv, a_min_cv, b_min_cv], dtype=np.uint8)
    upper_bound = np.array([l_max_cv, a_max_cv, b_max_cv], dtype=np.uint8)

    lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(lab_img, lower_bound, upper_bound)
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

def main():
    global l_min, l_max, a_min, a_max, b_min, b_max

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to ESP32 on {SERIAL_PORT}.")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    pygame.init()
    screen = pygame.display.set_mode((320, 240))
    pygame.display.set_caption("ESP32 LAB Mask Tuner (JPEG Mode)")
    font = pygame.font.SysFont("Consolas", 14)

    print_thresholds()
    buffer = bytearray()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                ser.close()
                pygame.quit()
                sys.exit()

            if event.type == pygame.KEYDOWN:
                changed = True
                if event.key == pygame.K_1: l_min = max(0, l_min - 1)
                elif event.key == pygame.K_q: l_min = min(l_max, l_min + 1)
                elif event.key == pygame.K_2: l_max = max(l_min, l_max - 1)
                elif event.key == pygame.K_w: l_max = min(100, l_max + 1)
                elif event.key == pygame.K_3: a_min = max(-128, a_min - 1)
                elif event.key == pygame.K_e: a_min = min(a_max, a_min + 1)
                elif event.key == pygame.K_4: a_max = max(a_min, a_max - 1)
                elif event.key == pygame.K_r: a_max = min(127, a_max + 1)
                elif event.key == pygame.K_5: b_min = max(-128, b_min - 1)
                elif event.key == pygame.K_t: b_min = min(b_max, b_min + 1)
                elif event.key == pygame.K_6: b_max = max(b_min, b_max - 1)
                elif event.key == pygame.K_y: b_max = min(127, b_max + 1)
                else: changed = False

                if changed:
                    print_thresholds()

        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting)

        while len(buffer) >= HEADER_SIZE:
            idx = buffer.find(MAGIC_HEADER)
            if idx == -1:
                buffer = buffer[-3:]
                break

            if idx > 0:
                buffer = buffer[idx:]

            if len(buffer) < HEADER_SIZE:
                break

            _, payload_len = struct.unpack('<4sI', buffer[:HEADER_SIZE])

            if len(buffer) < HEADER_SIZE + payload_len:
                break

            jpg_data = buffer[HEADER_SIZE : HEADER_SIZE + payload_len]
            buffer = buffer[HEADER_SIZE + payload_len:]

            np_arr = np.frombuffer(jpg_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                rgb_mask = apply_color_mask(img)

                frame_surface = pygame.image.frombuffer(
                    rgb_mask.tobytes(), (rgb_mask.shape[1], rgb_mask.shape[0]), 'RGB'
                )

                screen.blit(frame_surface, (0, 0))

                hud_text = font.render(
                    f"L:[{l_min},{l_max}] A:[{a_min},{a_max}] B:[{b_min},{b_max}]",
                    True, (0, 255, 0)
                )
                screen.blit(hud_text, (5, 5))
                pygame.display.flip()
            else:
                print("No Image Found")

if __name__ == '__main__':
    main()
