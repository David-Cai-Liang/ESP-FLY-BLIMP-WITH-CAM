import serial
import cv2
import numpy as np
import struct
import datetime

# Update to match your Xiao COM port:
# Windows: 'COM3', 'COM4', etc. | Mac/Linux: '/dev/ttyACM0'
SERIAL_PORT = 'COM35'
BAUD_RATE = 115200

MAGIC_HEADER = b'\xff\xaa\x55\xff'
HEADER_SIZE = 16  # 4 magic + 4 payload_len + 8 (6 x uint16, 1 x uint32)

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Xiao ESP32S3 on {SERIAL_PORT}.")
        print("Press 'q' on the video window to exit.\n")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    buffer = bytearray()
    last_print_time = 0

    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buffer += data

        while len(buffer) >= HEADER_SIZE:
            idx = buffer.find(MAGIC_HEADER)
            if idx == -1:
                buffer.clear()
                break
            if idx > 0:
                buffer = buffer[idx:]

            if len(buffer) < HEADER_SIZE:
                break

            # Unpack binary frame header & telemetry array
            unpacked = struct.unpack('<4sI11H', buffer[:HEADER_SIZE])
            payload_len = unpacked[1]
            blob_array = list(unpacked[2:]) # [x, y, cx, cy, w, h, pixels]

            if len(buffer) < HEADER_SIZE + payload_len:
                break  # Wait for complete JPEG payload

            if payload_len == 0:
                print(f"{datetime.datetime.now()}: {blob_array}")
                buffer = buffer[HEADER_SIZE:]
                continue

            jpg_data = buffer[HEADER_SIZE : HEADER_SIZE + payload_len]
            buffer = buffer[HEADER_SIZE + payload_len:]

            # Decode JPEG image
            np_arr = np.frombuffer(jpg_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                print(f"{datetime.datetime.now()}: {blob_array}")

                # Display frame with ROI overlays drawn in real-time
                cv2.imshow("Balloon Race USB Stream", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    ser.close()
                    cv2.destroyAllWindows()
                    return

if __name__ == '__main__':
    main()
