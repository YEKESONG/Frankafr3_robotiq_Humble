#!/usr/bin/env python3
"""Raw Modbus RTU probe for a Robotiq 2F-85, bypassing ROS entirely.

Reads the 3 status input registers at 0x07D0 from slave 9, which is exactly what
robotiq_driver does. If this gets no reply either, the problem is below the
driver: power, wiring, baud or slave id -- not the ROS side.
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else \
    "/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAAL8Y3V-if00-port0"


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def probe(port, baud, slave):
    frame = bytes([slave, 0x04, 0x07, 0xD0, 0x00, 0x03])
    frame += crc16(frame)
    try:
        with serial.Serial(port, baud, timeout=0.5) as ser:
            ser.reset_input_buffer()
            ser.write(frame)
            ser.flush()
            time.sleep(0.1)
            reply = ser.read(11)
    except Exception as exc:  # noqa: BLE001
        return f"open/IO error: {exc}"
    if not reply:
        return "no reply"
    return f"reply ({len(reply)} bytes): {reply.hex(' ')}"


def main():
    print(f"port: {PORT}\n")
    # 115200 is the Robotiq factory default; the others are what a
    # previously-reconfigured gripper might be sitting at.
    for baud in (115200, 19200, 57600, 38400, 9600):
        for slave in (9, 1):
            print(f"  baud={baud:<7} slave={slave}: {probe(PORT, baud, slave)}")


if __name__ == "__main__":
    main()
