#!/usr/bin/env python3
"""
Automatic Serial Port Detector for RPLiDAR and ESP32.
Probes all /dev/ttyUSB* ports by sending RPLiDAR health request bytes (\xa5\x52).
- Returns the port path that responds with RPLiDAR header (\xa5Z...) as RPLiDAR.
- Returns the other port (or ESP32 telemetry provider) as ESP32.
"""

import sys
import glob
import time
import serial


def detect_ports():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    lidar_port = None
    esp32_port = None

    for port in ports:
        try:
            # Open port with short timeout
            s = serial.Serial(port, 115200, timeout=0.8)
            time.sleep(0.1)

            # Send RPLiDAR GET_HEALTH command (\xa5\x52)
            s.write(b'\xa5\x52')
            resp = s.read(10)

            # RPLiDAR responds with magic bytes 0xA5 0x5A
            if len(resp) >= 2 and resp[0] == 0xA5 and resp[1] == 0x5A:
                lidar_port = port
            else:
                esp32_port = port

            s.close()
        except Exception:
            pass

    # Fallback heuristics if one port failed probe
    if lidar_port and not esp32_port:
        remaining = [p for p in ports if p != lidar_port]
        if remaining:
            esp32_port = remaining[0]
    elif esp32_port and not lidar_port:
        remaining = [p for p in ports if p != esp32_port]
        if remaining:
            lidar_port = remaining[0]

    return lidar_port, esp32_port


if __name__ == '__main__':
    lidar, esp = detect_ports()
    if len(sys.argv) > 1:
        target = sys.argv[1].lower()
        if target == 'lidar':
            print(lidar if lidar else '/dev/ttyUSB2')
        elif target == 'esp32':
            print(esp if esp else '/dev/ttyUSB1')
        else:
            print(f'LIDAR={lidar} ESP32={esp}')
    else:
        print(f'Detected RPLiDAR Port: {lidar}')
        print(f'Detected ESP32 Port:   {esp}')
