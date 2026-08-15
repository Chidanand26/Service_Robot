# AMR ServiceBot ROS 2 Mapping & Control Guide

Complete guide for running **amr_assembly_description** in both **Simulation/Offline Demo Mode** and on **Physical Robot Hardware** (ESP32 Wi-Fi + RPLiDAR + CS-D508 Steppers).

---

## 1. System Architecture & Folder Flow

```text
+-----------------------------------------------------------------------------------+
|                                 ROS 2 HOST / RASPBERRY PI                         |
|                                                                                   |
|  [teleop_twist_keyboard]                                                          |
|            | (/cmd_vel)                                                           |
|            v                                                                      |
|  [twist_to_wheel_cmd.py] ---> (/cmd_pos)                                          |
|                                    |                                              |
|                                    v (UDP Wi-Fi Port 8888)                        |
|                         [esp32_wifi_driver_node.py] <====== (Wi-Fi 50Hz) =====+   |
|                                    |                                          |   |
|                                    v (/joint_states)                          |   |
|  [robot_state_publisher] <---------+                                          |   |
|            |                                                                  |   |
|            v (TF Tree: base_footprint -> base_link -> laser_frame)            |   |
|            |                                                                  |   |
|  [sllidar_node] (/dev/ttyUSB0) ---> (/scan)                                   |   |
|            |                           |                                      |   |
|            +-----------------------+   |                                      |   |
|                                    |   |                                      |   |
|                                    v   v                                      |   |
|                             [slam_toolbox] ---> (/map)                        |   |
|                                    |                                          |   |
|                                    v                                          |   |
|                                 [RViz2]                                       |   |
+-------------------------------------------------------------------------------|---+
                                                                                |
                                     +------------------------------------------+
                                     |
                                     v
                  +--------------------------------------+
                  |           ESP32 CONTROLLER           |
                  |                                      |
                  |  - Wi-Fi UDP Server (Port 8888)      |
                  |  - Non-blocking Trapezoidal Stepping |
                  |  - Left Drive (CS-D508 Motor 1)      |
                  |  - Right Drive (CS-D508 Motor 2)     |
                  |  - Encoder Position Telemetry (50Hz) |
                  +--------------------------------------+
```

---

## 2. Directory Structure & Key Files

| File Path | Description |
|---|---|
| [`urdf/amr_assembly_description.urdf`](file:///home/ar08/ros2_ws/src/amr_assembly_description/urdf/amr_assembly_description.urdf) | Clean URDF model (REP-103 standard frames, wheels, LiDAR, camera, IMU). |
| [`launch/mapping.launch.py`](file:///home/ar08/ros2_ws/src/amr_assembly_description/launch/mapping.launch.py) | Master launch file for SLAM mapping (supports Simulation and Hardware modes). |
| [`scripts/esp32_wifi_driver_node.py`](file:///home/ar08/ros2_ws/src/amr_assembly_description/scripts/esp32_wifi_driver_node.py) | Wi-Fi UDP bridge between ESP32 and ROS 2 topics (`/joint_states`, `/cmd_pos`). |
| [`scripts/twist_to_wheel_cmd.py`](file:///home/ar08/ros2_ws/src/amr_assembly_description/scripts/twist_to_wheel_cmd.py) | Converts `/cmd_vel` velocity commands into wheel revolution commands. |
| [`scripts/dummy_laser_scan_publisher.py`](file:///home/ar08/ros2_ws/src/amr_assembly_description/scripts/dummy_laser_scan_publisher.py) | Generates simulated laser scans for testing when physical LiDAR is offline. |
| [`scripts/dummy_joint_state_publisher.py`](file:///home/ar08/ros2_ws/src/amr_assembly_description/scripts/dummy_joint_state_publisher.py) | Publishes default wheel joint states for offline visualization. |
| [`rviz/amr_assembly_description.rviz`](file:///home/ar08/ros2_ws/src/amr_assembly_description/rviz/amr_assembly_description.rviz) | Pre-configured RViz display (Map, RobotModel, LaserScan, TF, Grid). |

---

## 3. Mode A: Simulation / Offline Demo Mode (No Hardware Needed)

Run this command to test SLAM mapping, RViz visualization, and teleoperation immediately on your computer:

```bash
# Terminal 1: Launch Simulation Mapping
source /opt/ros/jazzy/setup.bash
source /home/ar08/ros2_ws/install/setup.bash

ros2 launch amr_assembly_description mapping.launch.py use_dummy_scan:=true
```

```bash
# Terminal 2: Drive the Simulated Robot with Keyboard
source /opt/ros/jazzy/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use `i` (forward), `j` (turn left), `l` (turn right), `k` (stop), `,` (backward) to drive the robot around!

---

## 4. Mode B: Physical Robot Hardware Mode

### Step 1: ESP32 Pinout & Wiring

| ESP32 Pin | Signal | Target CS-D508 Drive |
|---|---|---|
| **GPIO 25** | PUL- (Step) | Motor 1 (Left Wheel Drive) |
| **GPIO 26** | DIR- (Direction) | Motor 1 (Left Wheel Drive) |
| **GPIO 27** | ALM (Alarm, optional) | Motor 1 (Left Wheel Drive) |
| **GPIO 32** | PUL- (Step) | Motor 2 (Right Wheel Drive) |
| **GPIO 33** | DIR- (Direction) | Motor 2 (Right Wheel Drive) |
| **GPIO 13** | ALM (Alarm, optional) | Motor 2 (Right Wheel Drive) |
| **GND** | Ground | Common DC Ground with Drives |

### Step 2: Flash ESP32 Firmware

Use the updated C++ sketch in Section 6. Update your Wi-Fi SSID, Password, and Pi IP (`10.78.37.133`).

### Step 3: Grant USB Port Permissions for RPLiDAR

```bash
sudo chmod 666 /dev/ttyUSB0
```

### Step 4: Launch Real Hardware SLAM Mapping

```bash
# Terminal 1: Launch Hardware Mapping
source /opt/ros/jazzy/setup.bash
source /home/ar08/ros2_ws/install/setup.bash

ros2 launch amr_assembly_description mapping.launch.py \
  use_wifi:=true \
  serial_port:=/dev/ttyUSB1 \
  serial_baudrate:=115200
```

### Step 5: Drive Robot in Real-Time with Keyboard

```bash
# Terminal 2: Teleop Control
source /opt/ros/jazzy/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 5. Saving Your Map

When your map is complete, save it to disk:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run nav2_map_server map_saver_cli -f ~/my_servicebot_map
```

This creates:
- `~/my_servicebot_map.yaml` (metadata & resolution)
- `~/my_servicebot_map.pgm` (2D occupancy grid image)

---

## 6. Complete ESP32 Firmware Sketch

```cpp
// ============================================================================
// ESP32 Wi-Fi (UDP) Firmware for Dual CS-D508 Closed-Loop Steppers
// ============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>

// --- Network Configuration ---
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* pi_ip    = "10.78.37.133";
const uint16_t udp_port = 8888;

// Set to true only if ALM pins (GPIO 27 & 13) are physically wired to drive ALM outputs
#define ENABLE_HARDWARE_ALARM false

WiFiUDP udp;
const long PPR_OUT = 800L * 5L; // 4000 Pulses per Wheel Revolution

struct Axis {
  uint8_t  pul, dir, alm;
  long     total = 0, left = 0;
  long     pos = 0;
  int8_t   sign = 1;
  uint32_t half = 0, last = 0;
  bool     active = false;
  uint16_t startUs = 400, minUs = 60;
  long     ramp = 800;
};

// Motor 1: Left Wheel | Motor 2: Right Wheel
Axis m1 { 25, 26, 27 };
Axis m2 { 32, 33, 13 };

void initAxis(Axis &a) {
  pinMode(a.pul, OUTPUT_OPEN_DRAIN);
  pinMode(a.dir, OUTPUT_OPEN_DRAIN);
  digitalWrite(a.pul, HIGH);
  digitalWrite(a.dir, HIGH);
  pinMode(a.alm, INPUT_PULLUP);
}

void startMove(Axis &a, float rev) {
  a.total = a.left = labs((long)(rev * PPR_OUT));
  a.sign  = (rev > 0) ? 1 : -1;
  digitalWrite(a.dir, rev > 0 ? LOW : HIGH);
  a.half  = a.startUs;
  a.last  = micros();
  a.active = false;
}

void stopAxis(Axis &a) {
  a.left = 0;
  digitalWrite(a.pul, HIGH);
  a.active = false;
}

uint32_t speedAt(Axis &a) {
  long done = a.total - a.left;
  long ramp = min(a.total / 2, a.ramp);
  if (ramp < 1) return a.minUs;
  if (done < ramp) return a.startUs - (long)(a.startUs - a.minUs) * done / ramp;
  if (a.left < ramp) return a.startUs - (long)(a.startUs - a.minUs) * a.left / ramp;
  return a.minUs;
}

void service(Axis &a) {
  if (a.left <= 0) return;
#if ENABLE_HARDWARE_ALARM
  if (digitalRead(a.alm) == LOW) { stopAxis(a); return; }
#endif
  uint32_t now = micros();
  if (now - a.last < a.half) return;
  a.last = now;
  a.active = !a.active;
  digitalWrite(a.pul, a.active ? LOW : HIGH);
  if (!a.active) {
    a.left--;
    a.pos += a.sign;
    a.half = speedAt(a);
  }
}

char buf[64];
uint32_t lastReport = 0;

void handleCommand(char *line) {
  switch (line[0]) {
    case 'M': {
      float r1 = 0, r2 = 0;
      if (sscanf(line + 1, "%f %f", &r1, &r2) == 2) {
        if (r1 != 0) startMove(m1, r1);
        if (r2 != 0) startMove(m2, r2);
      }
      break;
    }
    case 'S': stopAxis(m1); stopAxis(m2); break;
    case 'Z': m1.pos = m2.pos = 0; break;
  }
}

void sendWifiTelemetry() {
  char packetBuf[128];
  int alm1 = ENABLE_HARDWARE_ALARM ? (digitalRead(m1.alm) == LOW) : 0;
  int alm2 = ENABLE_HARDWARE_ALARM ? (digitalRead(m2.alm) == LOW) : 0;

  snprintf(packetBuf, sizeof(packetBuf), "P %.4f %.4f %d %d %d %d\n",
    (float)m1.pos / PPR_OUT, (float)m2.pos / PPR_OUT,
    m1.left > 0, m2.left > 0,
    alm1, alm2);

  udp.beginPacket(pi_ip, udp_port);
  udp.print(packetBuf);
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  initAxis(m1);
  initAxis(m2);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(udp_port);
}

void loop() {
  service(m1);
  service(m2);

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = 0;
      handleCommand(buf);
    }
  }

  uint32_t now = millis();
  if (now - lastReport >= 20) {
    lastReport = now;
    sendWifiTelemetry();
  }
}
```

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

sudo chmod 666 /dev/ttyUSB* /dev/input/js*

ros2 launch amr_assembly_description mapping.launch.py \
      use_esp32:=true \
      serial_port:=/dev/ttyUSB0 \
      esp_port:=/dev/ttyUSB1 \
      enable_camera:=true
 ### How You Can Diagnose & Fix It (1-Minute Cheat Sheet)

  If you ever encounter this in the future, follow these 3 quick checks:

  #### Step 1: Kill any hanging background processes

  If a node crashed or terminal was closed improperly, stale processes might
  still be holding the serial or video ports:

    pkill -9 -f "sllidar|slam_toolbox|realsense|esp32_driver"

  #### Step 2: Check device symlinks & permissions

  Verify that both devices are recognized:

    ls -la /dev/rplidar /dev/esp32

  If missing or permissions are restricted, run:

    sudo chmod 666 /dev/ttyUSB*
    sudo udevadm control --reload-rules && sudo udevadm trigger

  #### Step 3: Verify the TF Tree

  To verify if map is being broadcasted by SLAM:

    ros2 run tf2_ros tf2_echo map base_footprint

  If you see Translation and Rotation coordinates streaming, the TF tree and
  SLAM are working.

  #### Step 4: Start the System

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash

    ros2 launch amr_assembly_description mapping.launch.py use_esp32:=true


