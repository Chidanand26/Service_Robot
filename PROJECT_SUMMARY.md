# AMR ServiceBot — Comprehensive Project Summary & Architecture Guide

## 1. Project Overview & Objective
The **AMR ServiceBot** is an Autonomous Mobile Robot (AMR) designed for indoor service delivery (e.g. food/beverage/tea transport) and interactive autonomous navigation. It runs on **ROS 2 Jazzy Jalisco** on an onboard **Raspberry Pi 5 / Desktop host** paired with a real-time **ESP32 microcontroller** for precision closed-loop motion control.

---

## 2. Hardware Architecture & Wiring

### A. Compute & Sensing Components
- **Host Compute:** Raspberry Pi 5 running Ubuntu 24.04 & ROS 2 Jazzy.
- **Microcontroller:** ESP32 NodeMCU (Dual-core Xtensa 240MHz).
- **2D LiDAR:** RPLiDAR A1 (360° Laser Scanner, 12m range, 10Hz, 8kHz sample rate).
- **3D Depth Camera:** Intel RealSense D435i (Depth + RGB + IMU over USB 3.0).
- **Teleoperation:** Sony PS5 DualSense Controller (Bluetooth HID `/dev/input/js0`).
- **Motor Actuation:** 2× CS-D508 Closed-Loop Stepper Drives paired with 57CME23 Stepper Motors + 5:1 Planetary Gearboxes.
- **Power System:** 24V LiFePO4 battery pack stepped down to 5V (compute) and direct 24V (motor drives).

### B. Pinout & Port Allocation Matrix

| Device | Linux Device Node | Permanent Udev Symlink | GPIO / Bus | Notes |
|---|---|---|---|---|
| **RPLiDAR A1** | `/dev/ttyUSB2` | `/dev/rplidar` | USB Host | 115200 baud, auto-baud & mode support |
| **ESP32 Motor Bridge** | `/dev/ttyUSB3` | `/dev/esp32` | USB Host | 115200 baud, full duplex serial |
| **Intel RealSense D435i** | `/dev/video0..5` | `/dev/camera` | USB 3.0 Blue Port | 640x480 @ 15fps pointcloud & depth |
| **Left Drive (CS-D508 #1)** | — | — | PUL=GPIO 25, DIR=GPIO 26 | Step pulse train + Direction |
| **Right Drive (CS-D508 #2)** | — | — | PUL=GPIO 32, DIR=GPIO 33 | Step pulse train + Direction |
| **MPU6050 IMU** | — | — | SDA=GPIO 21, SCL=GPIO 22 | I2C Telemetry at 50Hz |

---

## 3. Software Architecture & Flow

```text
                                 +-------------------------------------+
                                 |         PS5 DualSense Joy           |
                                 +-------------------------------------+
                                                    | (/joy)
                                                    v
                                 +-------------------------------------+
                                 |       teleop_twist_joy_node         |
                                 +-------------------------------------+
                                                    | (/cmd_vel)
                                                    v
                                 +-------------------------------------+
                                 |       twist_to_wheel_cmd.py         |
                                 |   - 50Hz S-Curve Jerk Limiter       |
                                 |   - Tea-Cup Safe Kinematics         |
                                 +-------------------------------------+
                                                    | (/cmd_pos [rps_L, rps_R])
                                                    v
                                 +-------------------------------------+
                                 |        esp32_driver_node.py         |
                                 |   - Serial Bridge /dev/esp32        |
                                 +-------------------------------------+
                                         |                      ^
                         ("V <rps1> <rps2>\n")                  | ("P <pos1> <pos2>...")
                                         v                      |
                                 +-------------------------------------+
                                 |          ESP32 Firmware             |
                                 |   - Non-blocking Pulse Gen          |
                                 |   - Frequency-Modulated Steps       |
                                 +-------------------------------------+
                                         |
                                         v (PUL/DIR 24V Opto)
                                 +-------------------------------------+
                                 |       CS-D508 Stepper Drives        |
                                 +-------------------------------------+

               ================== MAPPING & TF PIPELINE ==================

   [RPLiDAR A1] --------> (/scan) --------> [slam_toolbox (LifecycleNode)]
                                                    |
                                                    v
                                            (/map, TF: map -> odom)

   [ESP32 Telemetry] ---> (/odom) --------> (TF: odom -> base_footprint)

   [robot_state_publisher] ---------------> (TF: base_footprint -> base_link -> laser_frame)
```

---

## 4. Key Engineering Challenges Solved

### 1. The USB Port Shuffle & Permission Errors
- **Problem:** ESP32 and RPLiDAR shared the same Silicon Labs CP2102 USB-UART chip ID (`10c4:ea60`). Linux dynamically assigned random port numbers (`ttyUSB0`..`ttyUSB3`) and reverted permissions to restricted `0660`.
- **Solution:** 
  1. Created `/etc/udev/rules.d/99-servicebot-usb.rules` mapping physical kernel paths to permanent `/dev/rplidar` and `/dev/esp32` symlinks with global `0666` permissions.
  2. Implemented `auto_detect_ports.py` that sends an RPLiDAR health query byte (`0xA5 0x52`) over serial to automatically differentiate LiDAR from ESP32 on the fly.

### 2. Missing "Map" Frame in RViz & Broken TF Trees
- **Problem:** ROS 2 Jazzy's `async_slam_toolbox_node` is an internal Lifecycle Node. Launching it as a regular node left it dormant in the `unconfigured` state, causing RViz to fail with *"No transform from [base_link] to [map]"*.
- **Solution:** Configured `LifecycleNode` in `mapping.launch.py` with an automatic 5-second delayed `TRANSITION_CONFIGURE` and event-driven `TRANSITION_ACTIVATE`. Once active, it maps environments and publishes the `map -> odom` transform at 50Hz.

### 3. RealSense Optical Frame Disconnects
- **Problem:** RealSense ROS 2 node prefixed frame names with `camera_`, creating a disconnected branch in the TF tree (`camera_camera_link`).
- **Solution:** Standardized URDF optical link names and configured `tf_prefix: ''` in launch parameters.

### 4. Severe Motor Jerk & Initial Current Jolt (Tea-Spill Problem)
- **Problem:** 
  1. `twist_to_wheel_cmd.py` ran without a fixed timer, applying a full acceleration step in $< 60\text{ms}$ upon receiving joystick commands.
  2. Discrete position stepping (`M`) caused stop-start stuttering.
- **Solution:**
  1. **50Hz S-Curve Filter:** Upgraded `twist_to_wheel_cmd.py` with a fixed-interval 2nd-order S-Curve jerk limiter ($\text{Jerk} \le 0.75\text{ m/s}^3$, $\text{Accel} \le 0.25\text{ m/s}^2$).
  2. **Direct Velocity Protocol:** Transmitted continuous velocity commands (`V <rps1> <rps2>`) to ESP32.
  3. **Continuous Step Rate Modulation:** ESP32 firmware calculates step pulse half-periods (`half_us`) dynamically in `micros()` without blocking delays.

---

## 5. File & Repository Structure

```text
ros2_ws/
├── PROJECT_SUMMARY.md                   # This summary document
├── README.md                            # Quickstart & operational manual
├── 99-servicebot-usb.rules              # Linux udev rules for persistent hardware ports
├── .gitignore                           # Excludes build/, install/, log/, cache
└── src/
    ├── amr_assembly_description/        # Core Robot Package
    │   ├── config/
    │   │   └── mapper_params_online_async.yaml # SLAM Toolbox tuning parameters
    │   ├── firmware/
    │   │   └── esp32_serial_only/
    │   │       └── esp32_serial_only.ino       # ESP32 Zero-Jerk Motor Firmware
    │   ├── launch/
    │   │   ├── display.launch.py               # URDF visualization
    │   │   └── mapping.launch.py               # Master SLAM + Teleop + Drivers Launch
    │   ├── rviz/
    │   │   └── mapping.rviz                    # RViz configuration
    │   ├── scripts/
    │   │   ├── auto_detect_ports.py            # Hardware serial port prober
    │   │   ├── dummy_joint_state_publisher.py  # Caster wheel joint broadcaster
    │   │   ├── dummy_laser_scan_publisher.py   # Simulation scan generator
    │   │   ├── esp32_driver_node.py            # ROS 2 Serial Driver + Odometry Publisher
    │   │   ├── esp32_wifi_driver_node.py       # Wi-Fi UDP alternative driver
    │   │   ├── sim_robot_odometry.py           # Simulation odometry generator
    │   │   └── twist_to_wheel_cmd.py           # 50Hz S-Curve Jerk-Limited Kinematics
    │   └── urdf/
    │       └── amr_assembly_description.urdf   # Complete Robot URDF Model
    ├── servicebot_description/                 # Alternative URDF & meshes package
    └── sllidar_ros2/                           # Official RPLiDAR ROS 2 SDK & Driver
```

---

## 6. How to Run the System

### A. Clean Launch (Real Hardware Mode)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch amr_assembly_description mapping.launch.py use_esp32:=true
```

### B. Controls (PS5 DualSense Controller)
- **Hold L1**: Enable movement (deadman safety switch)
- **Left Stick Vertical**: Smooth forward / backward motion ($0.25\text{ m/s}$)
- **Left Stick Horizontal**: Smooth rotation ($0.60\text{ rad/s}$)
- **Hold R1 (while holding L1)**: Turbo mode ($0.45\text{ m/s}$)

### C. Quick Diagnostics (1-Liner)
```bash
# Check USB ports
ls -la /dev/rplidar /dev/esp32

# Check TF Transform Tree
ros2 run tf2_ros tf2_echo map base_footprint

# Clean stuck background processes
pkill -9 -f "sllidar|slam_toolbox|realsense|esp32_driver"
```
