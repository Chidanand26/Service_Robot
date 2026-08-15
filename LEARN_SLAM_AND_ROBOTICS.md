# Complete Guide to SLAM, Differential Drive Kinematics & Robotics

This guide explains the complete theory, mathematical foundations, algorithms, and end-to-end software pipeline behind your **AMR ServiceBot** mapping and navigation stack.

---

## 1. What is SLAM? The Core Problem

**SLAM** stands for **Simultaneous Localization and Mapping**.

### The "Chicken-and-Egg" Dilemma
- To build an accurate **Map** of an unknown room, you need to know the robot's exact **Position** at every millisecond.
- To know the robot's exact **Position**, you need an accurate **Map** of the environment to reference against.

SLAM algorithms solve both simultaneously by fusing:
1. **Internal Dead-Reckoning (Odometry)**: Wheel encoder counts measuring wheel rotations.
2. **External Environment Perception (LiDAR)**: Laser range measurements measuring distances to obstacles.

```text
[Wheel Encoders] ────> Dead-Reckoning Pose (Drifts over time) ───┐
                                                                ├──> [Graph-SLAM Optimizer] ───> Global Map & True Robot Pose
[RPLiDAR Scans]  ────> Environment Geometry (Laser Rays)    ───┘
```

---

## 2. Coordinate Frames & ROS Standard Transforms (REP-105 & REP-103)

In robotics, different parts of the system live in different coordinate frames. ROS 2 maintains a tree of geometric transformations called the **TF Tree**.

```text
+-------------------+
|       map         |   <--- Fixed global world frame (Origin of the building)
+---------+---------+
          |
          |  (Dynamic TF: Published by SLAM Toolbox / Scan Matching)
          v
+-------------------+
|      odom         |   <--- Continuous local world frame (Accumulates wheel drift)
+---------+---------+
          |
          |  (Dynamic TF: Published by esp32_wifi_driver_node.py / Wheel Odometry)
          v
+-------------------+
|   base_footprint  |   <--- 2D Ground plane projection of the robot center (Z = 0)
+---------+---------+
          |
          |  (Fixed TF: base_footprint_joint Z = +0.055m, from URDF)
          v
+-------------------+
|    base_link      |   <--- Structural center of the robot chassis
+---------+---------+
          |
          +---------> left_wheel   (Continuous joint Revolute_1 / left_wheel_joint)
          +---------> right_wheel  (Continuous joint Revolute_2 / right_wheel_joint)
          +---------> lidar_link   (Fixed joint, Z = +0.760m)
          |               |
          |               v
          |          laser_frame   (LiDAR scan emission optical frame)
          |
          +---------> camera_link  ---> camera_depth_optical_frame
          +---------> imu_link
```

### Why Do We Need Both `map` AND `odom`?
- **`odom` $\rightarrow$ `base_footprint`**: Calculated from wheel encoders. It is **smooth and continuous** (no sudden jumps), but it **drifts over time** due to wheel slippage and floor imperfections.
- **`map` $\rightarrow$ `odom`**: Calculated by `slam_toolbox`. When SLAM recognizes a previously visited wall or corner (**Loop Closure**), it corrects the accumulated drift by adjusting the `map -> odom` transform. This makes `map` globally accurate, while keeping `odom` locally smooth for motion controllers.

---

## 3. Differential Drive Kinematics & Odometry Math

Your robot uses **Two-Wheel Differential Drive**:
- Wheel radius: $R = 0.080\text{ m}$ ($80\text{ mm}$)
- Wheel track separation (distance between left and right wheels): $W = 0.427\text{ m}$ ($427\text{ mm}$)

```text
               ^ +X (Forward)
               |
        [Left Wheel] <--- W = 0.427m ---> [Right Wheel]
            (v_L)                             (v_R)
               |                               |
               +---------------O---------------+ ---> +Y (Left)
                           (Center)
```

### Forward Kinematics (Velocities to Robot Motion)

Given commanded forward velocity $v$ (m/s) and angular yaw rate $\omega$ (rad/s):
$$v_{\text{left}} = v - \frac{\omega \cdot W}{2}$$
$$v_{\text{right}} = v + \frac{\omega \cdot W}{2}$$

Wheel angular velocity in revolutions per second:
$$\text{Rev}_{\text{left}} = \frac{v_{\text{left}}}{2\pi R}$$
$$\text{Rev}_{\text{right}} = \frac{v_{\text{right}}}{2\pi R}$$

### Dead-Reckoning Odometry (Encoder Ticks to Pose $(x, y, \theta)$)

Every $20\text{ ms}$ ($50\text{ Hz}$), the ESP32 sends wheel positions $pos_1, pos_2$ in revolutions.
1. Compute incremental wheel distances:
   $$\Delta d_L = (pos_{1} - pos_{1,\text{prev}}) \cdot (2\pi R)$$
   $$\Delta d_R = (pos_{2} - pos_{2,\text{prev}}) \cdot (2\pi R)$$
2. Compute linear displacement $\Delta d$ and rotation $\Delta \theta$:
   $$\Delta d = \frac{\Delta d_L + \Delta d_R}{2}$$
   $$\Delta \theta = \frac{\Delta d_R - \Delta d_L}{W}$$
3. Integrate pose using 2nd-order Runge-Kutta (midpoint integration):
   $$\theta_{\text{mid}} = \theta + \frac{\Delta \theta}{2}$$
   $$x_{t+1} = x_t + \Delta d \cdot \cos(\theta_{\text{mid}})$$
   $$y_{t+1} = y_t + \Delta d \cdot \sin(\theta_{\text{mid}})$$
   $$\theta_{t+1} = \theta_t + \Delta \theta$$

---

## 4. How 2D LiDAR Works (RPLiDAR A1/A2)

1. The RPLiDAR spins an infrared laser diode and optical sensor at **$10\text{ Hz}$ ($600\text{ RPM}$)**.
2. For each laser pulse, it measures the Time-of-Flight / Triangulation distance $r_i$ at angle $\alpha_i$.
3. Converts polar coordinates $(r_i, \alpha_i)$ into Cartesian 2D coordinates $(x_i, y_i)$ in `laser_frame`:
   $$x_i^{\text{laser}} = r_i \cos(\alpha_i)$$
   $$y_i^{\text{laser}} = r_i \sin(\alpha_i)$$
4. Using the TF Tree, ROS 2 transforms every laser point into the global `map` frame:
   $$\begin{bmatrix} x_i^{\text{map}} \\ y_i^{\text{map}} \\ 1 \end{bmatrix} = \mathbf{T}_{\text{map} \leftarrow \text{odom}} \cdot \mathbf{T}_{\text{odom} \leftarrow \text{base\_footprint}} \cdot \mathbf{T}_{\text{base\_footprint} \leftarrow \text{laser\_frame}} \cdot \begin{bmatrix} x_i^{\text{laser}} \\ y_i^{\text{laser}} \\ 1 \end{bmatrix}$$

---

## 5. How SLAM Toolbox Works (Graph SLAM & Ceres Solver)

`slam_toolbox` uses **Pose-Graph SLAM** optimized with Google's **Ceres Solver**:

```text
    (Pose Node 0) ───[Odom Edge]───> (Pose Node 1) ───[Odom Edge]───> (Pose Node 2)
          |                                                                 |
          |                                                                 |
          +===================[Loop Closure Constraint]====================+
```

### Key Steps Inside SLAM Toolbox:

1. **Scan Matching (Correlative / ICP Matching)**:
   - When a new scan arrives, SLAM compares the shape of the scan against existing map geometry to find the most probable robot pose $(x, y, \theta)$.
2. **Graph Construction**:
   - Creates a **Node** for each key robot pose.
   - Creates an **Edge** (constraint) between nodes based on odometry and scan-match alignment.
3. **Loop Closure Detection**:
   - When the robot drives around a room and returns to an area visited minutes ago, SLAM compares the current scan against the old submap.
   - If a strong match is found, a **Loop Closure Edge** is added across the graph.
4. **Ceres Non-Linear Least Squares Optimization**:
   - Solves the global optimization problem:
     $$\min_{\mathbf{X}} \sum_{i,j} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)^T \mathbf{\Omega}_{ij} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)$$
   - Adjusts all past poses simultaneously to eliminate accumulated drift and snap the map into alignment.

---

## 6. Occupancy Grid Mapping & Map File Format

The resulting 2D map is an **Occupancy Grid**:
- **Grid Resolution**: $0.05\text{ m}$ ($5\text{ cm}$ per pixel).
- **Cell States**:
  - **$0$ (White)**: Free walkable space (laser rays passed through unobstructed).
  - **$100$ (Black)**: Occupied obstacle / wall (laser rays bounced back).
  - **$-1$ (Grey)**: Unknown unmapped area (laser never reached here).

### Saved Map Files (`~/my_servicebot_map`):
- `my_servicebot_map.pgm`: Portable Graymap image file storing pixel values.
- `my_servicebot_map.yaml`: Metadata file:
  ```yaml
  image: my_servicebot_map.pgm
  resolution: 0.050000            # 5cm per pixel
  origin: [-10.456, -10.788, 0.0] # World coordinates (x, y, yaw) of bottom-left pixel
  negate: 0
  occupied_thresh: 0.65           # Probability > 65% is considered an obstacle
  free_thresh: 0.196              # Probability < 19.6% is considered free space
  ```

---

## 7. Complete End-to-End Pipeline Trace

Here is what happens when you press `i` on your keyboard in `teleop_twist_keyboard`:

1. **`teleop_twist_keyboard`** publishes `geometry_msgs/msg/Twist` on topic `/cmd_vel` ($v = 0.5\text{ m/s}, \omega = 0.0$).
2. **`twist_to_wheel_cmd.py`** converts velocity to target revolutions and publishes `Float32MultiArray` on `/cmd_pos` ($[1.0, 1.0]$ revs).
3. **`esp32_wifi_driver_node.py`** sends UDP packet `M 1.0000 1.0000\n` over Wi-Fi to ESP32 on port `8888`.
4. **ESP32** generates trapezoidal pulse ramps on GPIO 25 & 32 to rotate the CS-D508 stepper drives.
5. **ESP32** reads pulse counters and sends UDP telemetry `P 1.0000 1.0000 0 0 0 0\n` at $50\text{ Hz}$ back to the Pi.
6. **`esp32_wifi_driver_node.py`** receives telemetry, computes differential odometry, and broadcasts TF `odom -> base_footprint`.
7. **`sllidar_node`** reads serial packets from `/dev/ttyUSB0` and publishes laser ranges on `/scan`.
8. **`slam_toolbox`** receives `/scan` + TF, matches scans against Ceres graph, updates the occupancy grid, and broadcasts TF `map -> odom`.
9. **`rviz2`** renders the live updated 2D map, robot 3D mesh, and red laser scan points on your screen.
