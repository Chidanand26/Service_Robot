# Transformation Matrices - servicebot

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | servicebot_wheels |
| $L_{2}$ | servicebot_wheels_2 |
| $L_{3}$ | lidar |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_1 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_2 | continuous (rad) | $L_{0}$ | $L_{2}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [continuous] Revolute_1 (q1)
  |   L1: servicebot_wheels
  |-- [continuous] Revolute_2 (q2)
  |   L2: servicebot_wheels_2
  +-- [fixed] Rigid_3
      L3: lidar
```

## Transforms

## Revolute_1

$L_{0}$ **base_link** -> $L_{1}$ **servicebot_wheels** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0.19, 0.05, 0) m
- **origin rpy**: (-3.141593, 0, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.19 \\
0 & -1 & 0 & 0.05 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & 0 & s_{1} & 0 \\
0 & 1 & 0 & 0 \\
-s_{1} & 0 & c_{1} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_2

$L_{0}$ **base_link** -> $L_{2}$ **servicebot_wheels_2** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.19, 0.395, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$$
T^{0}_{2}(q_{2}) = \begin{bmatrix}
c_{2} & 0 & s_{2} & 0.19 \\
0 & 1 & 0 & 0.395 \\
-s_{2} & 0 & c_{2} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_3

$L_{0}$ **base_link** -> $L_{3}$ **lidar** (fixed)

- **origin xyz**: (0.37, 0.2225, 0.254) m
- **origin rpy**: (0, 0, -1.570796) rad

### Local Transform

$$
T^{0}_{3} = \begin{bmatrix}
0 & 1 & 0 & 0.37 \\
-1 & 0 & 0 & 0.2225 \\
0 & 0 & 1 & 0.254 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

