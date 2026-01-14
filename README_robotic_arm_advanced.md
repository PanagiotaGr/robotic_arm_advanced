# 6-DOF Manipulator Kinematics & Control in ROS 2
**Forward Kinematics • Inverse Kinematics (Damped Least Squares) • Python Nodes**

This repository contains a complete **ROS 2 (Python)** implementation for a **6-DOF robotic manipulator**, including:
- **Forward Kinematics (FK)** using **Denavit–Hartenberg (DH)** convention
- **Inverse Kinematics (IK)** using **Damped Least Squares (DLS)** (singularity-robust)
- Utility/testing nodes (dummy JointState publisher)

Designed for **postgraduate-level coursework** in robotics, control, and ROS-based robotic systems.

---

##  Features

###  Forward Kinematics (FK)
- Implemented using the **Denavit–Hartenberg (DH)** convention  
- Computes full **4×4 transform** \(T_0^6\)  
- Publishes end-effector pose as `geometry_msgs/PoseStamped`  
- Supports **configurable joint names** via parameters

### Inverse Kinematics (IK) — Damped Least Squares (DLS)
- Robust numerical IK using **DLS** for singularity handling  
- **Numerical Jacobian** approximation (**3×6**, position-only)  
- **Position-based IK** (orientation can be added later)  
- Iterative solver with convergence / error logging  
- Publishes joint solutions as `sensor_msgs/JointState`

###  Dummy JointState Publisher
- Generates **sinusoidal joint trajectories**
- Enables testing FK/IK nodes **without simulation or hardware**

---

## Package Structure

```
robotic_arm_advanced/
├── robotic_arm_advanced/
│   ├── fk_node.py
│   ├── ik_dls_node.py
│   ├── dummy_joint_pub.py
│   └── __init__.py
├── package.xml
├── setup.py
└── README.md
```

---

##  Requirements

- ROS 2 (Humble / Iron / Jazzy should work—any recent ROS 2 distro)
- Python 3
- `rclpy`
- `numpy`

Messages used:
- `sensor_msgs/msg/JointState`
- `geometry_msgs/msg/PoseStamped`

---

##  Build & Run

### 1) Create/enter a ROS 2 workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2) Clone this repository into `src/`
```bash
# example
git clone <YOUR_REPO_URL> robotic_arm_advanced
```

### 3) Build
```bash
cd ~/ros2_ws
colcon build --packages-select robotic_arm_advanced
source install/setup.bash
```

---

##  Running the Nodes

### A) Dummy publisher → FK (visualize end-effector pose)
**Terminal 1**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm_advanced dummy_joint_pub
```

**Terminal 2**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm_advanced fk_node
```

Inspect FK output:
```bash
ros2 topic echo /end_effector_pose
```

---

### B) IK (DLS) node
**Terminal 3**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm_advanced ik_dls_node
```

Inspect IK joint outputs:
```bash
ros2 topic echo /ik_joint_states
```

> Note: Depending on your implementation, the IK node may subscribe to a target pose topic (recommended), or it may use an internal/test target. See parameters below.

---

##  Topics

### Published
- `/joint_states` (`sensor_msgs/JointState`)  
  From `dummy_joint_pub.py` (test input)

- `/end_effector_pose` (`geometry_msgs/PoseStamped`)  
  From `fk_node.py`

- `/ik_joint_states` (`sensor_msgs/JointState`)  
  From `ik_dls_node.py`

### Subscribed (typical)
- `/joint_states` by `fk_node.py` (current joints)
- `/end_effector_target` (`geometry_msgs/PoseStamped`) by `ik_dls_node.py` *(if implemented)*

If your IK node uses a different topic name, update it here (or set via parameters).

---

## ⚙️ Parameters (Typical)

### `fk_node.py`
- `joint_names` (string array): joint order expected in `JointState.name`
- `base_frame` (string): e.g. `"base_link"`
- `ee_frame` (string): e.g. `"tool0"`

### `ik_dls_node.py`
- `damping` (float): \(\lambda\) in DLS
- `max_iters` (int): maximum iterations
- `tol` (float): position error tolerance
- `step_size` (float): optional scaling for updates
- `joint_names` (string array): output joint order

Example:
```bash
ros2 run robotic_arm_advanced fk_node --ros-args   -p joint_names:="['joint1','joint2','joint3','joint4','joint5','joint6']"
```

---

## 🧠 Math Summary

### Forward Kinematics (DH)
For each link \(i\), the DH transform:
\[
A_i(\theta_i, d_i, a_i, \alpha_i) =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i  & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0             & \sin\alpha_i               & \cos\alpha_i               & d_i \\
0             & 0                           & 0                           & 1
\end{bmatrix}
\]
Then:
\[
T_0^6 = A_1 A_2 A_3 A_4 A_5 A_6
\]

### Inverse Kinematics (Damped Least Squares)
Position-only IK (3D):
\[
\Delta x = x_{target} - x_{current}
\]
Jacobian (numerical) \(J \in \mathbb{R}^{3\times 6}\), update:
\[
\Delta q = J^T (J J^T + \lambda^2 I)^{-1} \Delta x
\]
Iterate:
\[
q \leftarrow q + \Delta q
\]
until \(\|\Delta x\| < \text{tol}\) or `max_iters` reached.

---

## 🧪 Testing Tips

- Start with conservative values:
  - `damping = 0.05` to `0.2`
  - `max_iters = 100` to `300`
  - `tol = 1e-3` to `1e-4` (meters)
- If IK oscillates or diverges:
  - increase `damping`
  - reduce `step_size`
  - set joint limits/clamping (recommended)

---

## 🗺️ Roadmap (Optional Extensions)
- Add **orientation IK** (6D pose error) and full **6×6 Jacobian**
- Add **joint limits** & constraint handling
- Integrate with RViz / MoveIt 2
- Replace numerical Jacobian with analytic Jacobian

---

## 📄 License / Academic Use
Intended for educational and research use. Add a license file if required by your course or institution.

---

## ✍️ Author / Course Info
Add your name, university, course title, and semester here if you want:
- **Name:** …
- **Course:** …
- **University:** …
- **Year/Semester:** …
