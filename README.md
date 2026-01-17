# 6-DOF Manipulator Kinematics & Control in ROS 2

**Forward Kinematics • Inverse Kinematics (Damped Least Squares) • ROS 2 Python nodes**

This repository is a ROS 2 (Python) implementation for a 6-DOF robotic manipulator, built mainly for coursework and research prototyping. It includes:

* Forward kinematics (FK) with the Denavit–Hartenberg (DH) convention
* Inverse kinematics (IK) with Damped Least Squares (DLS) for singularity-robust numerical solving
* Small utility nodes to make testing easy (including a dummy `JointState` publisher)

The focus is on clarity and experimentation rather than a full MoveIt 2 pipeline.

## What’s inside

### Forward Kinematics (FK)

* DH-based chain model
* Computes the full (4 \times 4) transform (T_0^6)
* Publishes the end-effector pose as `geometry_msgs/PoseStamped` on `/end_effector_pose`
* Joint order can be set through parameters (so it matches your `JointState.name`)

### Inverse Kinematics (IK) — Damped Least Squares (DLS)

* Iterative, numerical IK using DLS (more stable near singularities than plain pseudoinverse)
* Numerical Jacobian (position-only, (3 \times 6))
* Publishes joint solutions as `sensor_msgs/JointState` on `/ik_joint_states`
* Logs convergence/error so you can see if it’s drifting or oscillating

> Current IK is position-only. Orientation IK is a straightforward extension and listed in the roadmap.

### Dummy JointState publisher

* Publishes sinusoidal joint trajectories
* Useful when you want to test FK/IK without Gazebo, RViz, or real hardware

## Repository layout

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

## Requirements

* ROS 2 (Humble / Iron / Jazzy should be fine)
* Python 3
* `rclpy`
* `numpy`

ROS messages:

* `sensor_msgs/msg/JointState`
* `geometry_msgs/msg/PoseStamped`

## Build

Create a workspace (if you don’t already have one):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the package into `src/`:

```bash
git clone <YOUR_REPO_URL> robotic_arm_advanced
```

Build and source:

```bash
cd ~/ros2_ws
colcon build --packages-select robotic_arm_advanced
source install/setup.bash
```

## Run

### A) Dummy publisher → FK

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

Check the FK output:

```bash
ros2 topic echo /end_effector_pose
```

### B) IK (DLS)

**Terminal 3**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm_advanced ik_dls_node
```

Check IK output:

```bash
ros2 topic echo /ik_joint_states
```

Depending on how your IK node is set up, it may either:

* subscribe to a target pose topic (recommended), or
* use an internal test target.

## Topics

Published:

* `/joint_states` (`sensor_msgs/JointState`) — from `dummy_joint_pub.py`
* `/end_effector_pose` (`geometry_msgs/PoseStamped`) — from `fk_node.py`
* `/ik_joint_states` (`sensor_msgs/JointState`) — from `ik_dls_node.py`

Subscribed (typical):

* `/joint_states` by `fk_node.py`
* `/end_effector_target` (`geometry_msgs/PoseStamped`) by `ik_dls_node.py` (if you use an external target)

If your target topic has a different name, update it here (or expose it as a parameter).

## Parameters (typical)

### `fk_node.py`

* `joint_names` (string array): expected joint order
* `base_frame` (string): e.g. `base_link`
* `ee_frame` (string): e.g. `tool0`

### `ik_dls_node.py`

* `damping` (float): (\lambda) for DLS
* `max_iters` (int)
* `tol` (float): position tolerance (m)
* `step_size` (float): optional scaling for updates
* `joint_names` (string array)

Example (FK joint ordering):

```bash
ros2 run robotic_arm_advanced fk_node --ros-args \
  -p joint_names:="['joint1','joint2','joint3','joint4','joint5','joint6']"
```

## Practical notes for IK tuning

If the IK oscillates or diverges:

* increase `damping` (try 0.05 → 0.2)
* reduce `step_size`
* clamp to joint limits (recommended)

## Roadmap / extensions

* Orientation IK (6D pose error) + full (6 \times 6) Jacobian
* Joint limits + constraint handling
* RViz / MoveIt 2 integration
* Analytic Jacobian (instead of numerical)
