# 6-DOF Manipulator Kinematics & Control in ROS 2  
**Forward Kinematics • Inverse Kinematics (Damped Least Squares) • Python Nodes**

This repository contains a complete ROS 2 (Python) implementation of a 6-DOF robotic manipulator, including forward kinematics (FK), inverse kinematics (IK) using the Damped Least Squares (DLS) method, and utility nodes for testing.  
The project is designed for **postgraduate-level coursework** in robotics, control, and ROS-based robotic systems.

## Features

### Forward Kinematics (FK)
- Implemented using the Denavit–Hartenberg (DH) convention  
- Computes the full 4×4 transformation `T₀⁶`  
- Publishes end-effector pose as `geometry_msgs/PoseStamped`  
- Supports configurable joint names through parameters

### Inverse Kinematics (IK)
- Uses Damped Least Squares (DLS) for singularity-robust IK  
- Numerical Jacobian approximation (3×6)  
- Position-based IK (orientation can be added later)  
- Iterative solver with convergence logging  
- Publishes joint solutions as `sensor_msgs/JointState`

### Dummy JointState Publisher
- Generates sinusoidal joint trajectories  
- Allows FK and IK nodes to be tested without simulation or hardware  

## Package Structure

robotic_arm_advanced/
│
├── robotic_arm_advanced/
│   ├── fk_node.py
│   ├── ik_dls_node.py
│   ├── dummy_joint_pub.py
│   └── __init__.py
│
├── package.xml
├── setup.py
└── README.md

## Mathematical Background

### Forward Kinematics  
The FK is computed using classical DH parameters.  
Each joint transform is:

T_i^{i+1} = ...

### Inverse Kinematics (Damped Least Squares)

Δθ = J^T(JJ^T + λ²I)^{-1}(x_d - x)

## Running the Nodes

### Build workspace

cd ~/ros2_ws
colcon build
source install/setup.bash

### Start dummy joint publisher

ros2 run robotic_arm_advanced dummy_joint_pub

### Run FK

ros2 run robotic_arm_advanced fk_node

### Run IK

ros2 run robotic_arm_advanced ik_dls_node

ros2 topic pub /ee_target geometry_msgs/PoseStamped ...

## License

Open-source for academic and research use.

## Author

Maintained by Panagiota Grosdouli (Lily-Evan).
