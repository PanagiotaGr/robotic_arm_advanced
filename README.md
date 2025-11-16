# 6-DOF Manipulator Kinematics & Control in ROS 2  
**Forward Kinematics • Inverse Kinematics (Damped Least Squares) • Python Nodes**

This repository contains a complete ROS 2 (Python) implementation of a 6-DOF robotic manipulator, including forward kinematics (FK), inverse kinematics (IK) using the Damped Least Squares (DLS) method, and utility nodes for testing.  
The project is designed for **postgraduate-level coursework** in robotics, control, and ROS-based robotic systems.

---

## 📌 Features

### ✔ Forward Kinematics (FK)
- Implemented using the **Denavit–Hartenberg (DH)** convention  
- Computes the full 4×4 transformation `T₀⁶`  
- Publishes end-effector pose as `geometry_msgs/PoseStamped`  
- Supports configurable joint names through parameters

### ✔ Inverse Kinematics (IK)
- Uses **Damped Least Squares (DLS)** for singularity-robust IK  
- Numerical Jacobian approximation (3×6)  
- Position-based IK (orientation can be added later)  
- Iterative solver with convergence logging  
- Publishes joint solutions as `sensor_msgs/JointState`

### ✔ Dummy JointState Publisher
- Generates sinusoidal joint trajectories  
- Allows FK and IK nodes to be tested without simulation or hardware  

---

## 📁 Package Structure

