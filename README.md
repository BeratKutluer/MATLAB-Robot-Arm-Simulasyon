# MATLAB Robot Arm Simulation
### An AI-Driven Object Sorting Project

This project simulates a robotic arm performing an object sorting task. Developed entirely in MATLAB, it serves as a practical demonstration of fundamental robotics and control system principles.

<p align="center">
  <img src="robot_animasyonu.gif" alt="Robot Animation">
</p>

## Key Skills Demonstrated
- **Manual Kinematic Modeling**: The robot's physical structure and movement are modeled from scratch without using any specialized toolboxes.
- **Inverse Kinematics**: Joint angles required to reach specific target positions are calculated by manually solving kinematic equations.
- **AI Simulation**: The robot's task of identifying and sorting objects is simulated by having it move to a series of predetermined target locations.
- **Problem-Solving & Debugging**: The project showcases the ability to diagnose and solve common robotics problems, such as unreachable targets and mathematical errors.

## How It Works
- The `robot_manuel.m` file contains the complete kinematic model and movement algorithms for the 3-DOF robot arm.
- The script uses inverse kinematics to calculate joint angles for each "object" position.
- A single animation is created by continuously updating the robot's position, resulting in a GIF file that demonstrates the sorting process.

---
**Developer:** Berat Kutluer
