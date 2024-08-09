# Extended Kalman Filter for Mobile Robot Localization

## Objective
This project involves implementing an Extended Kalman Filter (EKF) to estimate the state of a mobile robot, focusing on its 2D position and orientation (x, y, theta). The goal is to accurately localize the robot using velocity commands and sensor data, accounting for noise and uncertainties in both the dynamic model and sensor measurements. The project was carried out in a ROS2 environment, utilizing a simulated robot with landmark-based localization.

[![YouTube Video](https://img.youtube.com/vi/xmrllcgkuGU/0.jpg)](https://www.youtube.com/watch?v=xmrllcgkuGU)

## Key Features

1. **Dynamic Model Implementation:**
   - Modeled the robot's dynamics to predict its next state based on translational and rotational velocities.
   - Accounted for noise in the dynamic model equations to ensure realistic simulations.

2. **Sensor Data Integration:**
   - Processed sensor data to determine the range and bearing to landmarks.
   - Incorporated noise characteristics to reflect real-world conditions.

3. **Extended Kalman Filter (EKF):**
   - EKF to fuse the predicted states from the dynamic model with the observed sensor data.
   - Refined the robot's pose estimate iteratively.

## Project Structure
- `ekf.py`: Main script for implementing the Extended Kalman Filter.
