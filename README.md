# PROJECT : SPACE_STATION

## Table of Contents :
- [Introduction](#introduction)
- [Position and Velocity Control of DC Motor](#position-and-velocity-control-of-dc-motor)
- [Collecting Raw Data of IMU and Applying Kalman Filter](#collecting-raw-data-of-imu-and-applying-kalman-filter)
- [Dual Booting Windows and Ubuntu 20.04](#dual-booting-windows-and-ubuntu-2004)
- [Installing ROS Noetic](#installing-ros-noetic)
- [4 Omni Wheel Testbot](#4-omni-wheel-testbot)
- [Integrating Arduino IDE and ROS](#integrating-arduino-ide-and-ros)
- [Controlling of TESTBOT](#controlling-of-testbot)
- [License Info](#license-info)

- ## Introduction

Welcome to the Space Station project,   Robotics Club at IIT Kanpur. This project aims to develop a  robotic system capable of precise motion control and efficient data collection using a imu sensor and encoder and pid control.

The project involves multiple aspects of robotics, including:

- **Position and Velocity Control of DC Motors:** Utilizing PID controllers to achieve accurate motor control.
- **Sensor Data Processing:** Collecting raw data from an Inertial Measurement Unit (IMU) and applying a Kalman filter for noise reduction and improved measurement accuracy.
- **System Setup:** Dual booting systems with Windows and Ubuntu 20.04 to facilitate development and installing ROS Noetic for robot control and simulation.
- **Hardware Development:** Designing and building a 4 Omni Wheel Testbot to test various control algorithms and integration techniques.
- **Software Integration:** Integrating Arduino IDE with ROS for seamless hardware control and communication.

This README provides a comprehensive guide to the various components of the project, including detailed instructions, code examples, and troubleshooting tips. We welcome contributions from the community and encourage you to get involved in the development of this exciting project.

## Position and Velocity Control of DC Motor

In this section, we focus on the position and velocity control of DC motors, essential for achieving precise movements in our robotic system.

### Overview

The control of DC motors involves regulating their position and velocity to meet desired setpoints accurately. We use PID (Proportional-Integral-Derivative) controllers for this purpose, which help minimize the error between the desired and actual motor states.

### Components

- **DC Motors:** The primary actuators for movement.
- **Encoders:** Used for feedback to measure the motor's position and velocity.
- **PID Controllers:** Implemented in software to control the motors.

### Steps to Implement Position and Velocity Control

1. **Set Up Hardware:**
   - Connect the DC motor to the motor driver and the encoder to the microcontroller.
   - Ensure proper power supply and connections.

2. **Configure PID Controllers:**
   - Implement PID control algorithms in your microcontroller or a connected computer.
   - Tune the PID parameters (Kp, Ki, Kd) to achieve the desired response.

3. **Write Control Code:**
   - Read encoder values to get current position and velocity.
   - Compute the error between desired setpoints and current values.
   - Apply PID control to adjust motor inputs accordingly.

### Using the Code from the Repository

We have provided a comprehensive example of PID control for DC motors in our repository. To use this code, follow these steps:

1. **Clone the Repository:**
   ```sh
   git clone https://github.com/Shivansh-gupta2005/Space_Station_snt-.git
   ```

2. **Navigate to the Control Code Directory:**
   ```sh
   cd Space_Station_snt/velocity_control_of_dc_motor.cpp
   ```

3. **Upload the Code to Your Microcontroller:**
   Open the provided Arduino code in the `position_velocity_control` directory using the Arduino IDE and upload it to your microcontroller.

### Tuning PID Parameters

Tuning the PID parameters (Kp, Ki, Kd) is crucial for optimal performance. Here are some tips:

- **Kp (Proportional Gain):** Adjusts the response to the current error. Higher values increase the response speed but can cause overshoot.
- **Ki (Integral Gain):** Addresses accumulated errors over time. Higher values eliminate steady-state errors but can cause instability.
- **Kd (Derivative Gain):** Responds to the rate of change of the error. Higher values reduce overshoot and improve stability.

### Conclusion

Accurate position and velocity control of DC motors are vital for the precision and reliability of our robotic system. By carefully tuning PID controllers and ensuring proper feedback mechanisms, we can achieve the desired performance in our Space Station project. For the complete code and detailed instructions, please refer to the [repository]([https://github.com/your-repo/space-station](https://github.com/Shivansh-gupta2005/Space_Station_snt-.git)).
