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

Accurate position and velocity control of DC motors are vital for the precision and reliability of our robotic system. By carefully tuning PID controllers and ensuring proper feedback mechanisms, we can achieve the desired performance in our Space Station project. For the complete code and detailed instructions, please refer to the [repository](https://github.com/Shivansh-gupta2005/Space_Station_snt-.git).

## Collecting Raw Data of IMU and Applying Kalman Filter

In this section, we focus on collecting raw data from an Inertial Measurement Unit (IMU) and applying a Kalman filter to improve the accuracy of our measurements by reducing noise.

### Overview

An IMU provides critical data about the orientation and movement of our robotic system. However, the raw data from an IMU can be noisy. To address this, we use a Kalman filter, which is an algorithm that estimates the true state of a system from noisy measurements.

### Components

- **IMU Sensor:** Used to measure acceleration, angular velocity, and sometimes magnetic field.
- **Microcontroller:** For reading the IMU data and implementing the Kalman filter.
- **Kalman Filter:** An algorithm to filter out noise and provide more accurate measurements.

### Steps to Collect and Filter IMU Data

1. **Set Up Hardware:**
   - Connect the IMU sensor to the microcontroller.
   - Ensure proper power supply and connections.

2. **Collect Raw IMU Data:**
   - Write code to read raw data from the IMU sensor. This typically includes acceleration and gyroscope readings.

3. **Implement the Kalman Filter:**
   - Apply the Kalman filter to the raw data to reduce noise and improve accuracy.

### Using the Code from the Repository

We have provided a comprehensive example of collecting IMU data and applying a Kalman filter in our repository. To use this code, follow these steps:



2. **Navigate to the IMU Data Collection Directory:**
   ```sh
   cd Space_Station_snt/imu_data.cpp
   ```

3. **Upload the Code to Your Microcontroller:**
   Open the provided Arduino code in the `imu_data_collection` directory using the Arduino IDE and upload it to your microcontroller.

### Example Workflow

1. **Reading IMU Data:**
   - Initialize the IMU sensor.
   - Read acceleration and gyroscope data from the sensor.

2. **Applying the Kalman Filter:**
   - Initialize the Kalman filter parameters.
   - Continuously read IMU data and apply the Kalman filter to get filtered data.

### Conclusion

Collecting raw data from the IMU and applying a Kalman filter is crucial for accurate and reliable sensor measurements in our robotic system. By implementing this process, we can significantly reduce noise and obtain precise data for our Space Station project. For the complete code and detailed instructions, please refer to the [repository](https://github.com/Shivansh-gupta2005/Space_Station_snt-.git).
