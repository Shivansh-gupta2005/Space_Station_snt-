# Space_Station_snt-
PID control, Encoder data,IMU data,Rosserial
# DC Motor Control with Encoder PID and IMU Kalman Filter

This repository contains a project that demonstrates controlling a DC motor with an encoder using PID control. Additionally, it involves reading raw data from an IMU (Inertial Measurement Unit) and applying a Kalman filter for sensor fusion and noise reduction. The project is also integrated into a testbot equipped with four omni wheels, each wheel being perpendicular to the adjacent wheel, enabling precise and versatile movement. This project is developed using ROS Noetic.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [PID Control](#pid-control)
- [Kalman Filter](#kalman-filter)
- [Omni Wheel Testbot](#omni-wheel-testbot)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This project aims to provide a comprehensive solution for controlling a DC motor using an encoder for feedback, implementing PID control to achieve precise motor positioning. Furthermore, it incorporates an IMU to gather raw sensor data, which is then processed using a Kalman filter for accurate and stable measurements. The system is tested on a custom-built testbot with four omni wheels, enabling omnidirectional movement. The entire system is built using ROS Noetic.

## Features

- **DC Motor Control with Encoder PID**: Precise control of a DC motor using an encoder for feedback and a PID controller for stability.
- **IMU Data Processing**: Reading raw data from an IMU and applying a Kalman filter for sensor fusion and noise reduction.
- **Omni Wheel Testbot**: A testbot with four omni wheels, allowing for omnidirectional movement and versatile control.
- **ROS Noetic Integration**: Leverages ROS Noetic for modular and scalable robot software development.

## Hardware Requirements

- DC Motor with Encoder
- IMU Sensor (e.g., MPU6050)
- Microcontroller (e.g., Arduino, Raspberry Pi)
- Motor Driver
- Four Omni Wheels
- Chassis for the Testbot

## Software Requirements

- ROS Noetic
- Arduino IDE or PlatformIO
- Python (for Kalman filter implementation)
- Any required ROS packages (specified in the code)
- Any required libraries (specified in the code)

## Installation

1. **Clone the repository**:
    ```sh
    git clone https://github.com/yourusername/dc-motor-encoder-pid-imu-kalman.git
    cd dc-motor-encoder-pid-imu-kalman
    ```

2. **Install necessary libraries**:
    For Arduino:
    - PID Library
    - Encoder Library
    - Wire Library

    For Python (if applicable):
    ```sh
    pip install numpy
    ```

    For ROS Noetic:
    ```sh
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    ```

3. **Build the ROS workspace**:
    ```sh
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

4. **Upload the code to the microcontroller**:
    Open the Arduino IDE or PlatformIO, and upload the provided code to your microcontroller.

## Usage

1. **Connect the hardware**:
    - Connect the DC motor with the encoder to the motor driver and microcontroller.
    - Connect the IMU sensor to the microcontroller.
    - Assemble the omni wheel testbot.

2. **Run the control code**:
    - Power on the microcontroller and motor driver.
    - Ensure the IMU sensor is correctly initialized and calibrated.
    - Execute the control code to start the motor control and data processing.

3. **Monitor and adjust**:
    - Observe the motor's performance and adjust the PID parameters if necessary.
    - Monitor the IMU data and Kalman filter output for stability.

4. **Run ROS nodes**:
    - Launch the ROS nodes for motor control and IMU data processing.
    ```sh
    roslaunch your_package_name your_launch_file.launch
    ```

## PID Control

PID (Proportional-Integral-Derivative) control is used to maintain the desired motor position by adjusting the motor's speed based on the error between the desired and actual positions. The encoder provides feedback on the motor's position, which is used by the PID controller to make precise adjustments.

## Kalman Filter

The Kalman filter is an algorithm that processes noisy sensor data to produce more accurate and stable estimates. In this project, it is applied to the raw data from the IMU to improve the quality of the measurements used for motor control and testbot navigation.

## Omni Wheel Testbot

The testbot is equipped with four omni wheeestbot navigation.
Omni Wheel Testbot

ls, each mounted perpendicularly to the adjacent wheels. This configuration allows the testbot to move in any direction with high precision and agility, making it ideal for testing the motor control and sensor processing algorithms.

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.





Feel free to reach out if you have any questions or need further assistance. Happy coding!
