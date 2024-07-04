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

## Dual Booting Windows and Ubuntu 20.04

In this section, we provide a step-by-step guide to set up a dual-boot system with Windows and Ubuntu 20.04. This setup allows you to leverage the benefits of both operating systems, facilitating development and testing for the Space Station project.

### Overview

Dual booting enables you to have both Windows and Ubuntu installed on the same computer, allowing you to choose which operating system to boot into. This setup is particularly useful for robotics projects where you might need the development tools available on both platforms.

### Prerequisites

- A computer with Windows installed.
- A USB drive with at least 8GB capacity.
- Ubuntu 20.04 ISO file (download from [ubuntu.com](https://ubuntu.com/download/desktop)).
- Backup of important data.

### Steps to Dual Boot Windows and Ubuntu 20.04

1. **Backup Your Data:**
   - Ensure you have backups of all important data before proceeding.

2. **Create a Bootable USB Drive:**
   - Download and install [Rufus](https://rufus.ie/) on Windows.
   - Use Rufus to create a bootable USB drive with the Ubuntu 20.04 ISO file.

3. **Create a Partition for Ubuntu:**
   - Open Disk Management in Windows.
   - Shrink the volume of your main partition to create free space for Ubuntu (at least 20GB recommended).
   - Leave the space unallocated.

4. **Disable Fast Startup in Windows:**
   - Go to Control Panel > Power Options > Choose what the power buttons do.
   - Click on "Change settings that are currently unavailable."
   - Uncheck "Turn on fast startup" and save changes.

5. **Boot from the USB Drive:**
   - Restart your computer and boot from the USB drive (you may need to change the boot order in BIOS/UEFI settings).

6. **Install Ubuntu 20.04:**
   - Select "Install Ubuntu" when prompted.
   - Follow the installation steps. When asked about installation type, select "Something else."
   - Choose the unallocated space you created and set up the following partitions:
     - **Root (`/`)**: At least 15GB, ext4 filesystem.
     - **Swap**: Optional, typically 1-2 times your RAM size.
     - **Home (`/home`)**: Remaining space, ext4 filesystem (optional).
   - Proceed with the installation.

7. **Configure Bootloader:**
   - The installer will automatically detect Windows and configure the GRUB bootloader to allow selection between Windows and Ubuntu at startup.

8. **Complete Installation:**
   - Finish the installation and restart your computer.
   - You should see the GRUB menu, allowing you to choose between Windows and Ubuntu.

### Post-Installation Steps

1. **Update Ubuntu:**
   - After logging into Ubuntu, open a terminal and run:
     ```sh
     sudo apt update
     sudo apt upgrade
     ```

2. **Install Essential Software:**
   - Install any necessary development tools and libraries required for your project.

### Troubleshooting

- **Bootloader Issues:** If you don't see the GRUB menu, you may need to repair the bootloader using a live USB session.
- **Partitioning Errors:** Double-check the partition sizes and filesystems during the installation process.

### Conclusion

Dual booting Windows and Ubuntu 20.04 provides a flexible development environment, allowing you to leverage the strengths of both operating systems. By following this guide, you can set up a dual-boot system to facilitate your work on the Space Station project. For additional details and troubleshooting, please refer to the [official Ubuntu documentation](https://help.ubuntu.com/community/WindowsDualBoot).


