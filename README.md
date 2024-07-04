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


## Installing ROS Noetic

In this section, we provide a step-by-step guide to install ROS Noetic on Ubuntu 20.04. ROS (Robot Operating System) is a flexible framework for writing robot software, and ROS Noetic is the latest LTS (Long Term Support) release.

### Overview

ROS Noetic provides tools and libraries for building and simulating robots. This installation guide ensures that you have a working ROS Noetic environment on your Ubuntu 20.04 system, which is essential for developing and testing robotic applications in the Space Station project.

### Prerequisites

- A computer running Ubuntu 20.04.
- Basic knowledge of using the terminal.

### Steps to Install ROS Noetic

1. **Setup Your Sources.list:**
   - Open a terminal and configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."
   ```sh
   sudo add-apt-repository restricted
   sudo add-apt-repository universe
   sudo add-apt-repository multiverse
   ```

2. **Setup Your Keys:**
   - Add the ROS package repository to your sources list.
   ```sh
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-noetic.list'
   ```
   - Set up your keys.
   ```sh
   sudo apt update
   sudo apt install curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

3. **Installation:**
   - Update your package list.
   ```sh
   sudo apt update
   ```
   - Install ROS Noetic Desktop-Full.
   ```sh
   sudo apt install ros-noetic-desktop-full
   ```

4. **Environment Setup:**
   - Add the ROS environment variables to your bash session.
   ```sh
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

5. **Dependencies for Building Packages:**
   - Install dependencies for building ROS packages.
   ```sh
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

6. **Initialize rosdep:**
   - Initialize `rosdep` to easily install system dependencies for source you want to compile and is required by most ROS packages.
   ```sh
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```

### Verifying the Installation

1. **Check the ROS Installation:**
   - Ensure that the installation was successful by running the following command:
   ```sh
   roscore
   ```
   - This should start the ROS master. If you see messages about the ROS master running, then the installation was successful.

2. **Create and Build a Catkin Workspace:**
   - Create a catkin workspace and build a sample package to verify the setup.
   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   source devel/setup.bash
   ```

3. **Run a Sample Node:**
   - Test your installation by running a simple ROS node.
   ```sh
   rosrun turtlesim turtlesim_node
   ```

### Conclusion

By following these steps, you will have a fully functional ROS Noetic installation on your Ubuntu 20.04 system. This setup is essential for developing and testing the robotic applications in the Space Station project. For additional details and troubleshooting, refer to the [official ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu


## 4 Omni Wheel Testbot

In this section, we introduce the 4 Omni Wheel Testbot designed and built for the Space Station project. The Testbot is a versatile robotic platform capable of omnidirectional movement, which is crucial for testing and validating control algorithms and integration techniques.

### Overview

The 4 Omni Wheel Testbot is equipped with four omni wheels, each independently powered and controlled. This design allows the Testbot to move in any direction and rotate freely, making it ideal for applications requiring precise maneuverability and agility.

### Features

- **Omni Wheels:** Four omni-directional wheels allow movement in any direction without changing orientation.
- **Independent Control:** Each wheel is independently controlled, enabling complex movement patterns.
- **Payload Capacity:** Designed to carry various payloads for testing sensors, actuators, and other components.
- **Modular Design:** Facilitates easy customization and integration of additional sensors and hardware components.

### Components

- **Omni Wheels:** Specialized wheels with rollers set at 45-degree angles to the wheel's axis.
- **Motors:** Each wheel is driven by a dedicated motor for independent control.
- **Microcontroller:** Controls motor drivers and interfaces with external systems.
- **Frame:** Provides structural support and mounting points for components.

### Usage

The Testbot serves as a platform for testing and developing robotics applications, including:

- **Control Algorithms:** Implementing and fine-tuning algorithms for navigation and obstacle avoidance.
- **Sensor Integration:** Testing and calibrating sensors such as IMUs, cameras, and distance sensors.
- **Software Development:** Writing and debugging software for autonomous navigation and task execution.

### Repository Structure

The repository for the 4 Omni Wheel Testbot contains:
- **CAD Designs:** CAD files for the mechanical design of the Testbot.
- **Control Software:** Example code and libraries for controlling the Testbot's movement.
- **Documentation:** Detailed documentation on assembly, operation, and maintenance.

### Getting Started

To get started with the 4 Omni Wheel Testbot, follow these steps:

1. **Clone the Repository:**
   ```sh
   git clone https://github.com/Shivansh-gupta2005/Space_Station_snt-.git
   cd space-station/
   ls
   wheel1.cpp wheel2.cpp wheel3.cpp wheel4.cpp
   ```

2. **Setup and Assembly:**
   - Follow the assembly instructions provided in the `Documentation` directory.
   - Ensure all components are securely mounted and connected according to the design specifications.

3. **Upload Control Software:**
   - Upload the provided control software to the microcontroller onboard the Testbot.
   - Adjust configurations as needed for your specific setup.

4. **Testing and Validation:**
   - Conduct initial tests to ensure all wheels respond correctly to commands.
   - Test different movement patterns and verify the Testbot's omnidirectional capabilities.

### Contributing

We welcome contributions to the 4 Omni Wheel Testbot project. Feel free to submit issues, feature requests, or pull requests to help improve and expand the functionality of the Testbot platform.

### License

This project is licensed under the MIT License. See the LICENSE file for more details.

### Conclusion

The 4 Omni Wheel Testbot is a powerful tool for developing and testing robotics applications within the Space Station project. Its omnidirectional movement capabilities and modular design make it suitable for a wide range of experimental and educational purposes in robotics. For further details and updates, please refer to the [repository](https://github.com/Shivansh-gupta2005/Space_Station_snt-.git)

## Integrating Arduino IDE and ROS

In this section, we outline the process of integrating Arduino IDE with ROS (Robot Operating System) for controlling hardware components and exchanging data between the two environments. This integration is crucial for developing robotic systems that require real-time control and sensor data processing.

### Overview

ROS provides a flexible framework for robotics software development, while Arduino IDE is commonly used for programming microcontrollers that interface with sensors, actuators, and other hardware components. Integrating these two environments allows for seamless communication and control, leveraging the strengths of both platforms.

### Components

- **Arduino Microcontroller:** Used to interface with sensors and actuators.
- **ROS (Robot Operating System):** Provides middleware and tools for robot software development.
- **ROS Serial:** ROS package for serial communication between ROS and Arduino.

### Steps to Integrate Arduino IDE and ROS

1. **Install Arduino IDE:**
   - Download and install Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software).
   - Install necessary drivers for your Arduino board.

2. **Install ROS (if not already installed):**
   - Follow the steps outlined in the [Installing ROS Noetic README.md](#installing-ros-noetic) to install ROS Noetic on your Ubuntu 20.04 system.

3. **Install ROS Serial:**
   - ROS Serial enables communication between ROS and Arduino over serial port.
   - Install ROS Serial using the following command:
     ```sh
     sudo apt install ros-noetic-rosserial-arduino
     sudo apt install ros-noetic-rosserial
     ```

4. **Set Up Arduino Libraries:**
   - Open Arduino IDE and install the ROS libraries for Arduino:
     - Go to Sketch > Include Library > Manage Libraries.
     - Search for and install `ros_lib` library.

5. **Upload ROS Arduino Sketch:**
   - Create or open an Arduino sketch (.ino file) that communicates with ROS.
   - Include necessary headers and define ROS communication objects.
   - Upload the sketch to your Arduino board.

6. **Configure ROS Workspace:**
   - Create a ROS workspace if you haven't already:
     ```sh
     mkdir -p ~/catkin_ws/src
     cd ~/catkin_ws/src
     catkin_init_workspace
     cd ~/catkin_ws
     catkin_make
     source devel/setup.bash
     ```
   
7. **Run ROS Serial Node:**
   - Launch the `rosserial_python` node to establish communication between ROS and Arduino:
     ```sh
     roscore  # Start ROS master if not already running
     rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
     ```
   - Replace `/dev/ttyUSB0` with the correct serial port of your Arduino board.

8. **Verify Communication:**
   - Test communication by publishing and subscribing to topics between ROS and Arduino.
   - Use ROS tools (`rostopic`, `rqt`, etc.) to monitor and interact with data exchanged between the systems.

### Example Workflow

1. **Arduino Sketch:**
   - Implement code to read sensor data (e.g., from an IMU or distance sensor) and publish it as ROS messages.
   - Subscribe to ROS topics to receive commands for controlling actuators (e.g., motors or LEDs).

2. **ROS Nodes:**
   - Write ROS nodes to process sensor data, implement control algorithms, and interact with other components of your robotic system.

### Conclusion

Integrating Arduino IDE with ROS enables robust control and communication capabilities in robotic applications. By following these steps, you can effectively integrate Arduino-based hardware with ROS for developing advanced robotics projects, such as those within the Space Station initiative. For further details and updates, please refer to the [official ROS documentation](http://wiki.ros.org/rosserial).


## Controlling Velocity of 4 Wheels Based on IMU Data

In this section, we describe the implementation of the `velocity_distributor_node` responsible for controlling the velocity of the 4-wheel TESTBOT based on IMU data. This node plays a crucial role in interpreting sensor information and distributing velocity commands to achieve desired movement and stability.

### Overview

The `velocity_distributor_node` utilizes data from the IMU (Inertial Measurement Unit) to calculate and distribute velocity commands to each wheel of the TESTBOT. This approach ensures coordinated movement and responsiveness, essential for navigating and maintaining stability in various environments.

### Components

- **IMU Sensor:** Provides data on acceleration and angular velocity.
- **ROS (Robot Operating System):** Middleware for robotics software development.
- **ROS Messages:** Custom messages for transmitting velocity commands.
- **Control Algorithms:** Implemented to interpret IMU data and calculate wheel velocities.

### Implementation Details

1. **IMU Data Processing:**
   - Subscribe to IMU data messages (`sensor_msgs/Imu`) published on a ROS topic.
   - Extract relevant data such as linear acceleration and angular velocity.

2. **Velocity Calculation:**
   - Use control algorithms (e.g., PID controllers) to interpret IMU data and calculate desired velocities for each wheel.
   - Adjust velocity distribution based on robot orientation and movement goals.

3. **Publish Velocity Commands:**
   - Publish velocity commands (`geometry_msgs/Twist`) to individual wheel controllers or motor drivers.
   - Ensure synchronization and coordination between all four wheels for smooth movement.

### Example Workflow

- **Launch `velocity_distributor_node.py`:**
  ```sh
  rosrun Space_station_snt velocity_distributor_node
  ```

- **Subscribe to IMU Data:**
  - Implement a subscriber to receive IMU data from the sensor topic.
  - Process and interpret IMU data to extract orientation and velocity information.

- **Calculate Wheel Velocities:**
  - Implement algorithms to convert IMU data into velocity commands.
  - Consider factors like robot orientation, environmental conditions, and desired movement behavior.

- **Publish Velocity Commands:**
  - Publish calculated velocity commands to respective wheel controllers using ROS topics.
  - Ensure real-time communication and responsiveness to maintain stability and accuracy.

### Testing and Validation

- **Simulation and Real-World Testing:**
  - Conduct simulations to validate velocity control algorithms and behavior under different scenarios.
  - Perform real-world testing to verify performance and fine-tune parameters if necessary.

### Conclusion

The `velocity_distributor_node` is essential for controlling the velocity of the 4-wheel TESTBOT based on IMU data, ensuring precise and stable movement in the Space Station project. By implementing this node and integrating it with ROS, you can effectively harness sensor data to achieve desired robotic behaviors and navigation capabilities. For further details and updates, please refer to your project's documentation and the [official ROS documentation](http://wiki.ros.org/ROS/StartGuide).
