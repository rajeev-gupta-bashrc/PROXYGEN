# Proxygen: ROS-based Robot for Package Handling

Proxygen is a robotic system developed using the MoveIt package in ROS Noetic. It is designed to handle package sorting and delivery tasks efficiently. Proxygen integrates with hardware components to perform tasks such as detecting delivery boxes using a depth camera, picking them up using a suction gripper, and placing them in designated areas.

![Mobile Manipulator Robot](https://github.com/rajeev-gupta-bashrc/PROXYGEN/images/blob/proxygen_hardware.jpg)

## Hardware Specifications

- **Robot:** 5 degrees of freedom (DOF) robotic arm
- **Gripper:** Suction gripper ([Link to Product](https://robu.in/product/15l-high-flow-555-vaccum-pump-air-pump-oxygen-pump-fish-tank-folding/))
- **Depth Camera:** Intel RealSense D445
- **Microcontroller:** Arduino Mega
- **Stepper Motors:** 
  - 2 x NEMA 17
  - 2 x NEMA 23 with additional gearboxes for joint actuation

## Functionality

Proxygen performs the following tasks:

1. **Visual Input Processing**: Utilizes the depth camera to capture visual data of the environment.

![Mobile Manipulator Robot](https://github.com/rajeev-gupta-bashrc/PROXYGEN/images/blob/image_tracking.jpg)

2. **Object Detection**: Detects delivery boxes or packages within the camera's field of view.
3. **Coordinate Transformations**: Manipulates the acquired depth data through coordinate transformations to calculate the 3D positions of the packages relative to the base frame of the robot.
4. **Motion Planning**: Utilizes MoveIt for motion planning to pick up and place the packages.
5. **Inverse Kinematics**: Calculates joint configurations using inverse kinematics to reach desired end-effector positions.

![Mobile Manipulator Robot](https://github.com/rajeev-gupta-bashrc/PROXYGEN/images/blob/proxygen_gazebo.jpg)

6. **Gripper Control**: Activates the suction gripper to pick up and release packages as required.
7. **Package Delivery**: Places the packages in designated areas according to predefined instructions.

## Launch Files and Scripts

To execute the package handling task, the following launch files and scripts are used:

1. **proxygen_controller.launch**: Launches the controller node for Proxygen, initializing the hardware interfaces and communication.
2. **proxygen_simulation.launch**: Launches the simulation environment for testing and debugging purposes.
3. **pick_and_place.py**: Python script responsible for coordinating the entire package handling task, including object detection, motion planning, and gripper control.

## Usage

To utilize Proxygen for package handling tasks, follow these steps:

1. Ensure all hardware components are properly connected and powered.
2. Launch the `proxygen_controller.launch` file to initialize the robot controller node.
3. If testing in a simulation environment, launch the `proxygen_simulation.launch` file.
4. Execute the `pick_and_place.py` script to begin the package handling task.
5. Monitor the robot's actions through ROS messages and visualization tools.
6. Once the task is completed, verify the successful delivery of packages to the designated areas.

With its integrated hardware and software components, Proxygen provides an efficient solution for automating package handling processes, optimizing efficiency and accuracy in logistics operations.

## Results:

![Mobile Manipulator Robot](https://github.com/rajeev-gupta-bashrc/PROXYGEN/images/blob/proxygen_hardware.gif)

![Mobile Manipulator Robot](https://github.com/rajeev-gupta-bashrc/PROXYGEN/images/blob/proxygen_gazebo.gif)
