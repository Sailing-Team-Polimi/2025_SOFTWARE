# 2025 SuMoth Competition - Software

This repository contains the software developed by **Polimi Sailing Team** for the **2025 SuMoth competition**, which is designed to run on the **Raspberry Pi 5** computers aboard the main boat ("febe") and the side boat. 

The software is based on **ROS2 Jazzy** and provides various functionalities for the operation of the boats during the competition.

More details on the software developed can be found in the 2025 [Design Report](Meccatronica_DR_2025.pdf) of the Mechatronics department 


## Overview

The software is organized into the following ROS2 packages ofund in the `ros2_ws/src/` folder:

### Packages

| Package            | Description                                                                                       |
|--------------------|---------------------------------------------------------------------------------------------------|
| **`can_bus`**       | Provides communication with the CANbus system in NMEA2000 managed by an external microcontroller connected via UART. |
| **`flap_control`**  | Contains the control logic and rules for managing the servo connected to the flap of the main foil. |
| **`gps_interface`** | Handles communication with the GPS system through USB for accurate positioning data.             |
| **`gui`**           | Provides launch files and URDF descriptions to allow for simple GUI through **Foxglove** and **RViz2**. |
| **`orca`**          | The main package that includes general launch files for starting the boat's operations.           |
| **`poli_sail_gui`** | Contains the necessary programs to send and display data on a custom-made application for monitoring and controlling the boat. |
| **`sail_msgs`**     | Defines custom messages for incoming data from the CANbus system, ensuring proper parsing and usage. |
| **`sensor_estimates`** | Includes programs for post-processing raw sensor data to estimate the boat's position in space.  |
| **`state_estimation`** | Handles the fusion of sensor data, including all necessary launch files and programs for accurate state estimation. |

---

### Containerization

All software is containerized using **Docker** to ensure easy deployment and testing. The Docker setup allows the software to run on any computer, providing a consistent development and testing environment. Detailed setup instructions are available at the following [README](Docker/Ros_with_docker.md).

### Raspberry Pi 5 Setup

The setup instructions for configuring the **Raspberry Pi 5** are located in the `raspi_setup/` folder. This includes steps for installing necessary dependencies and configuring the Raspberry Pi environment to run the software.

---
