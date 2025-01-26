# Sequential Motion YAML Creation GUI
A GUI and ROS2 Node for Configuring and Executing Sequential Robot Movements

## Overview
This project provides a simple yet powerful tool to define, execute, and visualize sequential movements for a robot in a ROS2 environment. It includes:

1. A YAML Configuration GUI to easily create YAML files for robot motion planning.
2. A ROS2 Node that reads the YAML files, executes the movements, and visualizes them in RViz.

## Features
1. YAML Configuration GUI (yaml_gui.py):  
  - User-friendly interface built with Tkinter.
  - Create, save, and edit YAML files that define:
    -Rotational movements (left or right).
    -Straight-line movements (forward or backward).
2. ROS2 Node (controller_node.py):
  - Reads YAML files and executes the defined movements.
  - Publishes cmd_vel messages to control the robot.
  - Visualizes the robot's position and orientation in RViz.
  - Real-time updates using tf transformations.
3. Visualization in RViz:
  -Watch the robot's motion and verify the correctness of the planned sequence.

## System Components
1. YAML Configuration GUI
- Purpose: Create YAML files for movement planning.
- Capabilities:
  - Add rotational and straight-line movements.
  - Save the sequence as a YAML file for execution.
2. ROS2 Node
- Purpose: Execute movements and visualize them in RViz.
- Capabilities:
    - Read and parse YAML files (e.g., test_movement.yaml).
    - Execute:
        - Rotational movements: Rotate the robot by a specified angle at a defined speed.
        - Straight-line movements: Move forward or backward by a specified distance at a defined speed.
    - Publish transforms for RViz visualization.
    - Log execution details for verification.

## Installation
1. Clone the Repository:
```
$ git clone https://github.com/BojanAndonovski71/sequential-motion-yaml-creation-gui.git
$ cd sequential-motion-yaml-creation-gui
```

2. Build the ROS2 Workspace:
```
$ colcon build
$ source install/setup.bash
```

3. Install Dependencies:

Ensure ROS2 is installed on your system.
Install any missing Python dependencies for the GUI:
```
pip install tkinter pyyaml
```

## Usage
1. Generate a YAML File:
Run the YAML GUI:
```
ros2 run movement_controller yaml_gui
```
Define movements (rotations and straight-line motions) and save them as a YAML file.
2. Execute Movements:
Launch the movement controller and RViz:
bash
Copy
Edit
ros2 launch movement_controller movement_rviz.launch.py
3. Verify Execution:
Check robot motion in RViz.
Monitor cmd_vel messages:
bash
Copy
Edit
ros2 topic echo /cmd_vel
Publish static transforms for visualization:
bash
Copy
Edit
ros2 run tf2_ros static_transform_publisher 0.0 0 0. 0 0 0 map base_link
Example YAML Configuration
Here’s a sample YAML file for sequential movements:

yaml
Copy
Edit
- movement: rotation
  angle: 90    # Degrees
  speed: 0.5   # Angular velocity (rad/s)

- movement: straight
  distance: 1.0  # Meters
  speed: 0.2     # Linear velocity (m/s)
Testing
Step 1: Run the GUI to generate a YAML file.
Step 2: Launch the RViz visualization and movement controller node.
Step 3: Monitor:
Robot motion in RViz.
Logs and cmd_vel messages for verification.
Contributing
Contributions are welcome to improve the project! Here’s how to contribute:

Fork the repository.
Create a feature branch:
bash
Copy
Edit
git checkout -b feature/your-feature
Commit your changes:
bash
Copy
Edit
git commit -m "Add your feature"
Push to the branch and create a pull request.
License
This project is licensed under the MIT License. See the LICENSE file for details.

Contact
Author: Bojan Andonovski
GitHub: BojanAndonovski71
