# Supply-Bot
Modelling of a Autonomous Warehouse Robot.
Here's a refined README format that enhances the layout and clarity of your instructions, suitable for a GitHub project. This format includes better markdown organization and clear step-by-step instructions to improve the readability and usability of your document:

```markdown
# Project Title

## Contents
- [1. Assembly](#1-assembly)
- [2. Package Dependencies](#2-package-dependencies)
- [3. How to Run the Code](#3-how-to-run-the-code)

## 1. Assembly

Download the CAD file.

## 2. Package Dependencies

Ensure you have the following dependencies installed:

- **Python**: Version 3.9 (compatible with any Python 3.x version)
- **IDE**: PyCharm IDE for programming and executing the code, and Jupyter.
- **Libraries**:
  - numpy
  - matplotlib.pyplot
  - Sympy
  - math

## 3. How to Run the Code

### Setup Workspace

1. Create a `catkin_ws` and build it, then source it.
2. Download the package.
3. Paste the package in the source directory of `catkin_ws`.
4. Build and source the workspace again:

    ```bash
    cd catkin_ws
    catkin_make
    source devel/setup.bash
    ```

### Launch the Program

#### Launch ROS Nodes

Open a new terminal and run:

```bash
roslaunch supplybot template_launch.launch
```

#### Control Mobile Base

Open a new terminal (Ctrl+Shift+T) and run:

```bash
cd src/scripts/
rosrun supplybot teleop_template.py
```

#### Manipulate the Robot

Open another terminal (Ctrl+Shift+T) for manipulator controls:

- To bring the robot to the home position:
  ```bash
  rosrun supplybot manipulator_idle.py
  ```
- For manual control of the robot:
  ```bash
  rosrun supplybot teleop_template.py
  ```
- To enable forward motion:
  ```bash
  rosrun supplybot publisher.py
  ```
- To turn the vacuum gripper on:
  ```bash
  rosservice call /robot/left_vacuum_gripper/on
  ```
- To initiate a pick action:
  ```bash
  rosrun supplybot manipulator_pick.py
  ```
- To initiate a place action:
  ```bash
  rosrun supplybot manipulator_place.py
  ```
- To turn the vacuum gripper off:
  ```bash
  rosservice call /robot/left_vacuum_gripper/off
  ```

#### Visualize in RViz

Open a new terminal (Ctrl+Shift+T) and run:

```bash
rviz
```



