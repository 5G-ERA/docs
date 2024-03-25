# Onboarding of a webots simulated robot into the 5G-ERA Middleware - Turtlebot 3 Burger

In this turotial, we will onboard a simulated Webots ROS robot into the 5G-ERA Middleware. This tutorial has been tested in ROS 2 Foxy.
This tutorial covers the ROS foxy installation and the webots install and configuration with ROS2.

## 1) ROS Foxy installation

The operating system to be use for this tutorial needs to be Ubuntu 20.04.6. Let's follow the oficial ROS [tutorial](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) to install FOXY

You may run the following example to check if ROS was installed correctely.
```
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

## 2) Install webots

Let's follow the official [tutorial](https://cyberbotics.com/doc/guide/installation-procedure): 

If you open up a terminal and run webots an empty simulation should start.
```
webots
```

## 3) Install webots ROS2:

Let's follow the official [turotial](https://github.com/cyberbotics/webots_ros2/wiki/Linux-Installation-Guide)

## 4) Launch the turtlebot3 simulation:

```
ros2 launch webots_ros2_turtlebot robot_launch.py
```

For teleoperation node:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Check the official [tutorial](https://github.com/cyberbotics/webots_ros2/wiki/Example-TIAGo) for more references and other possible robot simulations. Tiago is also available.


## 5) Launch the utility script to complete the onboarding of turtlebot3.

In a terminal with ros and the workspace sourced, run the utility script to capture the data and generate the onboarding json file.

```
python3 onboarding_robot.py
```

## 6) Finish the template manually:

Copy from the newly created json file in section 5 the section rosNodes and paste it in a local copy of the [robot template](../User/Onboarding/robot_Onboarding_Template.json)

To add sensors, follow this json structure:
```
    {
      "name": "cameraSensor",
      "type": "Camera",
      "description": "RGDB camera sensor",
      "nodes": [
        "/camera_node"
      ],
      "number": 1
    }
```
To add an actuator:
```
   {
      "name": "RightEyebrow",
      "type": "Rotatory",
      "number": 1,
      "nodes": [
        "/actions_node"
      ]
    } 
```
To add a manipulator:
    
```
   {
      "name": "manipulator1",
      "type": "Rotatory",
      "number": 1,
      "nodes": [
        "/actions_node"
      ]
    } 
```

