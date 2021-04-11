# prl_ur5_description
This package provides a description of the UR5 bimanual robot from the Inria Paris robotics laboratory.

## Setup

For different experiments, the robot can be equipped with different grippers and sensors.
To simplify the modification of the description, a changeable parameters are placed in a separate setup files in the **config/** directory.
Each setup file contains a list of used devices and calibration parameters.
The calibration parameters contain the positions of the arms, grippers, sensors, and kinematic parameters of UR5 arms.
UR5 factory calibrations were obtained using the [ur_calibration](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_calibration) package.

Available setups:

- `standart_setup`: setup with OnRobot gripper and RealSense camera on each arm.

![standart_setup](https://github.com/inria-paris-robotic-lab/prl_ur5_robot/blob/master/prl_ur5_description/media/standart_setup.png?raw=true "standart_setup")

## Launch files

To upload the description to the parameter server use:
```
roslaunch prl_ur5_description upload.launch setup:=standart_setup
```

To debug the description in rviz use:
```
roslaunch prl_ur5_description test.launch setup:=standart_setup
```

## URDF

Xacro URDF macros located in the **urdf/** directory.

To generate URDF file for the robot with a setup **setup_name** use:

```
xacro `rospack find prl_ur5_description`/urdf/prl_ur5_robot.urdf.xacro [setup:=setup_name] > prl_ur5_robot.urdf
```