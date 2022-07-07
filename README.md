# prl_ur5_robot
ROS stack for the bimanual UR5 robot

## Components

- [prl_ur5_run](prl_ur5_run/README.md): the starting point.
- [prl_ur5_description](prl_ur5_description/README.md): robot's URDF description.
- [prl_ur5_control](prl_ur5_control/README.md): physical robot control.
- [prl_ur5_gazebo](prl_ur5_gazebo/README.md): simulated robot control.
- [prl_ur5_moveit_config](prl_ur5_moveit_config/README.md): [MoveIt!](https://ros-planning.github.io/moveit_tutorials/) configuration.
- [prl_ur5_demos](prl_ur5_demos/README.md): demo scripts.

## Start robot

To start real robot control use:

```
roslaunch prl_ur5_run real.launch [setup:=standart_setup] [sensors:=false] [moveit:=true] [rviz:=true] [pipeline:=ompl] [debug:=false]
```

To run simulation in Gazebo use:

```
roslaunch prl_ur5_run sim.launch [setup:=standart_setup] [gazebo_gui:=true] [moveit:=true] [rviz:=true] [pipeline:=ompl] [debug:=false]
```

To run MoveIt's "demo" mode use:

```
roslaunch prl_ur5_run demo.launch [setup:=standart_setup] [rviz:=true] [pipeline:=ompl] [debug:=false]
```

Possible arguments:

- _setup_: robot setup (details [here](prl_ur5_description/README.md)), default="standart_setup"
- _sensors_: start sensors or not, default="false"
- _moveit_: start MoveIt or not, default="true"
- _rviz_: open RViz window or not, default="true"
- _pipeline_: MoveIt planning pipeline, default="ompl"
- _gazebo_gui_: open Gazebo GUI window or not, default="true"
- _debug_: run in the debug mode, default="false"

## Run demos

See instructions [here](prl_ur5_demos/README.md).

## Using with docker

See instructions [here](https://github.com/inria-paris-robotic-lab/prl_containers).

## Install from sources

Install Noetic ROS version (Desktop-Full Install recomended):
[instructions here](http://wiki.ros.org/noetic/Installation/Ubuntu).

Make sure that ROS is correctly sourced:

```
source /opt/ros/noetic/setup.bash
```

Install [wstool](http://wiki.ros.org/wstool) and [rosdep](http://wiki.ros.org/rosdep):

```
sudo apt-get install python3-wstool python3-rosdep
```

Make a folder for catkin workspace:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```

Clone the repo and install dependencies:

```
git clone -b master https://github.com/inria-paris-robotic-lab/prl_ur5_robot src/prl_ur5_robot
wstool init src ./src/prl_ur5_robot/prl_ur5_robot.rosinstall
rosdep update
rosdep install --from-paths src --ignore-src --skip-keys=python-pymodbus -r -y
```

Get the robot configuration:
* **Option 1** :
Clone the repo in your catkin workspace:
```
git clone -b master https://github.com/inria-paris-robotic-lab/prl_ur5_robot_configuration src/prl_ur5_robot_configuration
```
* **Option 2** :
Create a symbolic link to an already existing configuration folder:
```
ln -s /path/to/existing/folder src/prl_ur5_robot_configuration
```

Init and build the workspace:

```
catkin config --init --extend /opt/ros/${ROS_DISTRO}/
catkin config --blacklist robotiq_3f_gripper_articulated_gazebo_plugins
catkin build
```

Add a link to setup to bashrc:

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

Set environment variables (for log files):

```
echo "export PRL_LOG_PATH=/my/log/path" >> ~/.bashrc
```
