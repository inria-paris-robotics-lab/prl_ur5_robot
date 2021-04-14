# prl_ur5_robot
ROS stack for the bimanual UR5 robot

## Components

- [prl_ur5_description](prl_ur5_description/README.md): robot's URDF description.
- [prl_ur5_control](prl_ur5_control/README.md): physical robot control.
- [prl_ur5_gazebo](prl_ur5_gazebo/README.md): simulated robot control.
- [prl_ur5_moveit_config](prl_ur5_moveit_config/README.md): [MoveIt!](https://ros-planning.github.io/moveit_tutorials/) configuration.
- [prl_ur5_demos](prl_ur5_demos/README.md): Demo scripts.

## Using with docker

See instructions [here](https://github.com/inria-paris-robotic-lab/prl_containers).

## Install from sources

Install Noetic ROS version (Desktop-Full Install recomended):
[instructions here](http://wiki.ros.org/noetic/Installation/Ubuntu).

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
rosdep install --from-paths src --ignore-src --skip-keys=python-pymodbus -r -y
```

Init and build the workspace:

```
catkin config --init --extend /opt/ros/${ROS_DISTRO}/
catkin config --blacklist robotiq_3f_gripper_articulated_gazebo_plugins
catkin build
```

Add a link to setup to bashrc:

```
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
source /catkin_ws/devel/setup.bash
```