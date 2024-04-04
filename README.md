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

In order to be able to start the robot (in sim or real), you need to setup some environment variable in your terminal:
```bash
source /opt/ros/noetic/setup.bash # ONLY IF YOU ARE USING ROS FROM APT
source ~/catkin_ws/devel/setup.bash # Make ros able to find packaged that you build manually (using catkin)

source ~/catkin_ws/src/prl_ur5_robot_configuration/script/setup_env.bash # To have the robot specific configurations
```

To start real robot control use:

```bash
roslaunch prl_ur5_run real.launch [setup:=standart_setup] [sensors:=false] [moveit:=true] [rviz:=true] [pipeline:=ompl] [debug:=false]
```

To run simulation in Gazebo use:

```bash
roslaunch prl_ur5_run sim.launch [setup:=standart_setup] [sensors:=false] [gazebo_gui:=true] [moveit:=true] [rviz:=true] [pipeline:=ompl] [debug:=false]
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

See instructions [here](https://github.com/inria-paris-robotics-lab/prl_containers).

## Install from sources
The following section contains instructions that can be specific to apt or conda.

1. Install Noetic ROS version (Desktop-Full Install recomended):
<details open><summary>Conda</summary>

```bash
conda create -n my_ros -c conda-forge -c robostack -c default ros-noetic-desktop python=3.8.* --no-default-packages
# For micromamba users, remove the "--no-default-packages" option
```

</details>
<details><summary>Apt</summary>

[instructions here](http://wiki.ros.org/noetic/Installation/Ubuntu).

</details>

2. Make sure that ROS is correctly sourced:
<details open><summary>Conda</summary>

```bash
conda activate my_ros
```

</details>
<details><summary>Apt</summary>

```bash
source /opt/ros/noetic/setup.bash
```

</details>

3. Install [wstool](http://wiki.ros.org/wstool) and [rosdep](http://wiki.ros.org/rosdep):

<details open><summary>Conda</summary>

```bash
conda install -c conda-forge catkin_tools wstool
```

</details>
<details><summary>Apt</summary>

```bash
sudo apt-get install python3-catkin-tools python3-wstool python3-rosdep
```

</details>


4. Make a folder for catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```

5. Clone the repo, (automatically) clone the depencies that needs to be installed from source and install the other dependencies:

```bash
git clone -b master https://github.com/inria-paris-robotics-lab/prl_ur5_robot src/prl_ur5_robot
wstool init src ./src/prl_ur5_robot/prl_ur5_robot.rosinstall
rosdep update
rosdep install --from-paths src --ignore-src --skip-keys=python-pymodbus -r -y
```

6. Get the robot configuration:
* **Option 1** :
Clone the repo in your catkin workspace:
```bash
git clone -b master https://github.com/inria-paris-robotics-lab/prl_ur5_robot_configuration src/prl_ur5_robot_configuration
```
* **Option 2** :
Create a symbolic link to an already existing configuration folder:
```bash
ln -s /path/to/existing/folder src/prl_ur5_robot_configuration
```

7. Init the workspace:

<details open><summary>Conda</summary>

```bash
catkin config --init --extend $CONDA_PREFIX
```

</details>
<details><summary>Apt</summary>

```bash
catkin config --init --extend /opt/ros/${ROS_DISTRO}/
```

</details>

```bash
catkin config --blacklist robotiq_3f_gripper_articulated_gazebo_plugins
```

8. Build everything
```bash
catkin build
```

9.  [optionnal - only for APT] Add a link to setup in your bashrc:

<details><summary>Apt</summary>

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

</details>

10. Setup local ros environment variables
```bash
source ~/catkin_ws/devel/setup.bash
```

11. Set custom environment variables (for log files):

```bash
echo "export PRL_LOG_PATH=/my/log/path" >> ~/.bashrc
```
