# prl_ur5_run
This package is the starting point for controlling the PRL UR5 robot.

## Start robot

To start real robot control use:

```
roslaunch prl_ur5_run real.launch [setup:=standart_setup] [sensors:=false] [moveit:=true] [rviz:=true] [pipeline:=ompl] [debug:=false]
```

To run simulation in Gazebo use:

```
roslaunch prl_ur5_run sim.launch [setup:=standart_setup] [sensors:=false] [gazebo_gui:=true] [moveit:=true] [rviz:=true] [pipeline:=ompl] [debug:=false]
```

To run MoveIt's "demo" mode use:

```
roslaunch prl_ur5_run demo.launch [setup:=standart_setup] [rviz:=true] [pipeline:=ompl] [debug:=false]
```

Possible arguments:

- _setup_: robot setup (details [here](../prl_ur5_description/README.md)), default="standart_setup"
- _sensors_: start sensors or not, default="false"
- _moveit_: start MoveIt or not, default="true"
- _rviz_: open RViz window or not, default="true"
- _pipeline_: MoveIt planning pipeline, default="ompl"
- _gazebo_gui_: open Gazebo GUI window or not, default="true"
- _debug_: run in the debug mode, default="false"