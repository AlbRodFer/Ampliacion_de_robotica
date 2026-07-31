# cartesian_trajectory_planning_private

Public repo for the Cartesian trajectory planning from the [Advanced Robotics Course](https://jmgandarias.com/advanced_robotics/) from the University of Málaga.

This package was inspired by the [full tutorial for a 6 DOF robot](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html) from [ros2 control](https://control.ros.org/humble/doc/getting_started/getting_started.html).

The package is structured as follows

* bringup: launch files and ros2_controller configuration
* config/poses.yaml: YAML file with the poses to test the linear
* controller: a controller for the 6-DOF robot
* description: the 6-DOF robot description
* hardware: ros2_control hardware interface
* reference_generator: A KDL-based reference generator for a fixed trajectory

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html).

## Luanch trajectory generation demo

In one terminal, run:

```bash
ros2 launch cartesian_trajectory_planning r6bot_controller.launch.py
```

In another terminal, run:

```bash
ros2 launch cartesian_trajectory_planning send_trajectory.launch.py
```

## show EE trail in Rviz

* Go to RobotModel>Links>tool0 (or the link that refers to the EE).
* Habilitate Show Trail.
