<div align="center">
  <img src="resources/images/safevisionary2.png" alt="Sick safeVisionary2" style="width: 30%;"/>
  <h1 align="center">Sick Safevisionary ROS</h1>
</div>

<p align="center">
  <a href="https://opensource.org/licenses/Apache-2.0">
    <img src="https://img.shields.io/badge/License-Apache_2.0-yellow.svg" alt="License">
  </a>
  <a href="https://github.com/SICKAG/sick_safevisionary_ros/actions">
    <img src="https://github.com/SICKAG/sick_safevisionary_ros1/actions/workflows/industrial_ci_noetic_action.yml/badge.svg" alt="build badge">
  </a>
</p>

This is the official ROS driver for the [Sick safeVisionary2](https://www.sick.com/de/en/safety-camera-sensors/safety-camera-sensors/safevisionary2/c/g568562) cameras.

## System dependencies
We use *Boost*'s [lock-free](https://www.boost.org/doc/libs/1_82_0/doc/html/lockfree.html) data structures in this driver.
You can install them with
```bash
sudo apt-get install libboost-all-dev
```

## Build and install

You can either setup a fresh ROS workspace, for instance with
```bash
mkdir -p $HOME/sick_ws/src && cd "$_"
catkin_init_workspace
```
or use an existing one.
Inside the `src` folder of that ROS workspace, get the relevant ROS packages

```bash
git clone https://github.com/SICKAG/sick_safevisionary_ros1.git
git clone https://github.com/SICKAG/sick_safevisionary_base.git
rosdep install --ignore-src --from-paths ./ -y -r
```

We use `catkin build` for building the workspace. That's more convenient when working with the `sick_safevisionary_base` library, which is a non-catkin package and would else require `catkin_make_isolated`.
`catkin` is part of the `catkin_tools` package. You can install it e.g. with
```bash
sudo apt-get install python3-catkin-tools
```
[Here's](https://catkin-tools.readthedocs.io/en/latest/installing.html) more information about this install.

You can then build everything in the root of the ROS workspace with

```bash
cd ..
catkin config --install
catkin build
```
If you used an existing workspace for the new Sick components, you might need
to remove the `build` folder first so that you don't mix build spaces with
previous calls to `catkin_make`.


## Getting started
Source your local `install/setup.bash` and run
```bash
roslaunch sick_safevisionary_driver driver_node.launch
```
You can list the relevant topics with
```bash
rostopic list | grep sick_safevisionary
```
