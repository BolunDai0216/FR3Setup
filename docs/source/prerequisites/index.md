# Prerequisites

After setting up the network and activating the FCI interface as described here, we need to install the following items for using the robot via ROS:

- Compile and install the appropriate version of `libfranka`
- Create a catkin workspace and compile `franka_ros`
- Adding [MoveIt](https://ros-planning.github.io/moveit_tutorials/) to the created catkin workspace

Throughout this tutorial, we assume the `FRANKA_WS` environment variable has been set to our working directory: 

```bash
export FRANKA_WS=/path/to/desired/workspace
```

## Compile and Install libfranka

First, in order to make sure everything is clean, remove all previously installed versions of `libfranka`:

```bash
sudo apt remove "*libfranka*"
```

Then install the required tools and libraries:

```bash 
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
```

Then clone the appropriate version of `libfranka`:

**For Panda**

```bash
cd ${FRANKA_WS}
git clone --recursive https://github.com/frankaemika/libfranka
```

**For FR3:**

```bash
cd ${FRANKA_WS}
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
```

Then compile the library as follows:

```bash
cd ${FRANKA_WS}/libfranka
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
cpack -G DEB
```

finally, install the library:

```bash
cpack -G DEB
sudo dpkg -i libfranka*.deb
```

## Install franka_ros

We assume that ROS Noetic has already been installed. If not, follow through the steps [here](http://wiki.ros.org/noetic/Installation/Ubuntu) to install it. Then, we need to add a catkin workspace:

```bash
cd ${FRANKA_WS}
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.sh
catkin_init_workspace src
git clone --recursive https://github.com/frankaemika/src/franka_ros
```
By default, this will check out the newest release of `franka_ros`. If you want to build a particular version of `franka_ros` instead, check out the corresponding git tag:

```bash
git checkout <version>
```

Then install any missing dependencies and build the package:

```bash
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys libfranka
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=${FRANKA_WS}/libfranka/build
catkin build
source devel/setup.bash
```

After installation, unlock the robot, enable FCI, and run the following for a simple test of the framework. The `robot_ip` should be changed to the IP address of the particular robot you are trying to control:

```bash
roslaunch franka_example_controllers move_to_start.launch robot_ip:=<robot_ip>
```

If successful, the robot moves to the home position and you get the following message:

```bash
move_to_start: Successfully moved in to start pose
```

**Note**: you may need to manually move the robot away from the home position to visually see the motion of the robot.

## Install MoveIt (Optional)

Finally, for calibration purposes, we need to install `MoveIt`. First, install `catkin-tools`:

```bash
sudo apt install ros-${ROS_DISTRO}-catkin python3-catkin-tools
```

Then, Install `wstool`:

```bash
sudo apt install python3-wstool
```

Clone `MoveIt` into your workspace:

```bash
cd ${FRANKA_WS}/catkin_ws/src
wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials
wstool update -t .
git clone https://github.com/ros-planning/moveit_tutorials.git -b master
git clone https://github.com/ros-planning/panda_moveit_config.git -b ${ROS_DISTRO}-devel
```

Install the dependencies:

```bash
cd ${FRANKA_WS}/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}
```

Finally, compile the library:

```bash
cd ${FRANKA_WS}/catkin_ws
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=${FRANKA_WS}/libfranka/build
catkin build
```

Finally, to test the system, run the following with `robot_ip` set to the IP address of the robot you want to control:

```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=<robot_ip>
```

### Install MoveIt Calibration Toolbox

For eye-in-hand and eye-on-base calibration of the cameras used with your robotic setup, you would need to add the MoveIt calibration toolbox. First, clone the source code into your catkin workspace's `src` directory:

```bash
cd ${FRANKA_WS}/catkin_ws/src
git clone https://github.com/ros-planning/moveit_calibration.git
```

Then, make sure you have the appropriate dependencies and build the package:

```bash
cd ${FRANKA_WS}/catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin build
```
