
# Virtual Machine Setup 

- Install the [VMware workstation](https://www.vmware.com/content/vmware/vmware-published-sites/us/products/workstation-player/workstation-player-evaluation.html.html)

NOTE : If you are using a linux machine, you can directly proceed with the ROS noetic installation
## Set up Ubuntu 20.04 for ROS Noetic


- Installing the. iso file for [Ubuntu 20.04](https://releases.ubuntu.com/focal/)

# ROS Noetic installation

- Follow the instructions on the [ROS Noetic installation page](http://wiki.ros.org/noetic/Installation) to install ROS Noetic on your system or follow the steps below:

1. Setup your computer to accept software from packages.ros.org.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
2. Set up your keys
```bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
3. Installation

```bash
ROS_DISTRO := noetic
sudo apt update
sudo apt install ros-{ROS_DISTRO}-desktop-full
```

4. Environment setup
```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

5. Dependencies for building packages
- To install this tool and other dependencies for building ROS packages, run:

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
- Initialize rosdep
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```