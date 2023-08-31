# Intel RealSense Camera 435i

## Installation of the realsense2_camera

```bash
FRANKA_WS = path/desired/to/workspace
```

**1.** Clone the latest Intel® RealSense™ ROS from here into 'catkin_ws/src/'
```bash
source FRNAKA_WS/devel/setup.zsh
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
```

**2.** Make sure you have all the dependent packages installed and then build the workspace
   Specifically, make sure that the ros package ddynamic_reconfigure is installed. If ddynamic_reconfigure cannot be installed using -apt .

```bash
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin build  
```

## Virtual Machine ( Vmware Workstation)

**1.** Connect the RealSense camera.

**2.** Go to the VMware settings and choose **Removable Devices**, then click on the **Intel RealSense Camera** option.

**3.** Click on **Connect to the machine from host.**

**4.** Once that is done, VMware will enable the detection of your camera. To check this, open the terminal and type 
```bash
lsusb
```
 This will show you the list of connected devices on VMware.

**5.** Download **cheese** a camera app on Linux, to view and capture images.

**6.** If you see the RealSense feed on the Cheese app, it means the camera detection was successful on VMware.

**NOTE** : If a linux mahcine is being used, you can skip the above steps


## Using RealSense ROS Package

Follow these steps to use the RealSense ROS package:

**1.** **Check the firmware version** of the RealSense camera. Incorrect firmware may cause ROS packages to throw errors. Follow these steps to check the latest firmware:

### Method 1 : Using a GUI 

**1.** Install the [realsense viewer](https://www.intelrealsense.com/get-started-depth-camera/) and download the [Intel.RealSense.Viewer.exe](https://github.com/IntelRealSense/librealsense/releases/tag/v2.54.1). Follow the steps on the github repository.

### Method 2 : Using the command line

**1.** Follow the steps on https://dev.intelrealsense.com/docs/firmware-update-tool after installing the real sense ROS package / library .

run the command to see the list of the connected devices and to check the firmware version

```bash
rs-fw-update -l
```

**2.** To install the latest firmware version for the camera , visit https://dev.intelrealsense.com/docs/firmware-releases

To update the firmware using the command line: 

SERIAL NUMBER is the output of the above command
VERSION NUMBER is the latest firmware version to be updated to. It can be found on the website mentioned above.

```bash
rs-fw-update -s [SERIAL_NUMBER] -f Signed_Image_UVC_[VERSION_NUMBER].bin

EXAMPLE:
rs-fw-update -s 725112060411 -f Signed_Image_UVC_5_11_6_250.bin
```

   Ensure that the firmware is up to date before proceeding.

**3.** Once the firmware is updated, start the camera node in ROS using the following command:

```bash
roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720 color_fps:=30

```

**4.** Visualize the image on RViz:

   - Run the following command: `rosrun rviz rviz`
   - Click on `Add by Topic`
   - Under the `Image` section, select `raw`. You should be able to view a live feed from the RealSense camera.
    
**5.** Visualize image on rqt:  `rqt_image_viewer`
