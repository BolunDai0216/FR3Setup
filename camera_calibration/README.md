# Camera Calibration

The camera calibration process contains two types of calibration: eye-in-hand and eye-to-hand. 

**1.** Set the ethernet IP address to static IP in the same IP range the robot has been set to.( You can save this configuration as a profile and later choose this profie for easier access)

**2.** Unlock the robot

**3.** Activate FCI 

**4.** Check the python numpy version. Should use `1.19`
```bash
python -m pip install numpy==1.19
```

**5.** To run the moveit for the robot : 
```bash
source $FRANKA_WS/devel/setup.zsh
roslaunch panda_moveit_config franka_control.launch robot_ip:=10.42.0.3 
```


**6.** Run the realsense camera launch cmd , and visualize the images on Rviz: 
```bash
roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720 color_fps:=30
```

**7.** In RViz, add the `By display type > moveit_calibration_gui > HandEyeCalibration`

**8.** In the HandEyeCalibration , set the target parameters :

  **a.** Measure the number of markers in a grid in X by Y ( eg.2x2)

  **b.** Measure each marker size and the separation distance between each marker and set it in (m). (0.067 , 0.0067 m respectively)

**9.** Click on Create Target and Print it out.

**10.**Choose the Image topic and Camera info topic where the image data is being published under the Target Pose Detection

`image topic : /camera/color_image/raw
camera info topic : /camera/color/camera_info`

**11.** Under the Context tab , set the Sensor Configuration as the appropriate configuration you are performing. 

**12.** Select the correct Frames of the sensor, object, end-effector and Robot base frame.

`Sensor Frame : camera_color_optical_frame`

`ObjectL : handeye_target`

`End-effector frame :panda_hand`

`Robot base frame : panda_link0`

**13.** Set the robot in the "Programming Mode" to move the robot manually

**14.** In RViz, add  `By topic > /handEye_calibration > /target_detection > Image > raw`

**15.** Take around 15 Samples of different configurations under the Calibration tab in the HandEyeCalibration and click on "Take Sample" for each configuration by manually moving the robot.

**16.** Leaving the solver name same, click on "Save the camera pose" option. This will create a launch file of all the TFs of the calibrated Eye-in-hand transformations

**17.** Run this launch file.

**18.** In RViz, , Add TFs under the Add by Display



## Eye-in-Hand Calibration

The hand-in-eye calibration is used to find the transformation matrix between the camera frame and the end-effector frame of the robot. In this case, we assume that the camera is rigidly attached to the end-effector of the robot and the April tag is fixed relative to the base of the robot.

<figure style="text-align: center;">
  <img src="./imgs/eye-in-hand.png" alt="Eye-in-Hand Calibration" style="width:70%;"/>
  <figcaption>Eye-in-Hand Calibration Illustration (Credit to <a href="https://www.torsteinmyhre.name/snippets/robcam_calibration.html">Torstein A. Myhre</a>)</figcaption>
</figure>

The problem can be formulated as $\mathbf{AX} = \mathbf{XB}$, where $\mathbf{A}$ can be computed using forward kinematics, $\mathbf{B}$ can be computed using the measurements of the transformation matrix between the camera frame and the April tag frame. The goal is to find $\mathbf{X}$, which represents the transformation matrix between the end-effector frame and the camera frame. 



## Eye-to-Hand Calibration

The hand-to-eye calibration is used to find the transformation matrix between the camera frame and the base frame of the robot. In this case, we assume that the camera is fixed relative to the base of the robot.

<figure style="text-align: center;">
  <img src="./imgs/eye-to-hand.png" alt="Eye-to-Hand Calibration" style="width:70%;"/>
  <figcaption>Eye-to-Hand Calibration Illustration (Credit to <a href="https://www.torsteinmyhre.name/snippets/robcam_calibration.html">Torstein A. Myhre</a>)</figcaption>
</figure>

Similar to the eye-in-hand case, here the value of $\mathbf{A}$ and $\mathbf{B}$ can be obtained using forward kinematics and camera measurements, respectively. The goal is to find $\mathbf{X}$, which represents the transformation matrix between the end-effector (hand) frame and the checkerboard frame. Then, the transformation matrix between the camera frame and the base frame can be computed as

$$^\mathrm{camera}\mathbf{T}_\mathrm{base} = {}^\mathrm{camera}\mathbf{T}_\mathrm{board}{}^\mathrm{board}\mathbf{T}_\mathrm{hand}{}^\mathrm{hand}\mathbf{T}_\mathrm{base}$$

```bash
roslaunch gray_base_to_white_camera.launch
python3 generate_launch_file.py --parent "/camera_color_optical_frame" --child "/panda_link0" --filename "white_camera_to_gray_base" --parentName "camera_color_optical_frame" --childName "grey_panda_link0"
roslaunch white_hand_to_white_camera.launch
roslaunch white_camera_to_gray_base.launch
python3 generate_launch_file.py --parent "/grey_panda_link0" --child "/panda_link0" --filename "gray_base_to_white_base" --parentName "grey_panda_link0" --childName "white_panda_link0"
```
