# Using MoveIt Calibration

To use MoveIt Calibration, to perform the hand-eye calibration, follow the steps below:

1. Set the ethernet IP address to static IP in the same IP range the robot has been set to (You can save this configuration as a profile and later choose this profie for easier access).

2. Unlock the robot.

3. Activate FCI.

4. Check the Python NumPy version. Should use `1.19`, if the NumPy version is not `1.19` then install it using:
    
    ```bash
    python -m pip install numpy==1.19
    ```

5. To run MoveIt for the robot:

    ```bash
    source $FRANKA_WS/devel/setup.zsh
    roslaunch panda_moveit_config franka_control.launch robot_ip:=10.42.0.3 
    ```


6. Run the RealSense camera launch command, and visualize the images in Rviz: 
    
    ```bash
    roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720 color_fps:=30
    ```

7. In RViz, add the `By display type > moveit_calibration_gui > HandEyeCalibration`.

8. In the HandEyeCalibration, set the target parameters:

  - measure the number of markers in a grid in X by Y (e.g., 2 x 2);
  - measure each marker size and the separation distance between each marker and set it in (m). (0.067, 0.0067 m respectively)m

9. Click on `Create Target` and print it out.

    ```{image} ../imgs/calibration_step1.png
    :alt: MoveIt Calibration Step 1
    :class: bg-primary mb-1
    :width: 70%
    :align: center
    ```

10. Choose the image topic and camera info topic where the image data is being published under `Target Pose Detection`.

    - image topic: `/camera/color_image/raw`;
    - camera info topic: `/camera/color/camera_info`;

11. Under the `Context` tab, set the `Sensor Configuration` as the appropriate configuration you are performing. 

12. Select the correct frames of the sensor, object, end-effector and robot base frame.

    - Sensor frame: `camera_color_optical_frame`;
    - Object frame: `handeye_target`;
    - End-effector frame: `panda_hand`;
    - Robot base frame: `panda_link0`;
    ```{image} ../imgs/calibration_step2.png
    :alt: MoveIt Calibration Step 2
    :class: bg-primary mb-1
    :width: 70%
    :align: center
    ```

13. Set the robot to "Programming Mode" to enable moving the robot manually.

14. In RViz, add  `By topic > /handEye_calibration > /target_detection > Image > raw`.

15. Take around 15 samples of different configurations under the `Calibrate` tab in the    `HandEyeCalibration` and click on "Take Sample" for each configuration after manually moving the robot.

    ```{image} ../imgs/calibration_step3.png
    :alt: MoveIt Calibration Step 3
    :class: bg-primary mb-1
    :width: 70%
    :align: center
    ```

16. Leaving the solver name same, click on "Save the camera pose" option. This will create a launch file of all the TFs of the calibrated Eye-in-hand transformations.

17. Run this launch file.

18. In RViz, Add TFs under `Add by Display`.
