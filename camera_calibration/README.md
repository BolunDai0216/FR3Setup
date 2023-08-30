# Camera Calibration

The camera calibration process contains two types of calibration: eye-in-hand and eye-to-hand. 

## Eye-in-Hand Calibration

The hand-in-eye calibration is used to find the transformation matrix between the camera frame and the end-effector frame of the robot. In this case, we assume that the camera is rigidly attached to the end-effector of the robot.

<p align="center">
  <img src="./imgs/eye-in-hand.png" alt="Eye-in-Hand Calibration" style="width:70%;"/>
</p>


## Eye-to-Hand Calibration

The hand-to-eye calibration is used to find the transformation matrix between the camera frame and the base frame of the robot. In this case, we assume that the camera is fixed relative to the base of the robot.

<p align="center">
  <img src="./imgs/eye-to-hand.png" alt="Eye-to-Hand Calibration" style="width:70%;"/>
</p>
