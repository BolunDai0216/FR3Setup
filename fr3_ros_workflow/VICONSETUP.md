# Vicon Tracking System

This document provides a step-by-step guide for using the Vicon Tracking System for motion capture and object tracking.

## Getting Started

**1.** Turn on the 3 switches.
   
   ![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/d0d3c609-9cdd-46a5-8b1d-ec9d011ee8e7)

**2.** Open Vicon Tracker 3.10.0.

   ![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/7a2fa313-1c89-4c45-8d58-7a2d4296b399)


**3.** Control the 3D view:
   - Left mouse click: Rotate
   - Right Click: Zoom in/out
   - Middle Scroll button: Translate
     
![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/708f5ae0-bde2-4f8f-9798-1218fdb63544)


## Calibrating the Vicon System

**4.** The first step in using the Vicon system is to calibrate it.

**5.** Under the Systems tab, select `Calibrations` and choose `Strobes Off 240Hz` for calibration

![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/7562f2c6-6a66-438a-a99e-57294e33c679)


**6.** Grab the calibration wand.
   
![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/0193088c-1d3e-42ee-ae25-e112d2663e3d)

**7.** The wand has two main switches: **On/Off and Stroke/Continuous**.

**8.** Put the wand in `Continuous Mode` by toggling the switch.

**9.** Under the Calibrate Menu, navigate to `Create Camera Masks` and click on 'Start`. This will disable all previous masks.

  ![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/adc319f3-1553-47af-a1f2-ca4a5357b911)

**10.** Press the stop button after **10 seconds** to reset the masks.

**11.** Now, press the **Start** button on The Calibrate Camera tab. This is the process for calibrating the cameras.

**12.** Switch on the wand in continuous mode and move it around in as many different positions and orientations as possible.

![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/257bd160-73a4-4098-8c31-e0c9f877d90b)

**13.** Move the wand to cover the maximum window area possible. You need as many patterns as possible to be seen on the screen.

![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/8b54500a-18c0-4ce1-a93b-56c52f1a0b62)  

  The first box above shows many black-colored patches. These need to be filled as much as the image under it by moving the wand continuously.

**14.** Stop the process when you see the progress bars reach 100% and the light turns green.

![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/ee981c3e-615c-49f2-808d-fdee73aa1839)

**15.** Make sure the **image error** is less than **0.5px**.
    
![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/50ce17f4-5cd1-419f-9e04-0d47b4f09097)

**16.** Defining the origin:
    - Place the wand on the ground laying flat.
    - Ensure the bubble on the wand is in the center.
    - Switch on the wand in the `Continuous mode` itself.

**17.** Under `Set Volume Origin`, press start and then `Set origin`. You should be able to see the wand. You can now switch off the wand and keep it aside.

**18.** If you do not see the wand markers, go to the `Object` menu, and under `Object Tracking Mode`, make sure `Track` is unselected.

![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/9c1cec84-d56f-45a8-adaa-74be347dc4ac)

**19.** The cameras are calibrated and the axes are marked below.

 ![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/61076b95-d7dc-4d80-a5ac-04552139586d)

## Object Tracking

**20.** Under the `Systems` menu, choose the `tracking_mode`.

**21.** Cover the markers you would like to track (for example, Unitree). We will now create masks on objects we do not wish to track.

**22.** Go to `Calibrate` > `Create camera masks` > `Start`. Wait for a while until all the markers on the screen turn blue and then press **Stop**.

### Object Definition:
**23.** Place the markers on the object of interest such that the placement of the markers on the object is not symmetrical to each other.
    
**24.**  Use a minimum of > 3 markers to define an object (e.g., on a plane board, put 5 markers not symmetrical to each other; the 5th marker is placed randomly in between the 4 markers to keep track of the rotations of the board and break the symmetry).

![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/aa1b070e-d30e-440d-973a-b2e78c085cbb)

**25.** To define the object you wish to track, select all the markers by dragging your mouse over all the markers and press Alt. You can also select each marker by clicking and holding Ctrl.

![image](https://github.com/PranayG/HowToCalibrate/assets/9202531/61fb910a-0198-4624-b95a-7d93463293db)

**26.** Once the object is selected, under `Object` > `Create Object` , type the name and hit `Create`.

**NOTE**: To save the configuration, click on  `Shared` > `Save Selected`
