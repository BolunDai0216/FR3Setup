<p align="center">
  <img src="imgs/logo.png" alt="logo" style="width:100%;"/>
</p>

This repo contains tutorials and instructions on how to perform camera calibrations for the FR3 robot.

## Content

- Prerequsites
- Reading from Intel RealSense Camera
- AprilTag Detection
- AprilTag Bundle Adjustment
- Hand-in-Eye Calibration
- Hand-to-Eye Calibration



Prerequsites
-Linux machine  or a VMware workstation
-ROS noetic
-Install the relasensek ros pkg
-Install the apriltag ROS pkg

-Reading from the Intel RealSense Camera
  VMWare
  Detection of the camera on the virtual machine
    1. Connect the real sense camera 
    2. Go to the VMware settings and choose Removeable devices and click on Intel Realsense camera option
    3. Click on " Connect to machine from host".
    4. Once that is done, VMware will enable to detect your camera
      To check this, you can go on the terminal and type , "lsusb". This will show you the list of connected devices on the VNMware
    5.Download "cheese". this is a camera app on linux to view and capture image 
    6. If you see the realsense feed on the cheese app, this would mean the camera is working on the VMware.

  - Using realsense ROS package : Put the link for downloading (Highlight the steps)
    1. Check the firmware version of the real sense camera
      HIGHLIGHT THE STEPS TO CHECK THE LATEST FIRMWARE ON THE CAMERA. SINCE ROS PCKGS MIGHT NOT WORK AND WILL THROW AN ERROR FOR THE CAMERA IF FIRMWARE IS NOT RIGHT
    2. Once tthat is updated , run the launch file for real sense and (NAME OF THE LAUNCH FILE)
    3. Visualize the image on Rviz : rosrun rviz rviz
    4. Click on "Add by Topic" and under the image , select raw, you should be able to view a live feed of the realsense camera

-April tag detection
There are majorly three chnages that are to be done for April Tag detection using a Real sense camera in ROS
1. Download the April Tag .pdf from the github links :PROVIDE THE GITHUB LINK
2.Notthe the ID , Family and size of that tag
3. Open the tag.yaml file in the /config folder of the april tag ROS package
4. Under the tag.yaml , add the April Tag ID and size under the standalone tab. {id: 0, size:0.2}   200mm is 0.2m
5. Go to the launch file and check the camera name and image name, The camera name is given by the first 2 names of the camera topics published by the realsense camera.The image name is "image_raw"
6. Once these parameters are set, check the tag family name in the settings.yaml file. The name of the family should be identical to the April tag being used.
7. Run the april tag detection launch file after running the real sense camera launch file.
8. Run rviz. Click on "Add by topic" and choose the image section as raw under the tag_detection topic.
9. Place the print of the April tag under the camera. Keep in mind the size of the April tag print may vary to the size specified under the tag.yaml file.
10. You should be able to see the image with detected april tag.
11. You can also use the rqt_image_view to view the detected april tag
