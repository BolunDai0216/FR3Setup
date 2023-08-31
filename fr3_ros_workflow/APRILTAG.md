# April Tag Detection

To perform April Tag detection using a RealSense camera in ROS, there are three major changes that need to be made:

## Installation of the April Tag Detection ROS package

**1.** **Follow** the instructions on the [April Tag Detection ROS package](https://github.com/AprilRobotics/apriltag_ros) to install ROS Noetic realsense2_camera package.  

```bash
export ROS_DISTRO=noetic               # Set this to your distro, e.g. noetic
export FRANKA_WS= desired/path/to/workspace
source FRNAKA_WS/devel/setup.zsh                     # Navigate to the source space
```
**2.** Clone the repository

```bash
cd
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
cd ~/catkin_ws                                           # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)
```
## Using the April Tag Detection ROS library

**1.**  **Download the April Tag PDF from GitHub:**
   Download the April Tag PDF from the provided GitHub link.

**2.** **Note the ID, Family, and Size of the Tag:**
    - Identify the ID, Family, and size of the April Tag you'll be using.

**3.**  **Configure the `tag.yaml` File:**
   - Open the `tag.yaml` file located in the `apriltag_ros/apriltag_ros/config` folder of the April Tag ROS package.
   - Under the `standalone` tab, add the April Tag ID and size: 

        For example :`{id: 0, size: 0.2}` (200mm is 0.2m).

**4.**  **Check Camera and Image Names in the Launch File `continuous_detection.launch`:**
   - `apriltag_ros/apriltag_ros/launch/continuous_detection.launch`
   - The **camera name** is determined by the first two names of the camera topics published by the RealSense camera : `/camera/color`
   - The image name should be set to `image_raw`

**5.**  **Set Tag Family Name in `settings.yaml`:**
   - Ensure that the tag family name specified in the `apriltag_ros/apriltag_ros/config ` folder ,in the `settings.yaml` file matches the family of the April Tag being used.

**6.** **Run April Tag Detection:**
   - Run the April Tag detection launch file **after running the RealSense camera launch file** using the following command:

   ```bash
   roslaunch apriltag_ros continuous_detection.launch   
   ```

**7.**  **Visualize Detection in RViz:**
   - Run RViz.
   ```bash
   rosrun rviz rviz
   ```
   - Click on **Add by topic** and choose the image section as **raw** under the `tag_detection` topic.

**8.** **Place the April Tag Print Under the Camera:**
   - Position the printed April Tag under the camera.
   - Keep in mind that the size of the April Tag print may vary from the size specified in the `tag.yaml` file.

**9.**  **View Detected April Tag:**
   - You should be able to see the image with the detected April Tag in rviz
   - You can also **use `rqt_image_view` for Viewing Detected April Tag:**