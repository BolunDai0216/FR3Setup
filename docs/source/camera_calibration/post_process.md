# Post-Processing For Two Arm Setup

From eye-in-hand calibration, we get the transformation between the end-effector (hand) frame and the camera frame, which looks like the following

```xml
<!-- white_hand_to_white_camera.launch -->
<launch>
  <!-- xyz="0.0494312 -0.0293577 0.0631452" rpy="3.13523 3.12853 -1.57277" -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="camera_link_broadcaster"
      args="0.0494312 -0.0293577 0.0631452   0.00236196 0.00686826 0.706374 0.707802 panda_hand camera_color_optical_frame" />
</launch>
```

From the eye-to-hand calibration, we get the transformation between the base of the gray FP3 and the camera frame rigidly attached to the end-effector of the white FR3, which looks like the following


```xml
<!-- gray_base_to_white_camera.launch -->
<launch>
  <!-- xyz="0.393764 -1.40181 0.453054" rpy="1.58379 -3.10959 -3.12857" -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="camera_link_broadcaster"
      args="0.393764 -1.40181 0.453054   -0.702468 -0.00681608 0.0158737 0.711506 panda_link0 camera_color_optical_frame" />
</launch>
```

```{figure} ../imgs/post_process_step1.png
:alt: Post Processing Step 1
:class: bg-primary mb-1
:width: 100%
:align: center
Transformation obtained using eye-in-hand and eye-to-hand calibration.
```

Then, to get the relative transformation of the white FR3 base in the gray FP3 base frame, we first need to get the transformation between the camera frame rigidly attached to the end-effector of the white FR3 and the base frame of the FP3, which is the inverse of what we have in `gray_base_to_white_camera.launch`. To do this, we run the following commands

```bash
roslaunch gray_base_to_white_camera.launch
python3 generate_launch_file.py --parent "/camera_color_optical_frame" --child "/panda_link0" --filename "white_camera_to_gray_base" --parentName "camera_color_optical_frame" --childName "grey_panda_link0"

# Optional step to format the launch file
xmllint --format white_camera_to_gray_base.launch --output white_camera_to_gray_base.launch
```

```{figure} ../imgs/post_process_step2.png
:alt: Post Processing Step 2
:class: bg-primary mb-1
:width: 100%
:align: center
Transformation obtained applying the steps above.
```

This will generate a launch file called `white_camera_to_gray_base.launch` in the current directory. Then, to get the relative transformation of the white FR3 base in the gray FP3 base frame, we run the following commands

```bash
roslaunch white_hand_to_white_camera.launch
roslaunch white_camera_to_gray_base.launch
python3 generate_launch_file.py --parent "/panda_link0" --child "/gray_panda_link0" --filename "white_base_to_gray_base" --parentName "white_panda_link0" --childName "gray_panda_link0"

# Optional step to format the launch file
xmllint --format white_base_to_gray_base.launch --output white_base_to_gray_base.launch
```

these commands will generate a launch file called `white_base_to_gray_base.launch` in the current directory which gives us the transformation between the white FR3 base and the gray FP3 base.

```{figure} ../imgs/post_process_step3.png
:alt: Post Processing Step 3
:class: bg-primary mb-1
:width: 100%
:align: center
Transformation obtained applying the steps above.
```

## Deploying the Calibration in the Two Arm Setup

Now that we assume that the white FR3's joint angle has changed, and we want to find the transformation between the gray FP3 base and the camera frame that is rigidly attached to the end-effector of the white FR3. To do this, we first launch two launch files

```bash
roslaunch white_base_to_gray_base.launch
roslaunch white_hand_to_white_camera.launch
```

which creates all the TF transformations required. We can then query TF to obtain the transformation ${}^\mathrm{camera}\mathbf{T}_\mathrm{gray\ base}$.
