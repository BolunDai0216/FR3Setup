# Vicon Setup

This document provides a step-by-step guide for using the Vicon Tracking System for motion capture and object tracking.

## Getting Started

```{figure} ../imgs/getting_started_vicon_system_switches.png
:alt: Vicon System Switches
:class: bg-primary mb-1
:width: 80%
:align: center
**Step 1: Turn on the 3 switches.**
```

```{figure} ../imgs/getting_started_vicon_tracker.png
:alt: Vicon Tracker
:class: bg-primary mb-1
:width: 80%
:align: center
**Step 2: Open Vicon Tracker 3.10.0**
```

```{figure} ../imgs/getting_started_vicon_tracker_controls.png
:alt: Vicon Tracker Controls
:class: bg-primary mb-1
:width: 80%
:align: center
**When controlling the 3D view, the following controls are available: Rotate (left mouse click), Zoom in/out (right click), Translate (middle scroll button).**
```

## Calibrating the Vicon System

- The first step in using the Vicon system is to calibrate it.

- Under the Systems tab, select `Calibrations` and choose `Strobes Off 240Hz` for calibration

```{image} ../imgs/calibrate_vicon_system.png
:alt: Vicon System Tab
:class: bg-primary mb-1
:width: 40%
:align: center
```

- Grab the calibration wand.
   
```{image} ../imgs/calibrate_wand.jpg
:alt: Vicon System Switches
:class: bg-primary mb-1
:width: 80%
:align: center
```

- The wand has two main switches: **On/Off and Stroke/Continuous**.

- Put the wand in `Continuous Mode` by toggling the switch.

- Under the Calibrate Menu, navigate to `Create Camera Masks` and click on 'Start`. This will disable all previous masks.

```{image} ../imgs/calibrate_menu.png
:alt: Calibrate Menu
:class: bg-primary mb-1
:width: 40%
:align: center
```

- Press the stop button after **10 seconds** to reset the masks.

- Now, press the **Start** button on The Calibrate Camera tab. This is the process for calibrating the cameras.

- Switch on the wand in continuous mode and move it around in as many different positions and orientations as possible.

```{image} ../imgs/calibrate_camera_calibrate_vicon.png
:alt: Calibration Process
:class: bg-primary mb-1
:width: 80%
:align: center
```

- Move the wand to cover the maximum window area possible. You need as many patterns as possible to be seen on the screen. The first box below shows many black-colored patches. These need to be filled as much as the image under it by moving the wand continuously.

```{image} ../imgs/calibrate_comparing_screens.png
:alt: Calibration Process
:class: bg-primary mb-1
:width: 40%
:align: center
``` 

- Stop the process when you see the progress bars reach 100% and the light turns green.

```{image} ../imgs/calibrate_camera_progress_bar.png
:alt: Camera Progress Bar
:class: bg-primary mb-1
:width: 40%
:align: center
``` 

- Make sure the **image error** is less than **0.5px**.

```{image} ../imgs/calibrate_image_error.png
:alt: Camera Progress Bar
:class: bg-primary mb-1
:width: 40%
:align: center
``` 

- Defining the origin:
    - Place the wand on the ground laying flat.
    - Ensure the bubble on the wand is in the center.
    - Switch on the wand in the `Continuous mode` itself.

- Under `Set Volume Origin`, press start and then `Set origin`. You should be able to see the wand. You can now switch off the wand and keep it aside.

- If the wand markers are not visible in the window, go to the `Object` menu, and under `Object Tracking Mode`, make sure `Track` is unselected.

```{image} ../imgs/calibrate_track_mode_selection.png
:alt: Track mode selection
:class: bg-primary mb-1
:width: 80%
:align: center
``` 

- The cameras are calibrated and the axes are marked below.

```{image} ../imgs/calibrate_set_origin.jpg
:alt: Track mode selection
:class: bg-primary mb-1
:width: 80%
:align: center
``` 

## Object Tracking

- Under the `Systems` menu, choose the `tracking_mode`.

- Cover the markers you would like to track (for example, Unitree). We will now create masks on objects we do not wish to track.

- Go to `Calibrate` > `Create camera masks` > `Start`. Wait for a while until all the markers on the screen turn blue and then press **Stop**.

### Object Definition:
- Place the markers on the object of interest such that the placement of the markers on the object is not symmetrical to each other.
    
-  Use a minimum of > 3 markers to define an object (e.g., on a plane board, put 5 markers not symmetrical to each other; the 5th marker is placed randomly in between the 4 markers to keep track of the rotations of the board and break the symmetry).

```{image} ../imgs/object_tracking_marker_placement.png
:alt: Marker Placement
:class: bg-primary mb-1
:width: 50%
:align: center
``` 

- To define the object you wish to track, select all the markers by dragging your mouse over all the markers and press Alt. You can also select each marker by clicking and holding Ctrl.

```{image} ../imgs/object_tracking_define_object.png
:alt: Object Definition
:class: bg-primary mb-1
:width: 80%
:align: center
``` 

- Once the object is selected, under `Object` > `Create Object` , type the name and hit `Create`.

**NOTE**: To save the configuration, click on  `Shared` > `Save Selected`
