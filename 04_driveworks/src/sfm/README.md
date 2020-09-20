# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_struct_from_motion_sample Structure from Motion Sample

The Structure from Motion sample demonstrates the triangulation functionality of
the SFM module; a car pose is estimated entirely from CAN data using the NVIDIA<sup>&reg;</sup>
DriveWorks egomotion module. The car has a 4-fisheye camera rig that is pre-calibrated.
Features are detected and tracked using the features module. Points are
triangulated for each frame by using the estimated pose and tracked features.

![Structure from Motion sample](sample_triangulation.png)

## Running the Sample

The command line for the sample is:

    ./sample_sfm --video0=<video file.h264> --video1=<video file.h264>
                 --video2=<video file.h264> --video3=<video file.h264>
                 --rig=<rig.xml>
                 --can=<canbus.can>
                 --videoTimestamps=<timestamps.txt>

- The video options receive the full path to the video files to load.
- The file must be an H.264 stream.
- Video containers as MP4, AVI, MKV, etc. are not supported.
- The `rig.xml` file is in the format produced by the DriveWorks calibration tool.
- The CAN bus file is read by the canbus virtual sensor.
- The video timestamps is a text file where each row contains the frame index
(starting at 1) and the timestamp for all cameras. It is read by the camera virtual sensor.

If a mouse is available, the left button rotates the 3D view, the right button
translate, and the mouse wheel zooms.

While the sample is running the following commands are available:
- Press V to enable / disable pose estimation.
- Press Space to pause / resume execution.
- Press Q to switch between different camera views.

## Output

The left side of the screen shows the 4 input images; tracked features are shown
in green. Triangulated points are reprojected back onto the camera and shown in
red. The right side shows a 3D view of the triangulated point cloud.

In 3D, the colors are:

- Blue points = points from frontal camera
- Red points = points from rear camera
- Green points = points from left camera
- Yellow points = points from right camera
- Green line = path of the car

@note When the playback of the data reaches the end of the files, the
application will not rewind and play, but will keep the final results on
the screen so the user can inspect the final obtained point cloud.
