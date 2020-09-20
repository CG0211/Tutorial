# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_calibration_sample Camera Calibration Sample

This sample demonstrates the ability to estimate camera extrinsics using the
DriveWorks Calibration Engine.

![Camera Calibration](sample_calibration_camera.png)

#### Running the Sample

The default usage is `./sample_calibration_camera`, and the sample reads predefined sample data.

Options are:

    --path={sample_path}                                The base path to offline data
    --rig=rig.xml                                       The name of the rig configuration file
    --sensorIndex=0                                     The index of the camera in the rig configuration file
    --video=video_0_roof_front_120.h264                 The name of the video file
    --video-time=video_time_0.txt                       The name of the video time file
    --can=can_vehicle.bin                               The name of the CAN file
    --imu=imu_xsens.bin                                 The name of the imu file

    --dbc={(sample_path}can.dbc                         The name of the CAN dbc file
    --dbc-speed=Steering_Report.SPEED                   The field name for speed in the CAN data
    --dbc-steeringSteering_Report.ANGLE                 The field name for angle in the CAN data
    --lanenet-threshold=0.3                             The threshold for lanenet

#### Output

The sample creates a window, displays a video, and displays nominal calibration
indicators (blue) and after convergence a corrected calibration indicators
(green). Indicators are showing the estimated rig horizon + forward directions,
as seen in the camera's frame.
