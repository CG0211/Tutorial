# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_stereo_camera_usb_sample USB Stereo Camera Sample

The stereo camera usb sample illustrates how to input an image from a usb stereo
camera (such as StereoLab ZED camera) and use the input to rectify and compute
a disparity map and display side by side

#### Testing

A camera must be connected to a USB port. The device ID, `N` of the camera, to
use can be specified using the `--device=N` command line option, which is set to
0 by default.
In order for the sample to run, the camera has to be calibrated using our 
Calibration Tool and the rig.xml file needs to be provided as input

    ./sample_stereo_camera_usb --rigconfig=pathTo/rig.xml --device=0 (default)

