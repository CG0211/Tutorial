# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_stereo_rectifier_sample Stereo Rectifier Sample

The Stereo Rectifier sample demonstrates stereo rectification.

![Unrectified and rectified images](sample_experimental_stereo_rectifier.png)

The above image shows:
- Top: Input left and right image with parallel red line showing that epipolar lines are not parallel.
- Bottom: Rectified images.

The sample consumes:
- Two videos taken from a calibrated stereo camera
- Rig configuration XML file, which contains the calibration of the camera (extrinsics and intrinsics)

The sample then performs rectification on the videos.

### Usage

	./sample_stereo_rectifier

### Runtime Commands

    space : pause

### Input Arguments

    --video0=pathTo/videoLeftRect.h264
    --video1=pathTo/videoRightRect.h264
	--single-input=1 (if only one unput is provided)
    --rigconfig=pathTo/rig.xml

Where the rig XML file contains the intrinsics and extrinsics for the stereo
camera. For guidance on producing the rig configuration file, see
<a href="NVIDIA_DriveWorks_Calibration_Tool.pdf" target="_blank">NVIDIA DriveWorks Calibration Tool</a>.
