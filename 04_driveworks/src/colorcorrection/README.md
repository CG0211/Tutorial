# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_color_correction_sample Camera Color Correction Sample

The color correction sample demonstrates H.264 playback with color
correction. The sample opens an X window to play back the 4 provided
video files and corrects their color based on a selected master camera
index. Color correction can be activated or deactivated by pressing the SPACE key.

![Four parallel H.264 streams](sample_camera_color_correction.png)

It is important to notice that color correction happens by reprojecting all cameras into a common plane.
In case of this sample the common plane is the ground plane.
That means when color correction is enabled the hue of the ground texture will be equalized to the selected
master camera. Please note that when modfying based on ground plane, other parts of the image might look
slightly different as one would intuitively expect.

# Usage

Run this sample by executing:

    ./sample_color_correction

### Command Line Options

These are the available command line options:

#### `--video1=filename1 --video2=filename2 --video2=filename3 --video2=filename4` ####
- Parameter: [Name of the file]
- Description: Provides the name of the H.264 input video.

#### `--rig=filename' ####
- Parameter: [Name of rig file]
- Description: Provides the name of the rig file that contains calibration information.
- Usage: `--rig=filename'

#### `--ref=[0,1,2,3]` ####
- Parameter: [Name of master camera]
- Description: Choose the master camera.

#### --factor=[0.0f...1.0f] ####
- Parameter: [Floating number between 0, 1]
- Description: Specifies the color correction factor. 0 means no correction (original video) and 1 means fully using the master camera's color.

On platforms that support NvMedia (i.e., DRIVE PX 2), the NvMedia engine is used for
decoding the stream. On Windows and Linux, NVCUVID decoder is used;
a proper NVIDIA<sup>&reg;</sup> CUDA<sup>&reg;</sup> toolkit installation is required.
