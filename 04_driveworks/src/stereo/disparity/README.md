# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_stereo_disparity_sample Stereo Disparity Sample

The Stereo Disparity sample demonstrates the stereo pipeline.

![](sample_experimental_stereo_disparity.png)

The above diagram shows:

- Top: Anaglyph of left and right image
- Bottom: Stereo images

The sample reads frames from two rectified stereo videos. It then runs the
frames through the stereo pipeline and displays a confidence map and final stereo
output.

The stereo output is color coded for clarity and some pixels are masked if they
are occluded or invalid. It is possible to use keyboard input to change
parameters:

    0-6: changes the level of refinement (0 no refinement)
    Q,W: changes the gain to the color code for visualizaion
    O  : toggles occlusions
    K  : infills occlusions (only if on)
    +,-: changes invalidy threshold (appears as white pixels)
    I  : toggle horizontal infill of invalidity pixels

#### Usage

	./sample_stereo_disparity

The input arguments are 

    --video0=pathTo/videoLeftRect.h264
    --video1=pathTo/videoRightRect.h264
    --level=0, 1, 2, 3, pyramid level to display the disparity, depends on the number of levels (default 4)
    --single_side=true/false

If `--single_side` is false, the sample computes left and right stereo images
and performs complete stereo pipeline. Otherwise, it computes only the left
image and approximates occlusions by thresholding the confidence map.

