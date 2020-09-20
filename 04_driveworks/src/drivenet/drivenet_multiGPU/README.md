# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_drivenet_multigpu_sample DriveNet Multi-GPU Sample

The DriveNet multi-GPU sample shows how to use two GPUs to perform the same task
that the @ref dwx_object_tracker_drivenet_sample performs. Here, inference and
tracking are split between GPU 0 (inference) and GPU 1 (tracking). For more
information about the functionality of Drivenet and the arguments of the sample,
see @ref dwx_object_tracker_drivenet_sample.


## Running the Sample on NVIDIA DRIVE platforms

The command line for running the sample with video input:

    ./sample_drivenet_multigpu --input-type=video --video=<video file.raw>

The command line for running the sample with camera input:

    ./sample_drivenet_multigpu --input-type=camera --camera-type=<rccb camera type> --csi-port=<csi port> --camera-index=<camera idx on csi port>

where `<rccb camera type>` is one of the following: `ar0231-rccb`, `ar0231-rccb-ssc`, `ar0231-rccb-bae`, `ar0231-rccb-ss3322`, `ar0231-rccb-ss3323`

## Running the Sample on Linux

The command line for running the sample with video input:

    ./sample_drivenet_multigpu --video=<video file.raw> --stopFrame=<frame_idx>

This runs `sample_drivenet_multigpu` until frame `<frame_idx>`. Default value is 0, for which the sample runs endlessly.

## Output

The sample creates a window, displays a video, and overlays bounding boxes for detected objects.
The color of the bounding boxes represent the classes that it detects:

    Red: Cars
    Green: Traffic Signs
    Blue: Bicycles
    Magenta: Trucks
    Orange: Pedestrians

## Limitations ##

@note At least 2 GPUs are needed. If the GPUs have peer access the performance will be higher.
