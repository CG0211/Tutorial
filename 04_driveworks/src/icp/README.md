# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_sample_icp ICP (Iterative Closest Planes) Sample

The NVIDIA DriveWorks ICP sample shows how to obtain ICP transforms via the
DriveWorks ICP module. The sample determines the relative transform between
two consecutive spins of Lidar and then chains them over a longer period. The
sample uses a point-to-plane algorithm and expects the points to be in an
order that approximately preserves their relative proximity in 3D space.

![ICP Sample](sample_icp.png)

Call the sample with the following syntax:

`./sample_icp <lidar_sample.bin>`

If no file is provided in the call, the sample uses the default Lidar
file, which is included in this DriveWorks release, `lidar_sample.bin`.

The sample output is shown on the screen.

The ICP sample supports the following options:

    --lidarFile,   string, Path to the Lidar file, which must be DW captured Velodyne HDL-64E file.
    --plyloc,      string, If specified, use this directory to write ICP-fused ASCII-PLY file.
    --init,        int,    Number of initial spins to skip before the first pair is fed to ICP.
                           Initial frames do not contain enough points for ICP.
                           Must be > 10.
    --skip,        int,    Number of frames to skip before getting the second spin
                           in the pair. The first spin in a pair is the second spin
                           from the previous pair. In the case of the first pair,
                           the first spin is specified by the value of --init + 1
                           and the second is N frames later, where --skip specifies N.
                           Must be < 5.
    --numFrames,   int,    These many pairs are used to perform ICP before stopping.
                           To process all frames, set to zero (0).
    --maxIters,    int,    Number of ICP iterations to run.


To pause the sample, press `SPACE`.

To rotate and move the camera while the sample is paused, select and drag the image.
