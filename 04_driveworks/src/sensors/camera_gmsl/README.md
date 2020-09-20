# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_gmsl_sample GMSL Camera Capture Sample

The GMSL Camera Capture sample uses the GMSL camera interface, if available, on
NVIDIA DRIVE<sup>&trade;</sup> PX 2 platforms. The sample opens an X window
showing the input from the first camera on the selected CSI port.

![Single OV10640 camera capturing](sample_camera_gmsl.png)

## Setting Up Cameras

For information about the physical location of the ports on NVIDIA DRIVE PX 2
platforms, see "Camera Setup under Configuration and Setup" in _NVIDIA DRIVE 5.0 Linux
PDK Development Guide_.

## Options

A camera must be connected to one of the NVIDIA DRIVE PX 2 CSI ports.

The following options are supported:
- `--csi-port={ab,cd,ef}` specifies a CSI port. Default value: `ab`.
  For more information about
  the physical location of these ports on the board, see "Camera Setup" under
  "Configuration and Setup" in the _NVIDIA DRIVE 5.0 Linux PDK
- `--camera-type` specifies the type of the camera. Use @ref dwx_sensor_indexer_tool
  to query for supported camera types. Default value: `ar0231-rccb`.
- `--write-file` specifies the output file. If `--write-file` is not provided, no file is written out on disk.
- `--serializer-type` represents the format of the video with two possible options: `h264` or `uncompressed`.
  When `serializer-type` is set to `h264`, you must also set the
  `--serializer-bitrate`, which is by default set to `8000000`.
  Furthermore, the frame rate of the recorded video can be overwritten
  by providing `--serialize-framerate`.
- `--fifo-size={3,...}` specifies the camera buffer size. Note that the size must
  be at least 3. To reduce the delay, use a low number when not recording.

If the camera type is not ar0231 or the CSI port is not the default ab, execute:

    ./sample_camera_gmsl --csi-port=cd --camera-type=c-ov10640-b1

When running on NVIDIA<sup>&reg;</sup> Tegra<sup>&reg;</sup> B,
it is possible to specify the "slave" flag, which can
be 0 or 1. If slave is true, then Tegra B will not be able to run cameras
autonomously but it requires that the camera be simultaneously run from Tegra A.
If slave is false, then Tegra B can control any camera that is not currently
being used by Tegra A.

Screenshots can be captured by pressing `s` while the sample is running.

@note
 - Continental cameras model OV10640 can be launched under different names: the
   generic name is `c-ov10640-b1` and is kept for backward compatibility. 
   The two other modes `ov10640-svc210` and `ov10640-svc212` will activate lense specific
   configurations for an improved image quality. 


## Run on Tegra B

#### Master mode
Before running camera applications only on Tegra B, you must disable FRSYNC and
the forward/reverse control channel of Tegra A aggregator.
Please see *Camera Setup (P2379)* in *NVIDIA DRIVE 5.0 Linux PDK Development Guide*.
This guide will show show you how to:
* Turn MAX9286 aggregator on.
* Disable FRSYNC and the forward/reverse control channel on MAX9286 aggregator to avoid any interference with MAX96799.

After this steps camera applications can run on Tegra B after reboot.
Be aware that running camera applications on Tegra A re-enables the forward/reverse control channel and FRSYNC from MAX9286
and the procecedure of activating camera for Tegra B need to be repeated.

#### Slave mode
Cameras can be captured on Tegra B in slave mode, i.e. when they are already captured
by an application on Tegra A. In such case it is possible to specify the "slave" flag, which can
be 0 or 1. If slave is 1, then Tegra B will not be able to run cameras
autonomously but it requires that camera to run at the same time from Tegra A.
If slave is false, then Tegra B can control any camera that is not currently
being used by Tegra A.
