# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_isp_sample Software ISP Sample

The software Image Signal Processor (ISP) sample demonstrates how to use the GPU image processing available in
NVIDIA<sup>&reg;</sup> DriveWorks to convert from a RAW camera frame to a common DriveWorks image
type, e.g., RCB. It uses the @ref soft_ISP API.

![Software ISP Sample](sample_raw_pipeline.png)

The sample reads frames from a RAW camera recording, processes the frames to
convert them to RCB format, and displays the result on screen.

    ./sample_isp

The RAW pipeline provides options to select the demosaicing method. The default
is to use the `DW_DEMOSAIC_DOWNSAMPLE` method. This method is fast but reduces
the resolution of the frame. To produce full resolution output, change the
demosaicing method to `DW_DEMOSAIC_INTERPOLATION` via:

    ./sample_isp --interpolationDemosaic=1

It is possible to demosaic only a certain region of interest via the
`--enableCrop=0` and `--cropLeft`, `--cropTop`, `--cropWidth`, `--cropHeight`
options:

    ./sample_isp --enableCrop=1

For the full list of options, run the sample with the `--help` argument.
