# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_sample_third_party_camera Third-Party Camera Sample

![DriveWorks window displaying data from artificial sensor](sample_third_party_camera.png)

#### Objective

The Third-Party Camera Sample demonstrates how user can take advantage of their
own sensor.

This sample consists of an artificial sensor that generates images in CPU
memory. The sample illustrates how to wrap this buffer in a `dwImageCPU`,
timestamps and then uses it as a native DriveWorks image.


#### Testing
To run this sample, issue the following :

    ./sample_third_party_camera

A virtual camera will generate images that will be wrapped in DriveWorks images, and then streamed for displayed, using DriveWorks facilities.

