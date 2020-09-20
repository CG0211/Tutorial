# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_image_streamer_multi_sample Image Streamer Multi-Thread Sample

The Image streamer sample demonstrates how to use an image streamer in a multi-thread environment.
It consumes a CPU image.

The sample shows how to create, setup, use and release
an image streamer in multi-thread. It does the following:
1. Manually creates a dwImageCPU object.
2. Streams the dwImageCPU to a dwImageCUDA object.
3. Applies a NVIDIA<sup>&reg;</sup> CUDA<sup>&reg;</sup> kernel on it.
4. Streams the resulting image to a dwImageGL object.
5. Renders it on screen.

The sample contains comments for every step.

## Running the Sample

To run the sample, enter:

    ./sample_image_streamer_multi

The sample has no inputs.
