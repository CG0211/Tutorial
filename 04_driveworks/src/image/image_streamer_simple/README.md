# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_image_streamer_simple_sample Image Streamer Simple Sample

The image streamer sample demonstrates how to use an image streamer.

The sample has no inputs and is intended as a guide on how to properly create, setup, use and release
an image streamer. The sample does the following:
1. Manually creates a dwImageCPU object.
2. Streams it to a dwImageCUDA object.
3. Applyies an NVIDIA<sup>&reg;</sup> CUDA<sup>&reg;</sup> kernel on it.
4. Streams the resulting image to dwImageGL object.
5. Render the image on screen.

The sample contains comments for every step.

## Running the Sample

Enter:

    ./sample_image_streamer_simple

Supported options:

    --offscreen to enable off-screen rendering.