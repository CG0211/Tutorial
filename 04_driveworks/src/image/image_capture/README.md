# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_image_capture_sample Image capture sample

The Image capture sample shows how to record a video from either a CUDA image or
direclty from the rendering window. Text is rendered on the screen in order to 
illustrate the difference between recording from window or serializing the synthetic
cuda frame. 

--capture-bitrate=10000000
--capture-file=capture.h264
--capture-framerate=30
--capture-screen=1

if capture-screen is true, the current window will be recorded, otherwise the synthetic
CUDA image would be recorded. In order to differentiate, a text is rendered on top of the
window which will appear in the recorded video only when recording directly from screen
