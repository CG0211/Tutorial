# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_seek_sample Camera Seek Sample

![Single H.264 stream](sample_camera_seek.png)

This sample demonstrates how to replay a video and use the seek to timestamp/event
feature to seek to any point in the video

The sample works analogous to sample_camera_replay:

For example:

    ./sample_camera_seek --video=/path/to/file.h264

The sample works with the following keyboard inputs

    T  : changes the seek mode to timestamp
    F  : changes the seek mode to frame event
    Left Arrow  : step backward (20 for frames, 100000 for timestamp)
    Right Arrow  : step forward (20 for frames, 10000 for timestamp)
    Space : pause the video

Seeking is available in both normal and paused states.


