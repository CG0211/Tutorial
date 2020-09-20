# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_tracker_sample Camera Tracker Sample

The Camera Tracker sample demonstrates the feature detection and feature
tracking capabilities of the dw_features module. It loads a video stream and
reads the images sequentially. For each frame, it tracks features  from the
previous frame and detects new features in the current frame.

![Tracked feature points on a single H.264 stream](sample_camera_tracker.png)

#### Running the Sample

The command line for the sample is:

    ./sample_camera_tracker --video=<video file.h264>

- The video option receives the full path to the video file to load.
- The file must be an H.264 stream.
- Video containers such as MP4, AVI, MKV, etc. are not supported.

#### Output

The sample creates a window, displays the video, and overlays the list of
features.

There are two modes for feature drawing:
- (default) feature trajectories will be overlaied (up to previous 10 frames of history).
- Only the current feature positions will be overlaied by small squares.
You can switch drawing mode by pressing 'h' key.

The color represents age of each feature (= how many frames has it been tracked):
- Red: 1 <= age < 5
- Yellow: 5 <= age < 10
- Green: 10 <= age < 20
- Light blue: 20 <= age
