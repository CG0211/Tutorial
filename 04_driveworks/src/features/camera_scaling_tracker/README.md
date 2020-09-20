# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_scaling_tracker_sample Camera Tracker Sample

The Camera Scaling Tracker sample demonstrates the scaling feature
tracking capabilities of the dw_features module. It loads a video stream and
reads the images sequentially. For each frame, it tracks scaling features from the
previous frame. It doesn't detect new features, when there's no scaling features
in the frame, the video replay will be paused automatically, you can use
mouse to drag the boxes to track and press space to start replay/tracking.

![Tracked scaling feature bounding boxes on a single H.264 stream](sample_camera_scaling_tracker.png)

#### Running the Sample

The command line for the sample is:

    ./sample_camera_scaling_tracker --video=<video file.h264>

- The video option receives the full path to the video file to load.
- The file must be an H.264 stream.
- Video containers such as MP4, AVI, MKV, etc. are not supported.

#### Output

The sample creates a window, displays the video, and overlays the list of
features.
The sample doesn't detect new features, you need press space to pause the video,
drag mouse to add new bounding boxes for tracking, and press space to start. When
there's no features in video, the video will pause automatically
