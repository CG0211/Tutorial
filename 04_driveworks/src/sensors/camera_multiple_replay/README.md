# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_mul_video_replay_sample Multiple Video Replay Sample

![Four parallel H.264 streams](sample_camera_multiple_replay.png)

The Multiple Video Replay sample demonstrates H.264 playback by using a
hardware decoder to playback 4 video streams. The sample opens an X window to
play back the 4 provided video files. The playback does not support any
container formats (MP4 or similar); a pure H.264 stream is required.

Files, other than the defaults found in sample data, can be specified
with the `--video1`, .. `--video4=` arguments.

For example,

    ./sample_camera_multiple_replay --video1=/path/to/file.h264 --video2=/path/to/some/other/video.h264

On platforms that support NvMedia (i.e., DRIVE PX), the NvMedia engine is used
for decoding the stream. On Windows and Linux NVCUVID decoder is used; a proper
NVIDIA<sup>&reg;</sup> CUDA<sup>&reg;</sup> toolkit installation is required.
