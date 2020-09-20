# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_video_replay_sample Video Replay Sample

![Single H.264 stream](sample_camera_replay.png)

The Video replay sample demonstrates H.264 playback using a
hardware decoder.

The sample opens an X window to play back the provided video file. The playback
does not support any container formats (MP4 or similar); a pure H.264 stream is
required. Provide the name of the file by using the `--video=` argument.

For example:

    ./sample_camera_replay --video=/path/to/file.h264

On platforms that support NvMedia (i.e., DRIVE PX 2), the NvMedia engine is used
for decoding the stream. On Windows and Linux NVCUVID decoder is used; a proper
NVIDIA<sup>&reg;</sup> CUDA<sup>&reg;</sup> toolkit installation is required.

