# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_lane_detection_sample Lane Detection Sample (LaneNet)

This lane detection example demonstrates how to use the NVIDIA proprietary deep neural network (DNN)
to perform lane marking detection on the road. It detects the lane you are in (ego-lane), as well
as the left and right adjacent lanes when they are present. LaneNet has been trained with RCB
images and aggressive data augmentation which allows the network to perform correctly when using
RGB encoded H.264 videos.

This lane detection sample can stream a H.264 or RAW video and computes the likelihood map of lane markings
on each frame. An user assigned threshold value binarizes a likelihood map into clusters of lane markings,
then image post-processing steps are employed to fit polylines onto the lane clusters and assign them with
lane positon types. The sample can also be operated with cameras.

![Lane Detection Sample](sample_lane_detection.png)

### Sensor details ####

The image datasets used to train Lanenet have been captured by a View Sekonix Camera Module (SS3323) with
AR0231 RCCB sensor. The camera is mounted high up at the rear view mirror position. Demo videos are
captured at 2.3 MP and down-sampled to 960 x 604.

To achieve the best lane detection performance, NVIDIA recommends to adopt a similar camera setup and align
the video center vertically with the horizon before recording new videos.

### Sample ###

The sample H264 video is located at:

    sdk/external/data/samples/laneDetection/video_lane.h264

The latency of the sample LaneNet model:
- DRIVE PX 2 with GP106: 4.9 milliseconds
- DRIVE PX 2 with iGPU: 21.6 milliseconds

#### Running the Sample

The command lines for running the sample on Linux:

    ./sample_lane_detection --video=<video file.h264> --threshold=<floating-point number in (0,1)>
or

    ./sample_lane_detection --video=<video file.raw> --threshold=<floating-point number in (0,1)>

The command line for running the sample on DRIVE PX 2 with cameras:

    ./sample_lane_detection --input-type=camera --camera-type=<camera_type> --csi-port=<csi_port> --threshold=<floating-point number in (0,1)>

where `<camera type>` is one of the following: `ar0231-rccb`, `ar0231-rccb-ssc`, `ar0231-rccb-bae`, `ar0231-rccb-ss3322`, `ar0231-rccb-ss3323`, `c-ov10640-b1`, `ov10640-svc210`, `ov10640-svc212`


Note that lane detection sample directly resizes video frames to the network
input resolution. Therefore, to get the best performance, it is suggested to
use videos with similiar aspect ratio to the demo video.  Or you can set Region
of Interest (ROI) to perform inference on a sub-window of the full frame.


### Command Line Parameters ###

#### `--`threshold ####
Parameter: A floating point number between 0 and 1.

Description: The threshold parameter is used to binarize a likelihood map.
Any likelihood value above the threshold is considered a lane marking pixel.
By default, the value is 0.3, which provides the best accuracy based on the NVIDIA
test data set. Reduce the threshold value if lane polylines flicker or cover
shorter distances.

#### Output

LaneNet creates a window, displays a video, and overlays ploylines for
detected lane markings. The colors of the ploylines represent the lane marking
position types that it detects, as follows:

- Red: Ego-lane left
- Green: Ego-lane right
- Yellow: Left adjacent lane
- Blue: Right adjacent lane

LaneNet detects the following lane markings and treats them as the same type.

- Yellow solid + dash
- White dash
- White solid
- White 2x solid
- Yellow dash
- Yellow solid
- Yellow 2x solid
