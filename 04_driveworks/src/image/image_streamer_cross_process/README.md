# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_image_streamer_cross_sample Image Streamer Cross-Process Sample

The image streamer cross-process sample demonstrates how to use an image
streamer across multiple processes.

## Running the Sample

The command line for the sample is:

    ./sample_image_streamer_cross --type=<consumer|producer> --csi-port=<ab/cd/ef> \
                 --timeout=<any in ms> --fifo-size=<3...>

You must run the sample from two terminals.
- On the first terminal, enter the above command with only the
  `type=consumer` argument. This launches a window and a cross-process consumer
  that waits for the producer.

      ./sample_image_streamer_cross --type=consumer

- On the other terminal, enter the above command but specify `type=producer` and the
  other optional parameters. Because this is a demonstration on cross-process
  streamer, only camera ar0231-rccb and similar are supported (resolution
  1920x1208).

      ./sample_image_streamer_cross --type=producer --csi-port=<ab/cd/ef> \
                   --timeout=<any in ms> --fifo-size=<3...>

NOTE: that the consumer needs to be launched first. In case the producer is launched first, do not kill
the process, simply wait for it to timeout. When consumer is running, after launching the 
producer, it will start receiving frames and displaying them on screen.


