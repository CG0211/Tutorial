# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_camera_usb_sample USB Camera Capture Sample

The USB Camera Capture sample enables capturing from generic USB cameras. The
sample opens an X window showing the output of a camera. Note that this sample
is not designed to be used with a specific brand of USB cameras (such as Point
Grey), for which specific support may be provided through other samples. This
sample is compatible with Linux/Windows and DRIVE PX 2.

![Single consumer grade usb camera capturing](sample_camera_usb.png)

#### Testing

A camera must be connected to a USB port. The device ID, `N` of the camera, to
use can be specified using the `--device=N` command line option, which is set to
0 by default.

Default testing, of the first camera present on the system.

    ./sample_camera_usb

Default testing of the third camera on a system:

    ./sample_camera_usb --device=2
