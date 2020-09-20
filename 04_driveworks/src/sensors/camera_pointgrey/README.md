@page dwx_camera_pointgrey_sample Point Grey USB Camera Capture Sample

The Point Grey USB Camera Capture sample enables capturing from Point Grey USB cameras. The sample opens
an X window showing the output of a camera. Note that this sample was designed to
be used with the Chameleon USB camera series, although support may be extended to other families of Point Grey USB cameras.
This sample is compatible with Linux/Windows and DRIVE PX 2.

![Single consumer grade Point Grey USB camera capture](sample_camera_pointgrey.png)

#### Prerequisites ####

Follow these steps before running `sample_camera_pointgrey` on DRIVE PX 2:

1. Download the FlyCapture2 SDK from https://www.ptgrey.com/support/downloads/10616/

2. Extract the package and perform the following steps:

        cd flycapture.2.9.3.43_arm64
        sudo sh flycap2-conf
        cd lib
        sudo cp libflycapture* /usr/lib/
        cd C
        sudo cp * /usr/lib
    This configures the udev and copies the required shared library files to `/usr/lib`.
3. Add the following lines to `/etc/sysctl.conf`:
	
        net.core.rmem_default = 1048576
        net.core.rmem_max = 10485760
        net.core.wmem_default = 1048576
        net.core.wmem_max = 10485760
        net.core.netdev_max_backlog = 30000
        net.ipv4.ipfrag_high_thresh = 8388608
        net.ipv6.conf.all.disable_ipv6 = 1
        vm.dirty_background_ratio = 5
        vm.dirty_ratio = 80

    A restart may be required before the changes take effect.
4. On every boot, the following must be executed:

        sudo -s
        echo 1000  > /sys/module/usbcore/parameters/usbfs_memory_mb

#### Testing

A camera must be connected to a USB port. The device ID, `N` of the camera to use can be specified using the
`--device=N` command line option, which is set to 0 by default.

Default testing of the first camera present on the system:

    ./sample_camera_pointgrey

Default testing of the third camera on a system:

    ./sample_camera_pointgrey --device=2
