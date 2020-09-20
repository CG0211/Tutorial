# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_vehicleio_sample  VehicleIO Sample

![VehicleIO sample](sample_vehicleio.png)

This VehicleIO sample demonstrates the ability to read and write the state of
the vehicle actuators (i.e. throttle, brake, steering, etc.) It can talk to
different vehicle control systems, one of which is DataSpeed. VehicleIO
abstracts the underlying physical protocols and connections. All the protocol
handling is done by the internal "driver layer" interface, while the state is
handled by VehicleIO module.

This sample has been tested with CAN-based DataSpeed vehicle control system and
validated with Ford MKZ and Ford Fusion vehicles. In order to send the
steering, brake or throttle commands, one can use a keyboard or a USB joystick
(tested with Logitech Gamepad F310). A few important Keyboard mappings are
listed below (please see main.cpp for all the keyboard and joystick mappings):

    E - enable the vehicle control system
    D - disable the vehicle control system
    Arrow key UP - incrementel throttle with every arrow key UP press
    Arrow key DOWN - incremental braking with every arrow key DOWN press
    Arrow key LEFT - incremental steering rotation (counter-clockwise) with every key press
    Arrow key RIGHT - incremental steering rotation (clockwise) with every key press

#### Running the Sample

To use with the pre-recorded CAN data, which is provided with the sample, one
can just run the sample without any arguments:

    ./sample_vehicleio

To use a custom pre-recorded CAN data, one can use the following command:

    ./sample_vehicleio --driver=can.virtual --params=file=CAN.bin

For live CAN (in a vehicle), one can use the following command:

    ./sample_vehicleio --driver=can.socket --params=device=can0

#### Output

The sample opens an X window to display the various vehicle control parameters
such as throttle, braking, steering, etc. Once the window is in the focus, the
throttle, brakes, steering of the vehicle can be controlled via a keyboard or a
USB joystick, as described above.

The basic vehicle state information is also being printed to the terminal.
