# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_gps_loc_sample GPS Location Logger Sample

The GPS Location Logger sample works with any serial port (UART) based GPS
sensor or with the Xsens GPS device connected over USB. 
The logger requires the GPS sensor connected over serial port to deliver messages 
in NMEA format, while the Xsens device connected over USB can run in proprietary mode.

#### Testing

Test the applications using consumer grade off-the-shelve GPS sensors, also
known as GPS mouse (e.g., Garmin GPS).

Usually, these mice are connected over USB and implement a serial-to-USB
connection with the help of FTDI devices. On Linux, these sensors can then be
reached over the `/dev/ttyUSB` or `/dev/ttyACM` devices.

The sample requires the name of the tty device of the GPS sensor, for example:

    ./sample_gps_logger --driver=gps.uart --params=device=/dev/ttyACM0

Any valid GPS message that is received results in an output on the
console similar to:

    GPS[0] - 2671445399 lat: 37.3901 lon: -122.163 alt: 71.2 speed: 17.131 course: 190.6

Where the first number indicates the timestamp of the received GPS message in
microseconds and the rest of the line indicates the geographical location of the
sensor.

If no parameters are provided, the sample starts a virtual GPS sensor and
interprets the content from the file located at `data/samples/sensors/gps/1.gps`
as GPS input.

##### Baudrate

Per default, if no `baud` parameter has been provided `gps.uart` driver assumes
a baudrate of 9600. In order to change the baudrate provide `baud` argument as:

    ./sample_gps_logger --driver=gps.uart --params=device=/dev/ttyACM0,baud=115200

##### Xsens GPS[/IMU]

The sample support reading GPS packets from a Xsens device through the `gps.xsens`
driver. The driver expects Xsens device to deliver packets in Xsens proprietary format.
To run the sample using Xsens device use:

    ./sample_gps_logger --driver=gps.xsens --params=device=0,frequency=100

Where `device=0` parameter sets the index of the Xsens device (usually 0 if only 
one device is installed) and `frequency=100` sets the frequency in `[Hz]` this device should
operate with. Please note that even if the Xsens device is a shared device, like
Xsens MTi-G-700, capable of delivering GPS and IMU packets, only the GPS packets
will be parsed by the `gps.xsens` driver. 

#### Sensor sharing

The sample also demonstrates how sensor sharing can be implemented. Two DriveWorks
GPS sensors are created from the same hardware device. Both sensor can be then
treated independently. Both sensors would deliver exactly the same set of
packets. Each sensor is using, however, their own FIFO hence they can be drained
at different rates. The output of both sensors is printed to the console:

    GPS[0] - 2671445399 lat: 37.3901 lon: -122.163 alt: 71.2 speed: 17.131 course: 190.6
    GPS[1] - 2671445399 lat: 37.3901 lon: -122.163 alt: 71.2 speed: 17.131 course: 190.6

The index [0], [1] indicates what sensor produced the output. As expected the data
packets and their timestamps are equal.
