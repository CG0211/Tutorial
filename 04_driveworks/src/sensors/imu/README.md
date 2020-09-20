# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_imu_loc_sample IMU Logger Sample

The IMU Logger sample works with any serial port (UART) based IMU sensor. The
logger requires the IMU sensor to deliver messages in NMEA format.

#### Testing

Test the applications using consumer grade off-the-shelve IMU sensors.

Usually, these IMUs are connected over USB and implement a serial-to-USB
connection with the help of FTDI devices. On Linux, these sensors can then be
reached over the /dev/ttyUSB or /dev/ttyACM devices.

Check the status of IMU first. For example:

    sudo stty -F /dev/ttyUSB0 raw -echo -echoe -echok [Optional baud rate, like 115200]
    sudo cat /dev/ttyUSB0

You should see raw NMEA sentences with the cat before continuing.

The sample requires the name of the tty device of the IMU sensor, for example:

    ./sample_imu_logger --driver=imu.uart --params=device=/dev/ttyUSB0,baud=115200

For every valid IMU message that the sample receives, it prints to the console
data such as:

@code
    [7156364888] Heading(True:112.07)
    [7156364959] Gyro(Z:0.0756667 )
    [7156369081] Orientation(R:-1.3 P:-0.9 ) Gyro(X:-0.01 Y:-0.05 ) Heading(True:112.1)
    [7156389724] Heading(Magnetic:112.07)
    [1475788068778749] Heading(Magnetic:112.1)
    [7156389797] Orientation(R:-1.31 P:-0.89 )
    [1475788068778919] Orientation(R:-1.30529 P:-0.893047 Y:112.078 ) Gyro(X:0.0229183 Y:-0.0687549 Z:0.120321 ) Acceleration(X:-0.1398 Y:-0.2612 Z:9.7838 ) Magnetometer(X:7.18 Y:3.056 Z:-16.16 )
@endcode

Where the first number indicates the timestamp of the received IMU
message in microseconds and the rest of the line indicates the IMU information
of the sensor.

If no parameters are provided, the sample starts a virtual IMU sensor and
interprets the content from the file located at
`data/samples/sensors/imu/imu.txt` as IMU input. Or give it a file:

    ./sample_imu_logger --driver=imu.virtual --params=file=[pathToData]/imu.txt


##### Baudrate

Per default, if no `baud` parameter has been provided `imu.uart` driver assumes
a baudrate of 9600. In order to change the baudrate provide `baud` argument as:

    ./sample_imu_logger --driver=imu.uart --params=device=/dev/ttyUSB0,baud=115200

##### Xsens IMU[/GPS]

The sample support reading IMU packets from a Xsens device through the `imu.xsens`
driver. The driver expects Xsens device to deliver packets in Xsens proprietary format.
To run the sample using Xsens device use:

    ./sample_imu_logger --driver=imu.xsens --params=device=0,frequency=100

Where `device=0` parameter sets the index of the Xsens device (usually 0 if only
one device is installed) and `frequency=100` sets the frequency in `[Hz]` this device should
operate with.

@note Even if the Xsens device is a shared device, like
Xsens MTi-G-700, capable of delivering GPS and IMU packets, only the IMU packets
will be parsed by the `imu.xsens` driver.
