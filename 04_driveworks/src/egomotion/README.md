# Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_egomotion_sample Egomotion Sample

The Egomotion sample shows how to use CAN measurements of steering
angle and velocity as well as IMU measurements to compute the position of the car 
in the world coordinate system. It uses the `can.virtual` sensor to read CAN messages 
from disk and interprets it via the sample CAN interpreter. 
The position of the car is plotted on the screen at a 30 Hz sampling rate.

![Egomotion Sample](sample_egomotion.png)

#### Testing

NVIDIA provides a default dataset files; the sample can be run without
arguments. The output is shown on the screen.

    ./sample_egomotion

Optionally, the egomotion results can be written to a CSV file by
setting the --output argument. In this case, the last lines are:

    ....
    5680571648,-7.67,118.30
    5680604981,-7.75,118.33
    5680638314,-7.83,118.36
    5680671647,-7.91,118.38
    5680704980,-7.98,118.41


Adjust the output file with `--output=<output filename>`.

#### Odometry+IMU
The sample supports different modes of egomotion estimation. The mode can be switched
by passing `--mode=0/1/2` argument. Mode 0 represent odometry based egomotion 
estimation. The vehicle motion is estimated using Ackerman principle. 

Mode 1 is using IMU measurements to estimate motion of the vehicle. Gyroscope
and linear accelerometers are filtered and fused to estimate vehicle orientation.
Using odometry reading of speed vehicle traveling path can be estimated.

Mode 2 has same motion behaviour as mode 1, however it also filters GPS
locations which are passed to the module as well. 

For more details about individual egomotion algorithms refer to
the documentation of the `dwEgomotion` module.


#### Arguments
The application expects you to provide the CAN file containing the CAN messages
with the speed and steering angle, and a DBC file that contains the information on how to
interpret the CAN messages. It is expected that the interpretation in the DBC
file maps the speed to `[m/s]` units and steering angle to `[rad]`, with
negative values representing right steering angles and positive left steering
angles.

Using arguments `--speedFactor` and `--steeringFactor` the interpretation of the
DBC file can be adjusted. In order for the sample application to understand what
part of the DBC file represents the speed and the steering angle, you must
pass the name of the signal for the car speed in `--dbcSpeed`, e.g., `--dbcSpeed
M_SPEED.S_WHEEL_SPEED`, where `M_SPEED` is the name of the message and
`S_WHEEL_SPEED` is the name of the signal within the `M_SPEED` message that
represents the speed in `[m/s]`.


