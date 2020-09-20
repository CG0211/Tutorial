# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_record_sample Simple Sensor Recording Sample

The recording sample allows you to record data from CAN, GPS, or
LiDAR sensors.

## Running the Sample

The syntax for calling the simple sensor recording sample is:

    ./sample_record <output_file> [options]

One or more of the following output file options is required:

    --write-file-gps=/path/to/file.gps     File for recording GPS data.
    --write-file-can=/path/to/canbusfile   File for recording CAN data.
    --write-file-lidar=/path/to/lidarfile  File for recording Lidar data.
    --write-file-radar=/path/to/radarfile  File for recording Radar data.

Additional options are:

    --can-driver=can.socket                        CAN driver to open (default=can.socket).
    --can-params=device=can0,bus=d                 Parameters passed to CAN sensor.
    --gps-driver=gps.uart                          GPS sensor driver (default=gps.uart).
    --gps-params=device=/dev/ttyACM0               Parameters passed to GPS sensor.
    --lidar-driver=lidar.virtual                   Lidar sensor driver (default=lidar.virtual).
    --lidar-params=device=QUAN_M18A,file=filename  Parameters passed to LIDAR sensor.
    --radar-driver=radar.virtual                   Radar sensor driver (default=radar.virtual).
    --radar-params=device=DELPHI_ESR2_5,file=filename  Parameters passed to RADAR sensor.

## Recording CAN ##

#### To record CAN ####

- Set `--can-driver` to `can.virtual` for recording from file
  or `can.socket` for live recording.
- Set `--can-params` to `file=filename.bin`, where
  filename.bin is the file name of the CAN data for virtual or `device=can0`
  and where `can0` is the can device to live record.
- Set  `--write-file-can=filename.bin` to the recorded output file,
  where filename.bin is the output file for
  CAN data.

Thus, to record data from a virtual can sensor, the following command would be used:

    ./sample_record --can-driver=can.virtual --can-params=file=/path/to/canfile.bin --write-file-can=/path/to/outputfile.bin


## Recording GPS and LIDAR ##

GPS and LIDAR take command-line options that are similar to the CAN options.

For example, the command line for recording gps data from a virtual gps sensor is:

    ./sample_record --gps-driver=gps.virtual --gps-params=file=/path/to/gpsfile.bin --write-file-gps=/path/to/outputfile.bin

## Recording Mixed Sensor Types

Different types of sensors can be combined. For example, the command line for
recording GPS data from a live GPS sensor is and lidar data from a virtual sensor is:

    ./sample_record --gps-driver=gps.uart --gps-params=device=/dev/ttyACM0 --write-file-gps=/path/to/gpsoutput.bin --lidar-driver=lidar.virtual --lidar-params=file=/path/to/lidarfile.bin --write-file-lidar=/path/to/lidaroutput.bin


@note This sample creates output files that, per default, are put into the
current working directory. Hence, write permissions to the current working
directory are necessary if the output file arguments are not changed.
