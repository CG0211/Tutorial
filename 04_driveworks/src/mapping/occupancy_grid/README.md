# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_occupancy_grid_sample Occupancy Grid Mapping Sample

The Occupancy Grid Mapping sample showcases a Bayesian occupancy grid. The
sample allocates a grid and allows the application to insert point clouds or
object lists, which are polygons stored as a list of points, on a
stationary-to-moving vehicle. You can observe the point clouds or objects being
accumulated, and as the vehicle moves, the grid is updated according to the
probabilities given in the initialization.

![Occupancy Grid Mapping sample](sample_occupancy_grid.png)

In the initialization of each layer of the occupancy grid, a set of
probabilities is specified. For point clouds, these probabilities correspond to
the actual probability of a grid cell being free at the actual point, the
probability that sensor origin is free space, and the probability of free space
beyond the point. Point clouds are inserted by casting a ray from the sensor
origin to the point. For object lists, only the probability that a cell is free
at the object is specified. Additionally, each layer specifies the sensor to
sensor rig transformation so that when the points are inserted, they are
oriented correctly in the grid.

The sample application uses three sensors in combination: CAN data for the car
position and orientation, Lidar for point cloud data, and camera for
visualization.

## Running the sample

Test the application directly by running it:

    ./sample_occupancy_grid

Alternatively, the sample
application accepts custom inputs for lidar, can, and camera data:

    ./sample_occupancy_grid --canFile=can.bin --dbcFile=can.dbc --lidarFile=lidar.bin --videoFile=video.h264 --videoTimestampFile=video_time.txt --fps=30

The following is an explanation of each argument:
@code
--canFile -             The recorded can data
--dbcFile -             DBC file for interpreting can
--lidarFile -           The recorded lidar data
--videoFile -           The recorded h264 video
--videoTimestampFile -  The timestamp file associated with the videoFile
--fps -                 The speed at which the sample plays
@endcode

All inputs are assumed to be in the same format that the DriveWorks recording tool uses when recording.
The videoFile must be h264 and must have an associated videoTimestampFile in order to be able
to synchronize with the other sensors properly. Additionally, the sensors must be recorded
at the same time so that they can be synchronized and played back together.

## Output
The expected output is a Bayesian occupancy grid with areas that are free colored white, and areas that
are not free as black. Everything unknown is gray. The colors represent the probability of a cell
being free.

Test the application directly by running it. Alternatively, the sample
application accepts custom inputs for Lidar, CAN, and camera data.

