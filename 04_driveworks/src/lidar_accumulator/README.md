# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_lidar_accumulator_sample Lidar Accumulator Sample

![Lidar Accumulator Sample](sample_lidar_accumulator.png)

The Lidar Accumulator sample demonstrates how to use LidarAccumulator module to process and collect data from decoded Lidar packets.
The sample shows 2 modes: 1) Sync mode adds the Lidar packet to the accumulator, lock
the sweep buffer and do rendering. 2) Aysnc mode creates a dedicated thread
 which adds Lidar packets to LidarAccumulator, the main thread keeps querying whether the full sweep is ready. Besides the packet collection, LidarAccumulator module allows the user to select Lidar data within certain range and certain spin angles. The data made available to the user consists of two types: Lidar point clouds in Cartesian XYZ space and cylindrical projection image. User can toggle in the application to choose the type of the cylindrical projection image. There are three types of  the image are supported: DW_LIDAR_IMAGE_TYPE_3D_DISTANCE_IMAGE, DW_LIDAR_IMAGE_TYPE_2D_DISTANCE_IMAGE and DW_LIDAR_IMAGE_TYPE_INTENSITY_IMAGE. By definition, it means the image pixel represents the 3D distance in XYZ space, 2D distance in XY plane and Lidar intensity. Currently LidarAccumulator supports Velodyne HDL64E, Velodyne HDL32E, Velodyne VLP16 and Quanergy M81A. 

#### Running the Sample

The command line for the sample to run with live Lidar sensor is:

    ./sample_lidar_accumulator --ip=[Lidar IP address] --port=[Lidar port]
    --device=[type of device] --scan-frequency=[valid frequency]
    --mode=[sync or async]
    --maxDistance[max distance in meters to select the Lidar points]
    --minDistance[min distance in meters to select the Lidar points]
    --minAngle[min angle in degrees to select the Lidar points]
    --maxAngle[max angle in degrees to select the Lidar points]
    --smooth-window-size[1, 2, 4, 8]
    --lidar-image-type[depth-xyz, depth-xy or intensity]

- The Lidar must be connected to the network.
- Currently supported devices are Quanergy M8 [QUAN_M81A], 
  Velodyne VPL16 [VELO_VPL16], Velodyne HDL32 [VELO_HDL32E] and Velodyne HDL64E [VELO_HDL64E]
- Scan frequency is usually preset using the Lidar vendor tools. 
- maxDistance and minDistance specify the distance range in meters for the Lidar Accumulator to collect Lidar points
- minAngle and maxAngle set the angular span for the Lidar Accumulator
- lidar-image-type specifies the type of cylindrical image. Current supported types include depth-xyz, depth-xy and intensity images.
- smooth-window-size allows the user to smooth the Lidar points jittering in horizontal direction due to the spinning noise. For example, for Velodyne HDL64E each Lidar beam has 3480 points in dual mode. With smooth-window-size=4, Lidar Accumulator chooses one single point out of every 4 adjacent Lidar points based on closest 3D distance from the Lidar sensor. 

The command line for the sample to run with recorded Lidar binary file is:

    ./sample_lidar_accumulator --file=[Lidar binary file] --mode=[sync or async]
    --maxDistance[max distance in meters to select the Lidar points]
    --minDistance[min distance in meters to select the Lidar points]
    --minAngle[min angle in degrees to select the Lidar points]
    --maxAngle[max angle in degrees to select the Lidar points]
    --smooth-window-size[1, 2, 4, 8]
    --lidar-image-type[depth-xyz, depth-xy or intensity]

- The Lidar file can be obtained with the provided recording tools.
- If no arguments are passed, a default Lidar file is loaded and default parameters are used.

#### Output

The sample opens an X window to display a full sweep of 3D point cloud, a rolling sector sweep of the 3D point cloud and a projected cylindrical image on the bottom. Use the mouse to
interact with the visualization:

- Mouse left button: rotate the point cloud
- Mouse wheel: zoom in or out
- Key 0: show the intensity image
- Key 1: show the depth image
- Left Key: rotate the sector spin in counter-clock direction
- Right Key: rotate the sector spin in clock direction
