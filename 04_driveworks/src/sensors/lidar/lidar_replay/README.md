# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_lidar_replay_sample Lidar Point Clouds Sample

![Lidar Point Clouds Sample](sample_lidar_replay.png)

The Lidar Replay sample demonstrates how to connect to a Lidar and visualizes
the generated point cloud in 3D.

#### Running the Sample

The command line for the sample to display live Lidar point clouds is:

    ./sample_lidar_replay --protocol=[lidar protocol] --params=device=[type of device],ip=[lidar IP address],port=[lidar port],scan-frequency=[valid frequency] (--show-intensity=[true])

- The Lidar must be up and running, and connected to the network.
- Currently supported protocols are lidar.virtual for replay from file, and
  lidar.socket for live Lidar replay
- Currently supported devices are Quanergy M8 [QUAN_M81A], IBEO Lux [IBEO_LUX],
  Velodyne VPL16 [VELO_VPL16], Velodyne HDL32 [VELO_HDL32E] and Velodyne HDL64E
  [VELO_HDL64E]
- Scan frequency is usually preset using the Lidar vendor tools. The exception
  is the IBEO Lux Lidar, where the value provided is used to set the sensor
  frequency (valid values are 12.5, 25, and 50 Hz)

The command line for the sample to display live Lidar point clouds is:

    ./sample_lidar_replay --file=[lidar file]

- The Lidar file can be obtained with the provided recording tools.
- If no arguments are passed, a default Lidar file is loaded.

#### Output

The sample opens an X window to display a 3D point cloud. Use the mouse to
interact with the visualization:

- Mouse left button: rotate the point cloud
- Mouse wheel: zoom in or out

Color is rendered by 2D distance from the origin where red is near and blue is far. An alternative hue based rendering mode is available by setting --show-intensity=true flag in the commnand line. This mode renders
intensity porportional to wavelength with higher intensities in blue and lower intensities in red
