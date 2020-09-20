# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_hd_maps_access_sample HD Maps Access Sample

The maps sample demonstrates how to access map data from the maps module. The sample 
mimics a car driving along a GPS path, visualizing the map around the car.
The map geometry is stored as polylines of WGS84 coordinate points. It is shown
how to query the map data around a current position, how to transform the
geometry into a local cartesian coordinate space, and how to render that
data from various views.

![HD Maps Access Sample](sample_maps.png)

## Running the Sample

The command line for the sample is:

    ./sample_maps

The sample plays automatically, running through a fixed route of GPS points.
'Esc' closes the application.

## Output

The output window is split into 3 parts:
- left side showing an overview of complete map data
- middle showing a zoomed top view of the current location
- right side showing the current view from a car driving along the GPS path

Rendered elements:

- Blue lines = all lane divider lines
- White lines = local non-traversable lane divider lines
- Green lines = local traversable lane divider lines
- Grey lines = local invisible lane divider lines
- Red dot = current gps position

## Notes
This sample contains a DriveWorks map format representation of
map data from TomTom HD Maps.

To learn more about TomTom HD Maps, see: <br>
<a href="http://automotive.tomtom.com/products-services/hd-map-roaddna/">http://automotive.tomtom.com/products-services/hd-map-roaddna/</a>

