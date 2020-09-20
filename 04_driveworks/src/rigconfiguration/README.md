# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_rig_config_sample Rig Configuration Sample

The Rig Configuration sample shows how to read the rig configuration from the
XML file produced by the DriveWorks rig calibration tool.

#### Testing

NVIDIA provides a default rig configuration file and you can run the sample
without arguments. The output is printed to the console:

    ../sample_rig_configuration

    Vehicle Information:
     width:         1.874000
     height:        1.455000
     length:        4.915000
     wheelbase:     2.912000
    Sensor Count: 4
     Sensor 0:
      Name:         SVIEW_FR
      Protocol:     camera.gmsl
     Sensor 1:
      Name:         SVIEW_RE
      Protocol:     camera.gmsl
     Sensor 2:
      Name:         SVIEW_LE
      Protocol:     camera.gmsl
     Sensor 3:
      Name:         SVIEW_RI
      Protocol:     camera.gmsl

Adjust the rig configuration file with `--rigconfig=<input filename>`.
