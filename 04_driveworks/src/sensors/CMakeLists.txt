# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Samples
#-------------------------------------------------------------------------------
add_subdirectory(info)
add_subdirectory(gps)
add_subdirectory(imu)
add_subdirectory(camera_replay)
add_subdirectory(camera_seek)
add_subdirectory(camera_multiple_replay)
add_subdirectory(camera_usb)
add_subdirectory(camera_pointgrey)
add_subdirectory(record)
add_subdirectory(canbus/logger)
add_subdirectory(canbus/interpreter)
add_subdirectory(lidar/lidar_replay)
add_subdirectory(radar/radar_replay)
add_subdirectory(third_party_camera)

if (VIBRANTE_V5L)
    add_subdirectory(camera_gmsl)
    add_subdirectory(camera_gmsl_custom)
    add_subdirectory(camera_multiple_gmsl)
    add_subdirectory(camera_gmsl_raw)
endif()

