# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Samples
#-------------------------------------------------------------------------------
add_subdirectory(image_streamer_multi)
add_subdirectory(image_streamer_simple)
add_subdirectory(image_capture)
add_subdirectory(image_common)

if (VIBRANTE_V5L)
    add_subdirectory(image_streamer_cross_process)
endif()

