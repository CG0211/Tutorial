# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

project(sample_camera_gmsl_custom C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(HEADERS
    captureConfig.h
)

set(SOURCES
    captureConfig.cpp
    main.cpp
)

set(LIBRARIES
    dw_samples_framework
    ${Driveworks_LIBRARIES}
    lodepng
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
sdk_add_sample(${PROJECT_NAME})
sdk_install_shared(data/samples/sensors/camera camera.cfg samples)
