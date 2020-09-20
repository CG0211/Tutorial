# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Samples
#-------------------------------------------------------------------------------

project(sample_icp C CXX)

set(PUBLIC_DOCS
        README.md
        )

set(SOURCES
        PlyWriters.hpp
        main.cpp
        )

set(LIBRARIES
        ${Driveworks_LIBRARIES}
        dw_samples_framework
        )


#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
sdk_add_sample(${PROJECT_NAME})
