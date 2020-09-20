# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.


# EXPERIMENTAL SAMPLE
# project(sample_experimental_template C CXX)

# PUBLIC SAMPLE
project(sample_template C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(SOURCES
    main.cpp
)

set(LIBRARIES
    dw_samples_framework
    ${Driveworks_LIBRARIES}
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target - UNCOMMENT FOR INSTALLATION
# ------------------------------------------------------------------------------
#sdk_add_sample(${PROJECT_NAME})
