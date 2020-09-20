# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

project(sample_ipc_socketclientserver C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(LIBRARIES
    dw_samples_framework
    ${Driveworks_LIBRARIES}
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} sample_ipc_socketclientserver.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
sdk_add_sample(${PROJECT_NAME})
