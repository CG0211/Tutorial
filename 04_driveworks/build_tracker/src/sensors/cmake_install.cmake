# Install script for directory: /usr/local/driveworks/samples/src/sensors

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local/driveworks-0.6/samples/build_tracker/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/info/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/gps/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/imu/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_replay/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_seek/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_multiple_replay/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_usb/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_pointgrey/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/record/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/canbus/logger/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/canbus/interpreter/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/lidar/lidar_replay/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/radar/radar_replay/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/third_party_camera/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_gmsl/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_gmsl_custom/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_multiple_gmsl/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/camera_gmsl_raw/cmake_install.cmake")

endif()

