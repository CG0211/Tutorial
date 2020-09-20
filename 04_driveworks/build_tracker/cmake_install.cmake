# Install script for directory: /usr/local/driveworks/samples

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
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/framework/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/egomotion/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sensors/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/features/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/mapping/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/rigconfiguration/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/renderer/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/sfm/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/dnn/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/laneDetection/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/colorcorrection/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/isp/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/rectifier/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/ipc/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/hello_world/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/image/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/stereo/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/freespace/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/drivenet/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/maps/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/template/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/icp/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/lidar_accumulator/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/calibration/cmake_install.cmake")
  include("/usr/local/driveworks-0.6/samples/build_tracker/src/vehicleio/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/usr/local/driveworks-0.6/samples/build_tracker/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
