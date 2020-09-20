# Install script for directory: /usr/local/driveworks/samples/src/image/image_streamer_cross_process

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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "samples")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sample_image_streamer_cross" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sample_image_streamer_cross")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sample_image_streamer_cross"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/usr/local/driveworks-0.6/samples/build_tracker/src/image/image_streamer_cross_process/sample_image_streamer_cross")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sample_image_streamer_cross" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sample_image_streamer_cross")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sample_image_streamer_cross"
         OLD_RPATH "/usr/local/cuda/targets/aarch64-linux/lib/stubs:/usr/local/cuda/targets/aarch64-linux/lib:/usr/local/driveworks-0.6/targets/aarch64-linux/lib:/usr/local/driveworks/samples/3rdparty/linux-aarch64/vibrante_Xlibs/lib:/home/xwang/NVIDIA/Drive/5050bL_SDK/DriveSDK/drive-t186ref-linux/lib-target:/usr/local/driveworks/samples/3rdparty/linux-aarch64/vibrante/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/home/xwang/NVIDIA/Drive/5050bL_SDK/DriveSDK/toolchains/tegra-4.9-nv/usr/bin/aarch64-gnu-linux/aarch64-gnu-linux-strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sample_image_streamer_cross")
    endif()
  endif()
endif()

