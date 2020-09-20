/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2016-2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include <cstring>
#include <functional>
#include <list>
#include <iomanip>
#include <chrono>

#include <framework/ProgramArguments.hpp>
#include <framework/DataPath.hpp>
#include <framework/Log.hpp>

// CORE
#include <dw/core/Logger.h>
#include <dw/core/Context.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/imu/IMU.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static bool gRun = true;
static bool timestampTrace = false;
static dwTime_t lastTimestamp = 0;

//------------------------------------------------------------------------------
void sig_int_handler(int)
{
    gRun = false;
}

//------------------------------------------------------------------------------
void printAll(const dwIMUFrame &frame)
{
    bool containIMUInfo = false;

    if (timestampTrace) {
        std::cout << "[" << frame.timestamp_us << "] ";
        std::cout << "lastTimeStamp: " << lastTimestamp << " ";
        std::cout << "delta: " << frame.timestamp_us - lastTimestamp;
        std::cout << std::endl;

        lastTimestamp = frame.timestamp_us;
    } else {
        std::cout << "[" << frame.timestamp_us << "] ";

        // orientation
        if(frame.flags & (DW_IMU_ROLL|DW_IMU_PITCH|DW_IMU_YAW)){
            std::cout << "Orientation(";

            if(!std::isnan(frame.orientation[0]))
                std::cout << "R:" << frame.orientation[0] << " ";
            if(!std::isnan(frame.orientation[1]))
                std::cout << "P:" << frame.orientation[1] << " ";
            if(!std::isnan(frame.orientation[2]))
                std::cout << "Y:" << frame.orientation[2] << " ";

            std::cout << ") ";

            containIMUInfo = true;
        }

        // orientationQuaternion
        if(frame.flags & (DW_IMU_QUATERNION_X|DW_IMU_QUATERNION_Y|DW_IMU_QUATERNION_Z|DW_IMU_QUATERNION_W)){
            std::cout << "OrientationQuaternion(";

            if(!std::isnan(frame.orientationQuaternion[0]))
                std::cout << "X:" << frame.orientationQuaternion[0] << " ";
            if(!std::isnan(frame.orientationQuaternion[1]))
                std::cout << "Y:" << frame.orientationQuaternion[1] << " ";
            if(!std::isnan(frame.orientationQuaternion[2]))
                std::cout << "Z:" << frame.orientationQuaternion[2] << " ";
            if(!std::isnan(frame.orientationQuaternion[3]))
                std::cout << "W:" << frame.orientationQuaternion[3] << " ";

            std::cout << ") ";

            containIMUInfo = true;
        }

        // gyroscope
        if(frame.flags & (DW_IMU_ROLL_RATE|DW_IMU_PITCH_RATE|DW_IMU_YAW_RATE)){
            std::cout << "Gyro(";

            if(!std::isnan(frame.turnrate[0]))
                std::cout << "X:" << frame.turnrate[0] << " ";
            if(!std::isnan(frame.turnrate[1]))
                std::cout << "Y:" << frame.turnrate[1] << " ";
            if(!std::isnan(frame.turnrate[2]))
                std::cout << "Z:" << frame.turnrate[2] << " ";

            std::cout << ") ";
            containIMUInfo = true;
        }

        // heading (i.e. compass)
        if(frame.flags & DW_IMU_HEADING){

            std::cout << "Heading(";

            if(frame.headingType==DW_IMU_HEADING_TRUE)
                std::cout << "True:";
            else if(frame.headingType==DW_IMU_HEADING_MAGNECTIC)
                std::cout << "Magnetic:";
            else
                std::cout << "Unknown:";

            std::cout << frame.heading << ") ";

            containIMUInfo = true;
        }

        // acceleration
        if(frame.flags & (DW_IMU_ACCELERATION_X|DW_IMU_ACCELERATION_Y|DW_IMU_ACCELERATION_Z)){

            std::cout << "Acceleration(";

            if(!std::isnan(frame.acceleration[0]))
                std::cout << "X:" << frame.acceleration[0] << " ";
            if(!std::isnan(frame.acceleration[1]))
                std::cout << "Y:" << frame.acceleration[1] << " ";
            if(!std::isnan(frame.acceleration[2]))
                std::cout << "Z:" << frame.acceleration[2] << " ";

            std::cout << ") ";
            containIMUInfo = true;
        }

        // magnetometer
        if(frame.flags & (DW_IMU_MAGNETOMETER_X|DW_IMU_MAGNETOMETER_Y|DW_IMU_MAGNETOMETER_Z)){

            std::cout << "Magnetometer(";

            if(!std::isnan(frame.magnetometer[0]))
                std::cout << "X:" << frame.magnetometer[0] << " ";
            if(!std::isnan(frame.magnetometer[1]))
                std::cout << "Y:" << frame.magnetometer[1] << " ";
            if(!std::isnan(frame.magnetometer[2]))
                std::cout << "Z:" << frame.magnetometer[2] << " ";

            std::cout << ") ";
            containIMUInfo = true;
        }

        if (containIMUInfo==false)
            std::cout << "No IMU related info";

        std::cout << std::endl;
    }
}

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
#ifndef WINDOWS
    struct sigaction action = {};
    action.sa_handler = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
#endif

    gRun = true;

    ProgramArguments arguments(
        {ProgramArguments::Option_t("driver", "imu.virtual"),
         ProgramArguments::Option_t("params", (std::string("file=") +
                                               DataPath::get() + "/samples/sensors/imu/imu.txt")
                                                  .c_str()),
         ProgramArguments::Option_t("timestamp-trace", "false")});

    if (!arguments.parse(argc, argv) || (!arguments.has("driver") && !arguments.has("params"))) {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "\t--driver=imu.virtual \t\t\t: one of the available IMU drivers "
                  << "(see sample_sensors_info)\n";
        std::cout << "\t--params=file=file.txt,arg2=value \t: comma separated "
                  << "key=value parameters for the sensor "
                  << "(see sample_sensor_info for a set of supported parameters)";
        std::cout << "\t--timestamp-trace=<true/false>\t\t: enables timestamp tracing only\n";

        return -1;
    }

    dwContextHandle_t sdk   = DW_NULL_HANDLE;
    dwSALHandle_t hal       = DW_NULL_HANDLE;

    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwInitialize(&sdk, DW_VERSION, nullptr);

    // create HAL module of the SDK
    dwSAL_initialize(&hal, sdk);

    // create IMUM bus interface
    dwSensorHandle_t imuSensor = DW_NULL_HANDLE;
    {
        dwSensorParams params;
        std::string parameterString = arguments.get("params");
        params.parameters           = parameterString.c_str();
        params.protocol = arguments.get("driver").c_str();
        timestampTrace = arguments.get("timestamp-trace") == "true";
        if (dwSAL_createSensor(&imuSensor, params, hal) != DW_SUCCESS) {
            std::cout << "Cannot create sensor " << params.protocol
                      << " with " << params.parameters << std::endl;

            dwSAL_release(&hal);
            dwRelease(&sdk);
            dwLogger_release();

            return -1;
        }
    }

    gRun = dwSensor_start(imuSensor) == DW_SUCCESS;

    while (gRun) {

        dwIMUFrame frame;
        dwStatus status = dwSensorIMU_readFrame(&frame, 10000, imuSensor);

        if (status == DW_END_OF_STREAM) {
            std::cout << "IMU end of stream reached" << std::endl;
            break;
        } else if (status == DW_TIME_OUT)
            continue;
        else if (status == DW_SUCCESS){
            printAll(frame); // Print all IMU outputs
        }
    }

    dwSensor_stop(imuSensor);
    dwSAL_releaseSensor(&imuSensor);

    // release used objects in correct order
    dwSAL_release(&hal);
    dwRelease(&sdk);
    dwLogger_release();

    return 0;
}
