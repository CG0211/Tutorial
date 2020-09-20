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
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <thread>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <framework/ProgramArguments.hpp>
#include <framework/Log.hpp>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/gps/GPS.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/lidar/Lidar.h>
#include <dw/sensors/radar/Radar.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static bool gRun = true;
static std::mutex gMutex;

//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;

    std::lock_guard<std::mutex> lock(gMutex);

    gRun = false;
}

//------------------------------------------------------------------------------
void run_gps(ProgramArguments arguments, dwSALHandle_t sal)
{
    dwSensorHandle_t gpsSensor            = DW_NULL_HANDLE;
    dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
    {
        std::lock_guard<std::mutex> lock(gMutex);

        std::string gpsDriver = "gps.uart";
        std::string gpsParams = "";

        if (arguments.get("gps-driver") != "") {
            gpsDriver = arguments.get("gps-driver");
        }
        if (arguments.get("gps-params") != "") {
            gpsParams = arguments.get("gps-params");
        }

        // create GPS bus interface
        {
            dwSensorParams params;
            params.parameters = gpsParams.c_str();
            params.protocol = gpsDriver.c_str();
            if (dwSAL_createSensor(&gpsSensor, params, sal) != DW_SUCCESS) {
                std::cout << "Cannot create sensor "
                          << params.protocol << " with "
                          << params.parameters << std::endl;

                return;
            }
        }

        std::string newParams = "";
        if (arguments.has("write-file-gps")) {
            newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-gps"));
        }
        dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
        dwSensorSerializer_initialize(&serializer, &serializerParams, gpsSensor);
        dwSensorSerializer_start(serializer);

        gRun = dwSensor_start(gpsSensor) == DW_SUCCESS;
    }

    // Message msg;
    while (gRun) {
        std::this_thread::yield();

        const uint8_t *data = nullptr;
        size_t size         = 0;

        dwStatus status = DW_NOT_READY;
        while (status == DW_NOT_READY) {
            status = dwSensor_readRawData(&data, &size, 1000000, gpsSensor);
            if (status != DW_SUCCESS) {
                break;
            }
            status = dwSensorGPS_processRawData(data, size, gpsSensor);

            dwSensorSerializer_serializeDataAsync(data, size, serializer);

            dwSensor_returnRawData(data, gpsSensor);
        }

        if (status == DW_END_OF_STREAM) {
            std::cout << "GPS end of stream reached" << std::endl;
            break;
        } else if (status == DW_TIME_OUT)
            continue;

        // if previous process raw data has not failed, then we must have valid message available
        dwGPSFrame frame{};
        if (dwSensorGPS_popFrame(&frame, gpsSensor) != DW_SUCCESS)
        {
            std::cerr << "GPS message was not found in the raw stream" << std::endl;
            continue;
        }

        // log message
        gMutex.lock();
        std::cout << frame.timestamp_us;
        if (status != DW_SUCCESS) // msg.is_error)
        {
            std::cout << " ERROR " << dwGetStatusName(status); // msg.frame.id;
        } else {
            std::cout << std::setprecision(10)
                      << " lat: " << frame.latitude
                      << " lon: " << frame.longitude
                      << " alt: " << frame.altitude
                      << " course: " << frame.course
                      << " speed: " << frame.speed
                      << " climb: " << frame.climb
                      << " hdop: " << frame.hdop
                      << " vdop: " << frame.vdop;
        }
        std::cout << std::endl;
        gMutex.unlock();
    }

    dwSensorSerializer_stop(serializer);

    dwSensorSerializer_release(&serializer);

    dwSensor_stop(gpsSensor);

    dwSAL_releaseSensor(&gpsSensor);
}

//------------------------------------------------------------------------------
void run_can(ProgramArguments arguments, dwSALHandle_t sal)
{
    dwSensorHandle_t canSensor            = DW_NULL_HANDLE;
    dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
    {
        std::lock_guard<std::mutex> lock(gMutex);
        std::string canParam  = "";
        std::string canDriver = "can.socket";

        if (arguments.get("can-driver") != "") {
            canDriver = arguments.get("can-driver");
        }
        if (arguments.get("can-params") != "") {
            canParam = arguments.get("can-params");
        }
        canParam += ",serialize=true";

        // create CAN bus interface
        {
            dwSensorParams canparams;
            canparams.parameters = canParam.c_str();
            canparams.protocol = canDriver.c_str();
            if (dwSAL_createSensor(&canSensor, canparams, sal) != DW_SUCCESS) {
                std::cout << "Cannot create sensor " << canparams.protocol << " with "
                          << canparams.parameters << std::endl;

                return;
            }
        }

        std::string newParams = "";
        if (arguments.has("write-file-can")) {
            newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-can"));
        }
        dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
        dwSensorSerializer_initialize(&serializer, &serializerParams, canSensor);
        dwSensorSerializer_start(serializer);
        gRun = dwSensor_start(canSensor) == DW_SUCCESS;
    }

    // Message msg;
    while (gRun) {
        dwCANMessage msg{};

        const uint8_t *data = nullptr;
        size_t size         = 0;

        dwStatus status = DW_NOT_READY;
        while (status == DW_NOT_READY) {
            status = dwSensor_readRawData(&data, &size, 100000, canSensor);
            if (status != DW_SUCCESS) {
                break;
            }

            status = dwSensorCAN_processRawData(data, size, canSensor);

            dwSensorSerializer_serializeDataAsync(data, size, serializer);

            dwSensor_returnRawData(data, canSensor);

            status = status == DW_SUCCESS ? dwSensorCAN_popMessage(&msg, canSensor) : status;
        }

        if (status == DW_END_OF_STREAM) {
            std::cout << "CAN end of stream reached" << std::endl;
            break;
        } else if (status == DW_TIME_OUT)
            continue;

        // log message
        gMutex.lock();
        std::cout << msg.timestamp_us;
        if (status != DW_SUCCESS) // msg.is_error)
        {
            std::cout << " ERROR " << dwGetStatusName(status); // msg.frame.id;
        } else {
            std::cout << " [0x" << std::hex << msg.id << "] -> ";
            for (auto i = 0; i < msg.size; i++)
                std::cout << "0x" << std::hex << (int)msg.data[i] << " ";
            std::cout << std::dec;
        }
        std::cout << std::endl;
        gMutex.unlock();
    }

    dwSensorSerializer_stop(serializer);
    dwSensorSerializer_release(&serializer);
    dwSensor_stop(canSensor);

    dwSAL_releaseSensor(&canSensor);
}

//------------------------------------------------------------------------------
void run_lidar(ProgramArguments arguments, dwSALHandle_t sal)
{
    dwSensorHandle_t lidarSensor          = DW_NULL_HANDLE;
    dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
    {
        std::lock_guard<std::mutex> lock(gMutex);

        std::string lidarDriver = "lidar.uart";
        std::string lidarParams = "";

        if (arguments.get("lidar-driver") != "") {
            lidarDriver = arguments.get("lidar-driver");
        }
        if (arguments.get("lidar-params") != "") {
            lidarParams = arguments.get("lidar-params");
        }

        // create lidar bus interface
        {
            dwSensorParams params;
            params.parameters = lidarParams.c_str();
            params.protocol = lidarDriver.c_str();
            if (dwSAL_createSensor(&lidarSensor, params, sal) != DW_SUCCESS) {
                std::cout << "Cannot create sensor "
                          << params.protocol << " with "
                          << params.parameters << std::endl;

                return;
            }
        }

        std::string newParams = "";
        if (arguments.has("write-file-lidar")) {
            newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-lidar"));
        }
        dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
        dwSensorSerializer_initialize(&serializer, &serializerParams, lidarSensor);
        dwSensorSerializer_start(serializer);
    }

    // To serialize one needs to disable decoding
    dwSensorLidar_disableDecoding(lidarSensor);

    dwLidarProperties    lidarProperties;

    dwSensorLidar_getProperties(&lidarProperties, lidarSensor);

    const dwLidarDecodedPacket *frame;
    
    // Message msg;
    int packetCount = 0;
    auto start = std::chrono::high_resolution_clock::now();
    std::chrono::microseconds usec(1);

    gRun = dwSensor_start(lidarSensor) == DW_SUCCESS;

    while (gRun) {
        const uint8_t *data = nullptr;
        size_t size         = 0;

        dwStatus status = DW_SUCCESS;
        
        status = dwSensor_readRawData(&data, &size, 5000, lidarSensor);
        
        if (status == DW_SUCCESS) {

            // If we are reading too fast, we can throttle here. real sensors are slower than the
            // disk write speed. Virtual sensors can be a lot faster
            do
                status = dwSensorSerializer_serializeDataAsync(data, size, serializer);
            while(status == DW_NOT_AVAILABLE || status == DW_BUFFER_FULL);

            if (status != DW_SUCCESS) {
                std::cout << "Error serializing packet " << packetCount << std::endl;
            }

            // Here we can optionally decode the packet
            status = dwSensorLidar_processRawData(&frame, data, size, lidarSensor);

            status = dwSensor_returnRawData(data, lidarSensor);
            if (status != DW_SUCCESS) {
                std::cout << "Error returning packet " << packetCount << std::endl;
            }
            packetCount++;

            if(packetCount % 10 == 0) std::cout << "Processed " << packetCount << std::endl;
        }
        else if (status == DW_END_OF_STREAM) {
            std::cout << "lidar end of stream reached" << std::endl;
            break;
        }
        else
        {
            if (status != DW_TIME_OUT) std::cout << "Error reading packet " << packetCount << std::endl;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end-start;
    std::cout << "Processed " << packetCount << " packets in " << elapsed.count() / 1000.0 << " s\n";

    dwSensorSerializer_stop(serializer);

    dwSensorSerializer_release(&serializer);

    dwSensor_stop(lidarSensor);

    dwSAL_releaseSensor(&lidarSensor);
}


//------------------------------------------------------------------------------
void run_radar(ProgramArguments arguments, dwSALHandle_t sal)
{
    dwSensorHandle_t radarSensor          = DW_NULL_HANDLE;
    dwSensorSerializerHandle_t serializer = DW_NULL_HANDLE;
    {
        std::lock_guard<std::mutex> lock(gMutex);

        std::string radarDriver = "radar.socket";
        std::string radarParams = "";

        if (arguments.get("radar-driver") != "") {
            radarDriver = arguments.get("radar-driver");
        }
        if (arguments.get("radar-params") != "") {
            radarParams = arguments.get("radar-params");
        }

        // create radar bus interface
        {
            dwSensorParams params;
            params.parameters = radarParams.c_str();
            params.protocol = radarDriver.c_str();
            if (dwSAL_createSensor(&radarSensor, params, sal) != DW_SUCCESS) {
                std::cout << "Cannot create sensor "
                          << params.protocol << " with "
                          << params.parameters << std::endl;

                return;
            }
        }
        std::string newParams = "";
        if (arguments.has("write-file-radar")) {
            newParams += std::string("type=disk,file=") + std::string(arguments.get("write-file-radar"));
        }
        dwSerializerParams serializerParams = {newParams.c_str(), nullptr, nullptr};
        dwSensorSerializer_initialize(&serializer, &serializerParams, radarSensor);
        dwSensorSerializer_start(serializer);
    }

    // To serialize one needs to disable decoding
    dwSensorRadar_setDataDecoding(false, radarSensor);
    dwSensorRadar_setDataDecoding(DW_FALSE, radarSensor);

    dwRadarProperties    radarProperties;

    dwSensorRadar_getProperties(&radarProperties, radarSensor);

    const dwRadarScan *frame = nullptr;

    // Message msg;
    int packetCount = 0;
    auto start = std::chrono::high_resolution_clock::now();
    std::chrono::microseconds usec(1);

    gRun = dwSensor_start(radarSensor) == DW_SUCCESS;
    while (gRun) {
        const uint8_t *data = nullptr;
        size_t size         = 0;

        dwStatus status = DW_SUCCESS;
        status = dwSensor_readRawData(&data, &size, 500000, radarSensor);
        if (status == DW_SUCCESS) {

            // If we are reading too fast, we can throttle here. real sensors are slower than the
            // disk write speed. Virtual sensors can be a lot faster
            do
            {
                status = dwSensorSerializer_serializeDataAsync(data, size, serializer);
            }
            while(status == DW_NOT_AVAILABLE || status == DW_BUFFER_FULL);

            if (status != DW_SUCCESS) {
                std::cout << "Error serializing packet " << packetCount << std::endl;
            }

            // Here we can optionally decode the packet
            status = dwSensorRadar_processRawData(&frame, data, size, radarSensor);
            if (status !=DW_SUCCESS) {
                std::cout << "Error processing raw data"<<std::endl;
                return;
            }

            status = dwSensor_returnRawData(data, radarSensor);
            if (status != DW_SUCCESS) {
                std::cout << "Error returning packet " << packetCount << std::endl;
            }
            packetCount++;

            if(packetCount % 10 == 0) std::cout << "Processed " << packetCount << std::endl;
        }
        else if (status == DW_END_OF_STREAM) {
            std::cout << "radar end of stream reached" << std::endl;
            break;
        }
        else
        {
            if (status != DW_TIME_OUT) std::cout << "Error reading packet " << packetCount << std::endl;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end-start;
    std::cout << "Processed " << packetCount << " packets in " << elapsed.count() / 1000.0 << " s\n";

    dwSensorSerializer_stop(serializer);

    dwSensorSerializer_release(&serializer);

    dwSensor_stop(radarSensor);

    dwSAL_releaseSensor(&radarSensor);
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

    ProgramArguments arguments({ProgramArguments::Option_t("can-driver", "can.socket"),
                                ProgramArguments::Option_t("can-params", ""),
                                ProgramArguments::Option_t("gps-driver", "gps.uart"),
                                ProgramArguments::Option_t("gps-params", ""),
                                ProgramArguments::Option_t("lidar-driver", "lidar.virtual"),
                                ProgramArguments::Option_t("lidar-params", ""),
                                ProgramArguments::Option_t("radar-driver", "radar.virtual"),
                                ProgramArguments::Option_t("radar-params", ""),
                                ProgramArguments::Option_t("write-file-gps", ""),
                                ProgramArguments::Option_t("write-file-can", ""),
                                ProgramArguments::Option_t("write-file-lidar", ""),
                                ProgramArguments::Option_t("write-file-radar", ""),
                                ProgramArguments::Option_t("radar-object-filename","")});

    if (!arguments.parse(argc, argv) ||
        (!arguments.has("write-file-lidar") &&
         !arguments.has("write-file-radar") &&
         !arguments.has("write-file-gps") &&
         !arguments.has("write-file-can"))) {
        std::cout << "One or more of the following output file options is required:\n";
        std::cout << "\t --write-file-gps=/path/to/file.gps \t: file to record GPS data to\n";
        std::cout << "\t --write-file-can=/path/to/canbusfile \t: file to record CAN data to\n";
        std::cout << "\t --write-file-lidar=/path/to/lidarfile \t: file to record Lidar data to\n";
        std::cout << "\t --write-file-radar=/path/to/radarfile \t: file to record Radar data to\n";
        std::cout << "Additional options are:\n";
        std::cout << "\t --can-driver=can.socket \t\t: CAN driver to open (default=can.socket)\n";
        std::cout << "\t --can-params=device=can0,bus=d \t: parameters passed to CAN sensor\n";
        std::cout << "\t --gps-driver=gps.uart \t\t\t: GPS sensor driver (default=gps.uart)\n";
        std::cout << "\t --gps-params=device=/dev/ttyACM0 \t: parameters passed to GPS sensor\n";
        std::cout << "\t --lidar-driver=lidar.virtual \t\t\t: Lidar sensor driver (default=lidar.virtual)\n";
        std::cout << "\t --lidar-params=device=QUAN_M18A,file=filename\t: parameters passed to LIDAR sensor\n";
        std::cout << "\t --radar-driver=radar.virtual \t\t\t: Radar sensor driver (default=radar.virtual)\n";
        std::cout << "\t --radar-params=file=filename\t: parameters passed to RADAR sensor\n";
        return -1;
    }

    dwContextHandle_t sdk   = DW_NULL_HANDLE;
    dwSALHandle_t hal       = DW_NULL_HANDLE;

    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

    dwInitialize(&sdk, DW_VERSION, &sdkParams);

    // create HAL module of the SDK
    dwSAL_initialize(&hal, sdk);

    std::thread gpsThread;
    std::thread canThread;
    std::thread lidarThread;
    std::thread radarThread;

    if (!arguments.get("write-file-gps").empty())
        gpsThread   = std::thread(run_gps, arguments, hal);
    if (!arguments.get("write-file-can").empty())
        canThread   = std::thread(run_can, arguments, hal);
    if (!arguments.get("write-file-lidar").empty())
        lidarThread = std::thread(run_lidar, arguments, hal);
    if (!arguments.get("write-file-radar").empty())
        radarThread = std::thread(run_radar, arguments, hal);

    if (gpsThread.joinable())
        gpsThread.join();
    if (canThread.joinable())
        canThread.join();
    if (lidarThread.joinable())
        lidarThread.join();
    if (radarThread.joinable())
        radarThread.join();

    // release used objects in correct order
    dwSAL_release(&hal);
    dwRelease(&sdk);
    dwLogger_release();

    return 0;
}
