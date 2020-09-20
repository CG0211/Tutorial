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

#include <fstream>
#include <iostream>
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ctime>
#include <iostream>
#include <chrono>
#include <unistd.h>

#include <memory>

#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif
#include <framework/ProgramArguments.hpp>

#include <framework/MathUtils.hpp>
#include <framework/DataPath.hpp>
#include <framework/Log.hpp>
#include <framework/Checks.hpp>

// CORE
#include <dw/core/Logger.h>
#include <dw/core/Context.h>
#include <dw/renderer/Renderer.h>

// SAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/lidar/Lidar.h>

//#######################################################################################
// Globals
dwContextHandle_t      gSdk         = DW_NULL_HANDLE;
dwSALHandle_t          gSal         = DW_NULL_HANDLE;
dwSensorHandle_t       gLidarSensor = DW_NULL_HANDLE;
dwRendererHandle_t     gRenderer    = DW_NULL_HANDLE;
dwRenderBufferHandle_t gPointCloud  = DW_NULL_HANDLE;
dwRenderBufferHandle_t gGroundPlane = DW_NULL_HANDLE;

dwRect gLidarRect;

dwLidarProperties gLidarProperties;

float32_t *gPointCloudBuffer;

// Windows and general GL variables.
WindowBase *gWindow = nullptr;

std::string gMessage1;
std::string gMessage2;
std::string gMessage3;
std::string gMessage4;
std::string gMessage5;

int  gFrameWidth;
int  gFrameHeight;
bool gFullScreen    = false;
bool gRun           = false;
bool gPause         = false;
bool gRecordedLidar = false;
bool gShowIntensity = false;

//#######################################################################################
// Function declarations
bool initializeSensor(ProgramArguments &arguments);
void initializeRenderer();
void constructGrid();

// Auxiliary functions
void renderFrame();

//#######################################################################################
//
// USER INPUT
//
//#######################################################################################
extern void initializeInputDefaults();

extern void keyPressCallback   (int key);
extern void mouseUpCallback    (int button, float x, float y);
extern void mouseDownCallback  (int button, float x, float y);
extern void mouseMoveCallback  (float x, float y);
extern void mouseWheelCallback (float xOffset, float yOffset);

extern float eye[3];
extern float center[3];
extern float up[3];
extern float fovRads;

float modelview[16];
float projection[16];

//#######################################################################################
void resizeWindowCallback(int width, int height) {

    gFrameWidth  = width;
    gFrameHeight = height;

    gLidarRect.width  = gFrameWidth;
    gLidarRect.height = gFrameHeight;
    gLidarRect.x      = 0;
    gLidarRect.y      = 0;

    dwRenderer_setRect(gLidarRect, gRenderer);

    glViewport(0, 0, gFrameWidth, gFrameHeight);
}

//#######################################################################################
void renderFrame() {
    glDepthFunc(GL_LESS);

    // Only render color by distance in x y plane
    const float32_t vscale[3] = {130, 130, std::numeric_limits<float32_t>::infinity()};

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 3D rendering
    lookAt(modelview, eye, center, up);
    perspective(projection, fovRads, 1.0f * gLidarRect.width / gLidarRect.height, 0.1f, 1000.0f);

    dwRenderer_setModelView(modelview, gRenderer);
    dwRenderer_setProjection(projection, gRenderer);

    dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREY, gRenderer);
    dwRenderer_renderBuffer(gGroundPlane, gRenderer);

    dwRenderer_setColorMapScale(vscale, gRenderer);
    dwRenderer_renderBuffer(gPointCloud, gRenderer);

    // Overlay text
    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, gRenderer);
    dwRenderer_renderText(20, 80, gMessage1.c_str(), gRenderer);
    dwRenderer_renderText(20, 60, gMessage2.c_str(), gRenderer);
    dwRenderer_renderText(20, 40, gMessage3.c_str(), gRenderer);
    dwRenderer_renderText(20, 20, gMessage4.c_str(), gRenderer);
    dwRenderer_renderText(20, gFrameHeight - 20, gMessage5.c_str(), gRenderer);

    // Swap front and back buffers
    gWindow->swapBuffers();
}

//#######################################################################################
void computeSpin() {
    const  dwLidarDecodedPacket *nextPacket;
    static uint32_t packetCount = 0;
    static uint32_t accumulatedPoints = 0;
    static bool endOfSpin = false;
    static std::chrono::system_clock::time_point t_start = std::chrono::high_resolution_clock::now();
    static std::chrono::system_clock::time_point t_end;

    // Allow pausing for recoded replay
    if(gRecordedLidar && gPause)
        return;

    // For recorded data throttling check how long to a full spin and match to the lidar frequency
    if(gRecordedLidar && endOfSpin)
    {
        t_end            = std::chrono::high_resolution_clock::now();
        double duration  = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        double sleepTime = 1000.0 / gLidarProperties.spinFrequency - duration;

        if(sleepTime > 0.0) return;
        else                endOfSpin = false;
    }

    // Empty the queue and append all points withing the same spin.
    // Update render structures only when a full spin is done.
    dwStatus status = DW_SUCCESS;

    t_start = std::chrono::high_resolution_clock::now();
    while(status == DW_SUCCESS)
    {
        status = dwSensorLidar_readPacket(&nextPacket, 1000, gLidarSensor);
        if (status == DW_SUCCESS)
        {
            packetCount++;

            // Append the packet to the buffer
            float32_t *map = &gPointCloudBuffer[accumulatedPoints * gLidarProperties.pointStride];
            memcpy(map, nextPacket->points,
                        nextPacket->nPoints * gLidarProperties.pointStride * sizeof(float32_t));

            accumulatedPoints += nextPacket->nPoints;

            // If we go beyond a full spin, update the render data then return
            if(packetCount % gLidarProperties.packetsPerSpin == 0)
            {
                float32_t *map;
                uint32_t maxVerts, stride;

                dwRenderBuffer_map(&map, &maxVerts, &stride, gPointCloud);
                memcpy(map, gPointCloudBuffer, accumulatedPoints * stride * sizeof(float32_t));

                dwRenderBuffer_unmap(accumulatedPoints, gPointCloud);

                accumulatedPoints = 0;

                gMessage1 = "Host timestamp    (us) "      + std::to_string(nextPacket->hostTimestamp);
                gMessage2 = "Sensor timestamp (us) "       + std::to_string(nextPacket->sensorTimestamp);
                gMessage3 = "Packets                     " + std::to_string(packetCount);
                gMessage4 = "Frequency (Hz)           "    + std::to_string(gLidarProperties.spinFrequency);
                gMessage5 = "Press ESC to exit";

                // Grab properties, in case they were changed while running
                dwSensorLidar_getProperties(&gLidarProperties, gLidarSensor);

                endOfSpin = true;
                dwSensorLidar_returnPacket(nextPacket, gLidarSensor);
                return;
            }

            dwSensorLidar_returnPacket(nextPacket, gLidarSensor);
        }
        else
        {
            // For recorded data, start over at the end of the file
            if (status == DW_END_OF_STREAM)
            {
                // reset lidar file
                dwSensor_reset(gLidarSensor);
                accumulatedPoints = 0;
                packetCount       = 0;
            }
        }
    }
}


//#######################################################################################
void sig_int_handler(int sig) {
    (void)sig;

    gRun = false;
}

//#######################################################################################
//
// MAIN
//
//#######################################################################################
int main(int argc, char *argv[]) {
#ifndef WINDOWS
    struct sigaction action = {};
    action.sa_handler = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
#endif

    (void)argc;
    (void)argv;

    ProgramArguments arguments(
    {

        ProgramArguments::Option_t("protocol", "lidar.virtual"),
        ProgramArguments::Option_t("params", ("file=" + DataPath::get() +
                                   "/samples/sensors/lidar/lidar_velodyne_64.bin").c_str()),
        ProgramArguments::Option_t("show-intensity", "false")
    });

    if (!arguments.parse(argc, (const char **)argv))
        return -1; // Exit if not all require arguments are provided
    std::cout << "Program Arguments:\n" << arguments.printList() << std::endl;
    gShowIntensity = arguments.get("show-intensity").compare("true") == 0;

    gFullScreen  = false;
    gFrameWidth  = 1024;
    gFrameHeight = 800;

    // Initialize Application and subsystems
    // Initialize the GL and GLFW
    gWindow = gWindow ? gWindow : new WindowGLFW(gFrameWidth, gFrameHeight);

    gWindow->makeCurrent();

    gWindow->setOnKeypressCallback    (keyPressCallback);
    gWindow->setOnMouseDownCallback   (mouseDownCallback);
    gWindow->setOnMouseUpCallback     (mouseUpCallback);
    gWindow->setOnMouseMoveCallback   (mouseMoveCallback);
    gWindow->setOnMouseWheelCallback  (mouseWheelCallback);
    gWindow->setOnResizeWindowCallback(resizeWindowCallback);

    initializeInputDefaults();

    // Initialize DriveWorks
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate DriveWorks SDK context
    dwContextParameters sdkParams = {};

    dwInitialize(&gSdk, DW_VERSION, &sdkParams);

    // create HAL module of the SDK
    dwSAL_initialize(&gSal, gSdk);

    // Initialize the sensor
    if (!initializeSensor(arguments))
    {
        std::cerr << "Cannot connect to Lidar." << std::endl;
    }else{
        // Create and initialize a renderer
        CHECK_DW_ERROR( dwRenderer_initialize(&gRenderer, gSdk) );
        initializeRenderer();

        // Start the sensor
        gRun = (dwSensor_start(gLidarSensor) == DW_SUCCESS);

        // MAIN LOOP
        while (gRun && !gWindow->shouldClose())
        {
            // Render here
            renderFrame();

            // Poll for and process events
            glfwPollEvents();

            // Process stuff
            computeSpin();
        }

        dwSensor_stop(gLidarSensor);
        dwSAL_releaseSensor(&gLidarSensor);

        // release used objects in correct order
        dwRenderBuffer_release(&gPointCloud);
        dwRenderBuffer_release(&gGroundPlane);
        dwRenderer_release(&gRenderer);
    }
    dwSAL_release(&gSal);
    dwRelease(&gSdk);
    dwLogger_release();

    // Delete the point buffer
    delete[] gPointCloudBuffer;

    delete gWindow;

    return 0;
}

//#######################################################################################
// Initialize Renderer
//#######################################################################################
void initializeRenderer() {
    // Set some renderer defaults
    gLidarRect.width = gFrameWidth;
    gLidarRect.height = gFrameHeight;
    gLidarRect.x = 0;
    gLidarRect.y = 0;

    dwRenderer_setRect(gLidarRect, gRenderer);

    float32_t rasterTransform[9];
    rasterTransform[0] = 1.0f;
    rasterTransform[3] = 0.0f;
    rasterTransform[6] = 0.0f;

    rasterTransform[1] = 0.0f;
    rasterTransform[4] = 1.0f;
    rasterTransform[7] = 0.0f;

    rasterTransform[2] = 0.0f;
    rasterTransform[5] = 0.0f;
    rasterTransform[8] = 1.0f;

    dwRenderer_set2DTransform(rasterTransform, gRenderer);
    dwRenderer_setPointSize(2.0f, gRenderer);
    dwRenderer_setColor(DW_RENDERER_COLOR_RED, gRenderer);
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, gRenderer);

    // Ground plane grid
    constructGrid();

    // Point cloud
    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32B32A32_FLOAT;
    layout.colSemantic = gShowIntensity ? DW_RENDER_SEMANTIC_COL_HUE : DW_RENDER_SEMANTIC_COL_LUT;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

    // Initialize the structure
    dwRenderBuffer_initialize(&gPointCloud, layout, DW_RENDER_PRIM_POINTLIST, 
                               gLidarProperties.pointsPerSpin, gSdk);

    // Auxiliary buffer for double buffering GL updates
    gPointCloudBuffer = new float32_t[gLidarProperties.pointsPerSpin * gLidarProperties.pointStride];
}

//#######################################################################################
// Initialize Sensor
//#######################################################################################
bool initializeSensor(ProgramArguments &arguments) {
    // create Lidar interface
    gLidarSensor = DW_NULL_HANDLE;
    {
        dwSensorParams params;

        std::string parameterString;
        std::string protocolString;

        if (strcmp(arguments.get("protocol").c_str(), "") != 0) {
            protocolString = arguments.get("protocol");

            if (protocolString == "lidar.virtual")
                gRecordedLidar = true;
            else
                gRecordedLidar = false;
        }

        if (strcmp(arguments.get("params").c_str(), "") != 0)
        parameterString = arguments.get("params");

        if(protocolString.empty() || parameterString.empty())
        {
            std::cout << "INVALID PARAMETERS" << std::endl;
            exit(-1);
        }

        params.protocol = protocolString.c_str();
        params.parameters = parameterString.c_str();
        if( dwSAL_createSensor(&gLidarSensor, params, gSal) == DW_SUCCESS)
        {
            // Get lidar properties
            dwSensorLidar_getProperties(&gLidarProperties, gLidarSensor);
            return true;
        }
        else
            return false;
    }
}

//#######################################################################################
// Initialize World Grid
//#######################################################################################
void constructGrid() {
  const float WORLD_GRID_SIZE_IN_METERS = 200.0f;
  const float WORLD_GRID_RES_IN_METERS = 5.0f;

  // World grid
  int gridResolution =
      static_cast<int>(WORLD_GRID_SIZE_IN_METERS / WORLD_GRID_RES_IN_METERS);

  // Rendering data
  dwRenderBufferVertexLayout layout;
  layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
  layout.posFormat = DW_RENDER_FORMAT_R32G32B32_FLOAT;
  layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
  layout.colFormat = DW_RENDER_FORMAT_NULL;
  layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
  layout.texFormat = DW_RENDER_FORMAT_NULL;

  dwRenderBuffer_initialize(&gGroundPlane, layout, DW_RENDER_PRIM_LINELIST,
                            2 * (gridResolution + 1), gSdk);

  // update the data
  float32_t *map;
  uint32_t maxVerts, stride;

  dwRenderBuffer_map(&map, &maxVerts, &stride, gGroundPlane);

  int nVertices = 0;
  float x, y;

  // Horizontal lines
  x = -0.5f * WORLD_GRID_SIZE_IN_METERS;
  for (int i = 0; i <= gridResolution; ++i) {
    y = -0.5f * WORLD_GRID_SIZE_IN_METERS;

    map[stride * nVertices + 0] = x;
    map[stride * nVertices + 1] = y;
    map[stride * nVertices + 2] = -0.05f;
    nVertices++;

    y = 0.5f * WORLD_GRID_SIZE_IN_METERS;
    map[stride * nVertices + 0] = x;
    map[stride * nVertices + 1] = y;
    map[stride * nVertices + 2] = -0.05f;

    nVertices++;
    x = x + WORLD_GRID_RES_IN_METERS;
  }

  // Vertical lines
  y = -0.5f * WORLD_GRID_SIZE_IN_METERS;
  for (int i = 0; i <= gridResolution; ++i) {
    x = -0.5f * WORLD_GRID_SIZE_IN_METERS;

    map[stride * nVertices + 0] = x;
    map[stride * nVertices + 1] = y;
    map[stride * nVertices + 2] = -0.05f;
    nVertices++;

    x = 0.5f * WORLD_GRID_SIZE_IN_METERS;
    map[stride * nVertices + 0] = x;
    map[stride * nVertices + 1] = y;
    map[stride * nVertices + 2] = -0.05f;

    nVertices++;
    y = y + WORLD_GRID_RES_IN_METERS;
  }

  dwRenderBuffer_unmap(maxVerts, gGroundPlane);
}
