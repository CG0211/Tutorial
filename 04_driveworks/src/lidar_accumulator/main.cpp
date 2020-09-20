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
// Copyright (c) 2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <signal.h>
#include <string.h>

#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include <algorithm>

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
#include <dw/image/ImageStreamer.h>

// SAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/lidar/Lidar.h>
#include <dw/lidaraccumulator/LidarAccumulator.h>

//#######################################################################################
// Globals
//#######################################################################################

dwContextHandle_t          gSdk                  = DW_NULL_HANDLE;
dwSALHandle_t              gSal                  = DW_NULL_HANDLE;
dwSensorHandle_t           gLidarSensor          = DW_NULL_HANDLE;
dwImageStreamerHandle_t    gCpu2Gl               = DW_NULL_HANDLE;

dwRendererHandle_t         gRenderer             = DW_NULL_HANDLE;
dwRenderBufferHandle_t     gGroundPlane          = DW_NULL_HANDLE;
dwRenderBufferHandle_t     gSectPointCloud       = DW_NULL_HANDLE;
dwRenderBufferHandle_t     gFullPointCloud       = DW_NULL_HANDLE;
dwLidarAccumulatorHandle_t gFullLidarAccumulator = DW_NULL_HANDLE;
dwLidarAccumulatorHandle_t gSectLidarAccumulator = DW_NULL_HANDLE;

dwLidarAccumulatorBuffer   gFullLidarBuffer{};
dwLidarAccumulatorBuffer   gSectLidarBuffer{};
dwLidarProperties          gLidarProperties{};

float32_t gMinAngle;
float32_t gMaxAngle;
float32_t gMinDistance;
float32_t gMaxDistance;
dwRect gPointCloudRect{};
dwRect gFullLidarImageRect{};
dwRect gSectLidarImageRect{};
dwImageCPU gFullLidarImageCPU{};
dwImageCPU gSectLidarImageCPU{};
std::string gLidarImagetypeString{};

// map the Lidar image type to dwLidarImageType
std::map<std::string, dwLidarImageType> gLidarImageTypeMaps{
                                                           {"depth-xyz", DW_LIDAR_IMAGE_TYPE_3D_DISTANCE_IMAGE},
                                                           {"depth-xy",  DW_LIDAR_IMAGE_TYPE_2D_DISTANCE_IMAGE},
                                                           {"intensity", DW_LIDAR_IMAGE_TYPE_INTENSITY_IMAGE  },
                                                           };

std::string gSupportedLidarDevices[] {"VELO_HDL64E", "VELO_HDL32E", "VELO_VLP16", "QUAN_M81A"};

typedef std::chrono::high_resolution_clock myclock_t;
typedef std::chrono::time_point<myclock_t> timepoint_t;
typedef std::chrono::milliseconds millisec_t;

// Windows and general GL variables.
WindowBase *gWindow = nullptr;

std::string gMessage1;
std::string gMessage2;
std::string gMessage3;
std::string gMessage4;
std::string gMessage5;
std::string gMessage6;
std::string gMessage7;

int  gFrameWidth;
int  gFrameHeight;
bool gRun           = false;
bool gPause         = false;
bool gRecordedLidar = false;

//#######################################################################################
// Function declarations
//#######################################################################################
bool initializeSensor(ProgramArguments &arguments);
void initializeLidarAccumulators(ProgramArguments &arguments);
void initializeRenderer();
void constructGrid();
void pipelineSync(millisec_t frameDuration);
void pipelineAsync(millisec_t frameDuration);
void bindRenderBuffer(dwRenderBufferHandle_t renderBuffer, dwLidarAccumulatorBuffer *lidarBuffer);
void renderLidarImage(const dwRect &rect, dwImageCPU &imageCPU);
void renderText(dwLidarAccumulatorBuffer *lidarBuffer, uint32_t spinCount);
void renderPointCloud();
void renderScreen();

//#######################################################################################
// USER INPUT
//#######################################################################################
extern void initializeInputDefaults();

extern void keyPressCallback  (int key);
extern void mouseUpCallback   (int button, float x, float y);
extern void mouseDownCallback (int button, float x, float y);
extern void mouseMoveCallback (float x, float y);
extern void mouseWheelCallback(float xOffset, float yOffset);

extern float eye[3];
extern float center[3];
extern float up[3];
extern float fovRads;
extern float deltaSpinAngle;
float modelview[16];
float projection[16];

//#######################################################################################
//  Render Depth Image
//#######################################################################################
void renderLidarImage(const dwRect &rect, dwImageCPU &imageCPU)
{
    if (!gCpu2Gl) {
        return;
    }

    dwImageGL *imageGL = nullptr;

    CHECK_DW_ERROR(dwImageStreamer_postCPU(&imageCPU, gCpu2Gl));
    CHECK_DW_ERROR(dwImageStreamer_receiveGL(&imageGL, 30000, gCpu2Gl));

    if (imageGL) {
        dwImageCPU *nextImage;
        dwRenderer_setRect(rect, gRenderer);
        dwRenderer_renderTexture(imageGL->tex, imageGL->target, gRenderer);
        CHECK_DW_ERROR(dwImageStreamer_returnReceivedGL(imageGL, gCpu2Gl));
        CHECK_DW_ERROR(dwImageStreamer_waitPostedCPU(&nextImage, 30000, gCpu2Gl));
    }
}

//#######################################################################################
// Render Lidar Point Cloud
//#######################################################################################
void renderPointCloud()
{
    glDepthFunc(GL_LESS);

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    dwRenderer_setRect(gPointCloudRect, gRenderer);

    // 3D rendering
    lookAt(modelview, eye, center, up);
    perspective(projection, fovRads, 1.0f * gPointCloudRect.width / gPointCloudRect.height, 0.1f, 1000.0f);

    dwRenderer_setModelView(modelview, gRenderer);
    dwRenderer_setProjection(projection, gRenderer);

    dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREY, gRenderer);
    dwRenderer_renderBuffer(gGroundPlane, gRenderer);

    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, gRenderer);
    dwRenderer_renderBuffer(gSectPointCloud, gRenderer);

    dwRenderer_setColor(DW_RENDERER_COLOR_BLUE, gRenderer);
    dwRenderer_renderBuffer(gFullPointCloud, gRenderer);

    // Overlay text
    dwRenderer_setColor(DW_RENDERER_COLOR_ORANGE, gRenderer);
    dwRenderer_renderText(20, 80, gMessage1.c_str(), gRenderer);
    dwRenderer_renderText(20, 60, gMessage2.c_str(), gRenderer);
    dwRenderer_renderText(20, 40, gMessage3.c_str(), gRenderer);
    dwRenderer_renderText(20, 20, gMessage4.c_str(), gRenderer);
    dwRenderer_renderText(20, gFrameHeight - 220, gMessage5.c_str(), gRenderer);
    dwRenderer_renderText(20, gFrameHeight - 240, gMessage6.c_str(), gRenderer);
    dwRenderer_renderText(20, gFrameHeight - 260, gMessage7.c_str(), gRenderer);

}

//#######################################################################################
// Render Screen
//#######################################################################################
void renderScreen()
{
    // render point cloud
    renderPointCloud();

    // render the sector depth image
    renderLidarImage(gSectLidarImageRect, gSectLidarImageCPU);

    // render the full spin depth image
    renderLidarImage(gFullLidarImageRect, gFullLidarImageCPU);

    // swap front and back buffers
    gWindow->swapBuffers();

    // poll for and process events
    glfwPollEvents();
}

//#######################################################################################
// Signal Handling
//#######################################################################################
void sig_int_handler(int sig)
{
    (void)sig;
    gRun = false;
}

//#######################################################################################
// Init Command Line Arguments
//#######################################################################################
void initCommandLineArguments(ProgramArguments &arguments)
{
    std::string dataRootPath = DataPath::get();
    std::string lidarBinFile = dataRootPath + "/samples/sensors/lidar/lidar_velodyne_64.bin";
    arguments.addOption({"file", lidarBinFile.c_str(), "set it to recorded Lidar file if not using live Lidar sensor"});
    arguments.addOption({"mode", "sync", "mode=sync, it gets accumulated spin and visualizes the spin will be in the same thread, "
                         "mode=async, it uses dedicated thread to accumulate the spin, the visualization runs in another thread"});
    arguments.addOption({"ip", "", "live Lidar sensor ip address, no need if using Lidar file"});
    arguments.addOption({"port", "", "live Lidar sensor port number, no need if using Lidar file"});
    arguments.addOption({"scan-frequency", "", "live Lidar sensor scan frequency, i.e. 5, 10, no need if using Lidar file"});
    arguments.addOption({"device", "", "live Lidar sensor type, i.e. device=VELO_HDL64E, no need if using Lidar file"});
    arguments.addOption({"minAngle", "30", "i.e. minAngle=50, Lidar accumulator will collect points whose azimuth are beyond 50 degrees"});
    arguments.addOption({"maxAngle", "150", "i.e. maxAngle=100, Lidar accumulator will collect points whose azimuth are less or equal to 100 degrees"});
    arguments.addOption({"maxDistance", "0", "max distance (in meters) the accumulator covers, max=0 will cover all distances"});
    arguments.addOption({"minDistance", "0", "min distance (in meters) the accumulator covers, "
                         "i.e. min=10 will collect points at least 10 meters aways from Lidar"});
    arguments.addOption({"smooth-window-size", "4", "smoothing the Lidar horizontal jittering, smooth-window-size=1 or 2 or 4 or 8"});
    arguments.addOption({"lidar-image-type", "depth-xyz", "there are three options, i.e. lidar-image-type=depth-xyz "
                         "or lidar-image-type=depth-xy or lidar-image-type=intensity"});
}

//#######################################################################################
// MAIN
//#######################################################################################
int main(int argc, char *argv[])
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

    (void)argc;
    (void)argv;

    ProgramArguments arguments;
    initCommandLineArguments(arguments);
    if (!arguments.parse(argc, (const char **)argv))
        return -1; // Exit if not all require arguments are provided
    std::cout << "Program Arguments:\n" << arguments.printList() << std::endl;

    gFrameWidth  = 1024;
    gFrameHeight = 800;

    // Initialize the GL and GLFW
    gWindow = gWindow ? gWindow : new WindowGLFW(gFrameWidth, gFrameHeight);

    gWindow->makeCurrent();

    gWindow->setOnKeypressCallback  (keyPressCallback);
    gWindow->setOnMouseDownCallback (mouseDownCallback);
    gWindow->setOnMouseUpCallback   (mouseUpCallback);
    gWindow->setOnMouseMoveCallback (mouseMoveCallback);
    gWindow->setOnMouseWheelCallback(mouseWheelCallback);

    initializeInputDefaults();

    // Initialize DriveWorks Logger
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate DriveWorks SDK context
    dwContextParameters sdkParams{};

    dwInitialize(&gSdk, DW_VERSION, &sdkParams);

    // create HAL module of the SDK
    dwSAL_initialize(&gSal, gSdk);

    // Initialize the sensor
    if (!initializeSensor(arguments)) {
        std::cerr << "Cannot connect to Lidar." << std::endl;
    } else {
        // Create and initialize a renderer
        dwRenderer_initialize(&gRenderer, gSdk);
        initializeRenderer();

        // Start the sensor
        gRun = (dwSensor_start(gLidarSensor) == DW_SUCCESS);

        // Init Lidar Accumulator
        initializeLidarAccumulators(arguments);

        int32_t spinFrequency = static_cast<int32_t>(gLidarProperties.spinFrequency);
        auto frameDuration    = static_cast<millisec_t>(1000/spinFrequency);

        // Main Loop
        if (strcmp(arguments.get("mode").c_str(), "sync") == 0) {
            pipelineSync(frameDuration);
        } else {
            pipelineAsync(frameDuration);
        }

        // Clean up
        dwLidarAccumulator_destroyImage(&gFullLidarImageCPU, gFullLidarAccumulator);
        dwLidarAccumulator_destroyImage(&gSectLidarImageCPU, gSectLidarAccumulator);
        dwLidarAccumulator_release(&gFullLidarAccumulator);
        dwLidarAccumulator_release(&gSectLidarAccumulator);
        dwSensor_stop(gLidarSensor);
        dwSAL_releaseSensor(&gLidarSensor);
        dwImageStreamer_release(&gCpu2Gl);

        // release used objects in correct order
        dwRenderBuffer_release(&gFullPointCloud);
        dwRenderBuffer_release(&gSectPointCloud);
        dwRenderBuffer_release(&gGroundPlane);
        dwRenderer_release(&gRenderer);
    }
    dwSAL_release(&gSal);
    dwRelease(&gSdk);
    dwLogger_release();

    delete gWindow;

    return 0;
}

//#######################################################################################
// Throttle the speed for recorded data
//#######################################################################################
timepoint_t throttleFrameSpeed(millisec_t frameDuration, timepoint_t lastUpdateTime)
{
    auto timeSinceUpdate = std::chrono::duration_cast<millisec_t>(myclock_t::now() - lastUpdateTime);
    if (timeSinceUpdate < frameDuration) {
        std::this_thread::sleep_for(frameDuration - timeSinceUpdate);
    }
    lastUpdateTime = myclock_t::now();
    return lastUpdateTime;
}

//#######################################################################################
// Generate render text
//#######################################################################################
void renderText(dwLidarAccumulatorBuffer *lidarBuffer, uint32_t spinCount)
{
    gMessage1 = "Point/Spin count   " + std::to_string(lidarBuffer->dataCount)
                                      + "/" + std::to_string(spinCount);
    gMessage2 = "Valid point count  " + std::to_string(lidarBuffer->validCount);
    gMessage3 = "Spin timestamp     " + std::to_string(gFullLidarImageCPU.timestamp_us);
    gMessage4 = "Frequency (Hz)     " + std::to_string(gLidarProperties.spinFrequency);
    gMessage5 = "Press ESC to exit ";
    gMessage6 = "Press 0 or 1 to show Lidar intensity image or depth image";
    gMessage7 = "Press Left key or Right key to rotate counter-clock-wise or clock-wise";
}

//#######################################################################################
// Bind Lidar Accumulator buffer to render buffer
//#######################################################################################
void bindRenderBuffer(dwRenderBufferHandle_t renderBuffer, dwLidarAccumulatorBuffer *lidarBuffer)
{
    float32_t *map;
    uint32_t maxVerts, stride;

    dwRenderBuffer_map(&map, &maxVerts, &stride, renderBuffer);
    memcpy(map, lidarBuffer->data, gLidarProperties.pointsPerSpin * sizeof(dwVector4f));
    dwRenderBuffer_unmap(gLidarProperties.pointsPerSpin, renderBuffer);
}

//#######################################################################################
// Run the synchronous pipeline
// Read Lidar packet and do spin extraction in the same thread
//#######################################################################################
void pipelineSync(millisec_t frameDuration)
{
    auto lastUpdateTime = myclock_t::now();
    static uint32_t spinCount = 0;
    // MAIN LOOP
    while (gRun && !gWindow->shouldClose()) {
        if (!(gRecordedLidar && gPause)) {
            const dwLidarDecodedPacket *nextPacket;
            dwStatus status = dwSensorLidar_readPacket(&nextPacket, 1000, gLidarSensor);
            if (status == DW_SUCCESS) {
                CHECK_DW_ERROR(dwLidarAccumulator_addPacket(nextPacket, gFullLidarAccumulator));
                CHECK_DW_ERROR(dwLidarAccumulator_addPacket(nextPacket, gSectLidarAccumulator));
                CHECK_DW_ERROR(dwSensorLidar_returnPacket(nextPacket, gLidarSensor));
            }
            else if (status == DW_END_OF_STREAM) {
                // send an empty packet to Lidar Accumulator
                CHECK_DW_ERROR(dwSensor_reset(gLidarSensor));
                CHECK_DW_ERROR(dwLidarAccumulator_reset(gFullLidarAccumulator));
                CHECK_DW_ERROR(dwLidarAccumulator_reset(gSectLidarAccumulator));
                spinCount = 0;
                continue;
            }

            // get sector Lidar buffer
            status = dwLidarAccumulator_getSweep(&gSectLidarBuffer, gSectLidarAccumulator);
            CHECK_DW_ERROR(dwLidarAccumulator_fillImage(&gSectLidarImageCPU,
                                                        gLidarImageTypeMaps[gLidarImagetypeString],
                                                        gSectLidarAccumulator));
            if (status == DW_NOT_READY) {
                continue;
            } else if (status != DW_SUCCESS) {
                std::cerr << "Fail to get sector spin from Lidar Accumulator!" << std::endl;
                continue;
            }
            bindRenderBuffer(gSectPointCloud, &gSectLidarBuffer);
            CHECK_DW_ERROR(dwLidarAccumulator_returnSweep(gSectLidarAccumulator));
            CHECK_DW_ERROR(dwLidarAccumulator_setAngleSpan(gMinAngle + deltaSpinAngle,
                                                           gMaxAngle + deltaSpinAngle, gSectLidarAccumulator));

            // get full spin Lidar buffer
            status = dwLidarAccumulator_getSweep(&gFullLidarBuffer, gFullLidarAccumulator);
            if (status == DW_NOT_READY) {
                continue;
            } else if (status != DW_SUCCESS) {
                std::cerr << "Fail to get full sweep from Lidar Accumulator!" << std::endl;
                continue;
            }
            // get full spin Lidar cylindrical image
            CHECK_DW_ERROR(dwLidarAccumulator_fillImage(&gFullLidarImageCPU,
                                                        gLidarImageTypeMaps[gLidarImagetypeString],
                                                        gFullLidarAccumulator));
            bindRenderBuffer(gFullPointCloud, &gFullLidarBuffer);
            renderText(&gFullLidarBuffer, spinCount);
            CHECK_DW_ERROR(dwLidarAccumulator_returnSweep(gFullLidarAccumulator));
            spinCount++;
        }

        // sync with Lidar frequency
        lastUpdateTime = throttleFrameSpeed(frameDuration, lastUpdateTime);

        // render the data
        renderScreen();
    }
}

//#######################################################################################
void loadPacketThread()
{
    while(gRun) {
        const dwLidarDecodedPacket *nextPacket;
        dwStatus status = dwSensorLidar_readPacket(&nextPacket, 1000, gLidarSensor);
        if (status == DW_SUCCESS) {
            while (true) {
                status = dwLidarAccumulator_addPacket(nextPacket, gFullLidarAccumulator);
                if (status == DW_SUCCESS) break;
            }
            CHECK_DW_ERROR(dwSensorLidar_returnPacket(nextPacket, gLidarSensor));
        }
        else if (status == DW_END_OF_STREAM) {
            // send an empty packet to Lidar Accumulator
            while (true) {
                status = dwLidarAccumulator_addPacket(nullptr, gFullLidarAccumulator);
                if (status == DW_SUCCESS) break;
            }

            // reset Lidar file
            CHECK_DW_ERROR(dwSensor_reset(gLidarSensor));
            break;
        }
    }
}

//#######################################################################################
// Run the asynchronous pipeline
// Read Lidar packet and do spin extraction in different threads
//#######################################################################################
void pipelineAsync(millisec_t frameDuration)
{
    std::thread loadThread     = std::thread(loadPacketThread);
    timepoint_t lastUpdateTime = myclock_t::now();

    // MAIN LOOP
    uint32_t spinCount = 0;
    while (gRun && !gWindow->shouldClose()) {
        std::this_thread::yield();

        // sync with Lidar frequency
        lastUpdateTime = throttleFrameSpeed(frameDuration, lastUpdateTime);

        if (!(gRecordedLidar && gPause)) {
            dwLidarAccumulatorBuffer sweepBuffer = {};
            dwStatus status = dwLidarAccumulator_getSweep(&sweepBuffer, gFullLidarAccumulator);
            CHECK_DW_ERROR(dwLidarAccumulator_fillImage(&gFullLidarImageCPU,
                                                        gLidarImageTypeMaps[gLidarImagetypeString],
                                                        gFullLidarAccumulator));
            if (status == DW_END_OF_STREAM) {
                std::cout << "Reach the end of recording" << std::endl;
                CHECK_DW_ERROR(dwSensor_reset(gLidarSensor));
                CHECK_DW_ERROR(dwLidarAccumulator_reset(gFullLidarAccumulator));
                loadThread.join();
                loadThread = std::thread(loadPacketThread);
                spinCount = 0;
                continue;
            } else if (status == DW_NOT_READY) {
                std::cout << "Lidar full sweep is not ready, sleep 10 ms" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            } else if (status != DW_SUCCESS) {
                std::cerr << "Fail to load Lidar full sweep" << std::endl;
                continue;
            }
            bindRenderBuffer(gFullPointCloud, &sweepBuffer);
            renderText(&sweepBuffer, spinCount);
            CHECK_DW_ERROR(dwLidarAccumulator_returnSweep(gFullLidarAccumulator));
            spinCount++;
        }
        renderScreen();
    }

    if (loadThread.joinable()) {
        loadThread.join();
    }
}

//#######################################################################################
// Initialize Renderer
//#######################################################################################
void initializeRenderer()
{
    // Depth Image Window
    gFullLidarImageRect.width  = gFrameWidth;
    gFullLidarImageRect.height = 100;
    gFullLidarImageRect.x = 0;
    gFullLidarImageRect.y = 0;

    gSectLidarImageRect.width  = gFrameWidth;
    gSectLidarImageRect.height = 100;
    gSectLidarImageRect.x = 0;
    gSectLidarImageRect.y = gFullLidarImageRect.height;

    // Point cloud Window
    gPointCloudRect.width  = gFrameWidth;
    gPointCloudRect.height = gFrameHeight - 200;
    gPointCloudRect.x = 0;
    gPointCloudRect.y = 200;

    dwRenderer_setRect(gPointCloudRect, gRenderer);

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

    // RenderBuffer
    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32B32A32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

    // Initialize Full Spin Point cloud
    dwRenderBuffer_initialize(&gFullPointCloud, layout, DW_RENDER_PRIM_POINTLIST,
                               gLidarProperties.pointsPerSpin, gSdk);

    // Initialize Sector Spin Point cloud
    dwRenderBuffer_initialize(&gSectPointCloud, layout, DW_RENDER_PRIM_POINTLIST,
                               gLidarProperties.pointsPerSpin, gSdk);
}

//#######################################################################################
// Initialize Sensor
//#######################################################################################
bool initializeSensor(ProgramArguments &arguments)
{
    dwSensorParams params{};

    std::string parameterString;
    std::string protocolString;

    bool validParameters = false;

    if (strcmp(arguments.get("ip").c_str(), "") != 0) {

        if (strcmp(arguments.get("port").c_str(), "") != 0) {

            if (strcmp(arguments.get("device").c_str(), "") != 0) {

                if (strcmp(arguments.get("scan-frequency").c_str(), "") != 0) {
                    protocolString  = "lidar.socket";
                    parameterString = "ip=" + arguments.get("ip");
                    parameterString += ",port=" + arguments.get("port");
                    parameterString += ",device=" + arguments.get("device");
                    parameterString += ",scan-frequency=" + arguments.get("scan-frequency");

                    gRecordedLidar  = false;
                    validParameters = true;
                }
            }
        }
    } else {
        if (strcmp(arguments.get("file").c_str(), "") != 0) {
            protocolString  = "lidar.virtual";
            parameterString = "file=" + arguments.get("file");

            // When reading from file, throttle the frame rate
            gRecordedLidar  = true;
            validParameters = true;
        }
    }

    if(!validParameters) {
        std::cout << "INVALID PARAMETERS" << std::endl;
        exit(-1);
    }

    params.protocol   = protocolString.c_str();
    params.parameters = parameterString.c_str();
    if( dwSAL_createSensor(&gLidarSensor, params, gSal) == DW_SUCCESS) {
        dwSensorLidar_getProperties(&gLidarProperties, gLidarSensor);
        return true;
    } else {
        return false;
    }
}

//#######################################################################################
// Initialize Lidar Accumulators
//#######################################################################################
void initializeLidarAccumulators(ProgramArguments &arguments)
{
    // Radial Distance
    gMinDistance = std::stof(arguments.get("minDistance"));
    gMaxDistance = std::stof(arguments.get("maxDistance"));
    gMinAngle    = std::stof(arguments.get("minAngle"));
    gMaxAngle    = std::stof(arguments.get("maxAngle"));

    gLidarImagetypeString = arguments.get("lidar-image-type");

    auto search = gLidarImageTypeMaps.find(gLidarImagetypeString);
    if (search == gLidarImageTypeMaps.end()) {
        gLidarImagetypeString = "depth-xyz";
        std::cerr << "invalid argument for --lidar-image-type, set it to " << gLidarImagetypeString << std::endl;
    }
    // Window size for Lidar jittering smoothing
    uint32_t smoothWinSize = std::stol(arguments.get("smooth-window-size"));

    size_t sz = sizeof(gSupportedLidarDevices) / sizeof(std::string);
    std::vector<std::string> targets(gSupportedLidarDevices, gSupportedLidarDevices + sz);
    if (std::find(targets.begin(), targets.end(), std::string{gLidarProperties.deviceString}) == targets.end()) {
        std::cerr << "unsupported lidar device type " << gLidarProperties.deviceString << std::endl;
        exit(-1);
    }

    // Init Full Spin Lidar Accumulator
    CHECK_DW_ERROR(dwLidarAccumulator_initialize(&gFullLidarAccumulator, gSdk, smoothWinSize, &gLidarProperties));
    CHECK_DW_ERROR(dwLidarAccumulator_setDistanceSpan(gMinDistance, gMaxDistance, gFullLidarAccumulator));
    CHECK_DW_ERROR(dwLidarAccumulator_createImage(&gFullLidarImageCPU, gFullLidarAccumulator));

    // Init Sector Lidar Accumulator (every sector covers sectorAngle degree)
    CHECK_DW_ERROR(dwLidarAccumulator_initialize(&gSectLidarAccumulator, gSdk, smoothWinSize, &gLidarProperties));
    CHECK_DW_ERROR(dwLidarAccumulator_setAngleSpan(gMinAngle, gMaxAngle, gSectLidarAccumulator));
    CHECK_DW_ERROR(dwLidarAccumulator_createImage(&gSectLidarImageCPU, gSectLidarAccumulator));

    // Init CPU to GL streamer
    if ((gFullLidarImageCPU.prop.width > 0) && (gFullLidarImageCPU.prop.height > 0)) {
        CHECK_DW_ERROR(dwImageStreamer_initialize(&gCpu2Gl, &gFullLidarImageCPU.prop, DW_IMAGE_GL, gSdk));
    }
}

//#######################################################################################
// Initialize World Grid
//#######################################################################################
void constructGrid()
{
    const float WORLD_GRID_SIZE_IN_METERS = 200.0f;
    const float WORLD_GRID_RES_IN_METERS = 5.0f;

    // World grid
    int gridResolution =
            static_cast<int>(WORLD_GRID_SIZE_IN_METERS / WORLD_GRID_RES_IN_METERS);

    // Rendering data
    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

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
