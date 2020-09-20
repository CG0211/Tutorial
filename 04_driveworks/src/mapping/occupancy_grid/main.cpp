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
// Copyright (c) 2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#define _USE_MATH_DEFINES
#include <cmath>
#define M_PI_F static_cast<float>(M_PI)

// SAMPLE COMMON
#include <framework/Checks.hpp>
#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif

#include <framework/Log.hpp>
#include <framework/DataPath.hpp>
#include <framework/Mat4.hpp>
#include <framework/ProgramArguments.hpp>

#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/egomotion/Egomotion.h>
#include <dw/gl/GL.h>
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>
#include <dw/mapping/OccupancyGrid.h>
#include <dw/renderer/Renderer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>
#include <dw/sensors/lidar/Lidar.h>

#include <sstream>
#include <iomanip>

#ifndef WINDOWS
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
#endif

#include <fstream>
#include <iostream>

#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool g_run          = true;
static const char *CAN_CAR_SPEED    = "Steering_Report.SPEED";
static const char *CAN_CAR_STEERING = "Steering_Report.ANGLE";

static bool RENDER_ROTATION_ARROW = true;

dwOccupancyGridHandle_t g_occupancyGrid = DW_NULL_HANDLE;
dwOccupancyGridParameters g_occupancyGridInitParams;

// For drawing the fbo that is rendered to by
// the occupancy grid.
dwImageGL g_gridImage{};

dwRenderBufferHandle_t g_lineBuffer;

// Bookkeeping vars for controlling render
// framerate.
int64_t g_frameRate = 30; // Set by program arguments
bool g_pause        = false;

dwSensorHandle_t g_lidarSensor            = DW_NULL_HANDLE;
dwSensorHandle_t g_canSensor              = DW_NULL_HANDLE;
dwSensorHandle_t g_cameraSensor           = DW_NULL_HANDLE;
dwCANInterpreterHandle_t g_canInterpreter = DW_NULL_HANDLE;
dwSALHandle_t g_sal                       = DW_NULL_HANDLE;
dwEgomotionHandle_t g_egomotion           = DW_NULL_HANDLE;
dwImageFormatConverterHandle_t g_yuv2RGBA = DW_NULL_HANDLE;
dwRendererHandle_t g_renderer             = DW_NULL_HANDLE;
dwImageStreamerHandle_t g_image2GL        = DW_NULL_HANDLE;
#ifdef VIBRANTE
dwImageNvMedia *g_nvmediaRGBA = nullptr;
#else
dwImageCUDA *g_cudaRGBA = nullptr;
#endif
dwTransformation g_rig2World{};

const float32_t *g_arrowColor     = &DW_RENDERER_COLOR_RED[0];
float64_t g_currentFPS            = g_frameRate;
uint64_t g_currentLidarPointCount = 0;
//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void initGL(WindowBase **window);
void initSdk(dwContextHandle_t *context, WindowBase *window);
void initialize(dwContextHandle_t sdk, const std::string lidarFile, const std::string canFile,
                const std::string dbcFile, const std::string videoFile,
                const std::string videoTimestampFile);
void loop(WindowBase *window);
void release();

void sig_int_handler(int sig);
void keyPressCallback(int key);

//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;

    g_run = false;
}

//------------------------------------------------------------------------------
void keyPressCallback(int key)
{
    // stop application
    if (key == GLFW_KEY_ESCAPE) {
        g_run = false;
    } else if (key == GLFW_KEY_SPACE || key == GLFW_KEY_P) {
        g_pause = !g_pause;
    }
}

//------------------------------------------------------------------------------
void initGL(WindowBase **window)
{
#ifndef VIBRANTE
    glewExperimental = GL_TRUE;
    glewInit();
#endif

    if (!*window)
        *window = new WindowGLFW(800, 800);

    (*window)->makeCurrent();
    (*window)->setOnKeypressCallback(keyPressCallback);
}

//------------------------------------------------------------------------------
void initSdk(dwContextHandle_t *context, WindowBase *window)
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = window->getEGLDisplay();
#else
    (void)window;
#endif

    dwInitialize(context, DW_VERSION, &sdkParams);
}

bool nextCANMeasurement(dwTime_t &timestamp, float32_t &speed, float32_t &steeringAngle,
                        bool &hasSpeed, bool &hasSteering)
{
    dwCANMessage frame{};
    dwStatus status = dwSensorCAN_readMessage(&frame, 100000, g_canSensor);

    if (status == DW_END_OF_STREAM) {
        printf("EndOfStream\n");
        dwSensor_reset(g_canSensor);
        return false;
    }

    // pass message to interpreter
    if (status == DW_SUCCESS && g_canSensor) {
        dwCANInterpreter_consume(&frame, g_canInterpreter);
    }

    hasSpeed    = false;
    hasSteering = false;

    // check between all available signals for consumption
    uint32_t numSignals = 0;
    if (dwCANInterpreter_getNumberSignals(&numSignals, g_canInterpreter) == DW_SUCCESS) {
        for (uint32_t i = 0; i < numSignals; i++) {
            const char *name = nullptr;
            float32_t value  = 0;
            dwTime_t ts;

            dwCANInterpreter_getSignalName(&name, i, g_canInterpreter);
            dwCANInterpreter_getf32(&value, &ts, i, g_canInterpreter);

            if (0 == strcmp(name, CAN_CAR_SPEED)) {
                speed     = value;
                timestamp = ts;
                hasSpeed  = true;
            } else if (0 == strcmp(name, CAN_CAR_STEERING)) {
                steeringAngle = value;
                timestamp     = ts;
                hasSteering   = true;
            }
        }
    }

    return true;
}

//------------------------------------------------------------------------------
void initialize(dwContextHandle_t sdk, const std::string lidarFile, const std::string canFile,
                const std::string dbcFile, const std::string videoFile,
                const std::string videoTimestampFile)
{

    dwStatus status = dwSAL_initialize(&g_sal, sdk);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot create SAL, exiting..." << std::endl;
        throw std::exception();
    }

    std::string params = "";
    dwSensorParams sensorParams{};
    sensorParams.protocol   = "lidar.virtual";
    params                  = std::string("file=") + lidarFile;
    sensorParams.parameters = params.c_str();
    dwSAL_createSensor(&g_lidarSensor, sensorParams, g_sal);
    sensorParams.protocol   = "can.virtual";
    params                  = std::string("file=") + canFile;
    sensorParams.parameters = params.c_str();
    dwSAL_createSensor(&g_canSensor, sensorParams, g_sal);
    sensorParams.protocol = "camera.virtual";
    params                = std::string("video=") + videoFile + std::string(",timestamp=") +
             videoTimestampFile;
    sensorParams.parameters = params.c_str();
    dwSAL_createSensor(&g_cameraSensor, sensorParams, g_sal);

    g_occupancyGridInitParams.cellSizeMeters         = 0.05f;
    g_occupancyGridInitParams.gridDimensionsMeters.x = 80;
    g_occupancyGridInitParams.gridDimensionsMeters.y = 80;
    g_occupancyGridInitParams.gridDimensionsMeters.z = 3.0f;

    g_occupancyGridInitParams.gridMinMeters.x = -0.5f * g_occupancyGridInitParams.gridDimensionsMeters.x;
    g_occupancyGridInitParams.gridMinMeters.y = -0.5f * g_occupancyGridInitParams.gridDimensionsMeters.y;
    g_occupancyGridInitParams.gridMinMeters.z = 0.2f;

    g_occupancyGridInitParams.renderColor.x  = 1.0f;
    g_occupancyGridInitParams.renderColor.y  = 1.0f;
    g_occupancyGridInitParams.renderColor.z  = 1.0f;
    g_occupancyGridInitParams.renderColor.w  = 1.0f;
    g_occupancyGridInitParams.isScrollingMap = DW_FALSE;

    RENDER_ROTATION_ARROW = (g_occupancyGridInitParams.isScrollingMap == DW_TRUE);

    dwStatus g_status = dwOccupancyGrid_initialize(&g_occupancyGrid, &g_occupancyGridInitParams, sdk);
    if (g_status != DW_SUCCESS) {
        std::cerr << "Cannot initialize occupancy grid, exiting..." << std::endl;
        throw std::exception();
    }

    // Flip the y and z because lidar is upside down
    static float32_t sensorToRig[16] = {
        1.f, 0.f, 0.f, 0.f,
        0.f, -1.f, 0.f, 0.f,
        0.f, 0.f, -1.f, 0.f,
        0.f, 0.f, 0.f, 1.f};

    dwOccupancyGridLayerParameters layerParameters{};
    layerParameters.minValidDistanceMeters = 0.0f;
    layerParameters.maxValidDistanceMeters = 200.f;
    memcpy(layerParameters.sensorToRigTransformation.array, sensorToRig,
           sizeof(float32_t) * 16);
    layerParameters.minAccumulatedProbability       = 0.01f;
    layerParameters.maxAccumulatedProbability       = 0.999f;
    layerParameters.probabilityRayFreeAtOrigin      = 0.51f;
    layerParameters.probabilityRayFreeAtMaxDistance = 0.5f;
    layerParameters.probabilityFreeAtHit            = 0.39f;
    layerParameters.probabilityRayFreeBeyondHit     = 0.5f;
    layerParameters.isCumulative                    = DW_TRUE;
    layerParameters.isRaycast                       = DW_TRUE;
    layerParameters.rayWidth                        = 6.0f;
    layerParameters.hitWidth                        = 4.0f;

    for (size_t i = 0; i < 1; ++i) {
        if (i % 2 == 1) {
            layerParameters.isRaycast = DW_FALSE;
        } else {
            layerParameters.isRaycast = DW_TRUE;
        }

        dwOccupancyGrid_setLayerProperties(i, &layerParameters, g_occupancyGrid);
    }

    dwStatus result = dwCANInterpreter_buildFromDBC(&g_canInterpreter,
                                                    dbcFile.c_str(),
                                                    sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot create callback based CAN message interpreter" << std::endl;
    }

    // initialize Egomotion module
    dwEgoMotionParameters egoparams{};
    egoparams.wheelBase   = 2.9f; // meter
    egoparams.motionModel = DW_EGOMOTION_ODOMETRY;

    result            = dwEgomotion_initialize(&g_egomotion, &egoparams, sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Error dwEgomotion_initialize: " << dwGetStatusName(result) << std::endl;
    }

    dwImageProperties cameraProps{};
    dwSensorCamera_getImageProperties(&cameraProps, DW_CAMERA_PROCESSED_IMAGE, g_cameraSensor);

    dwImageProperties displayProps = cameraProps;
    displayProps.pxlType           = DW_TYPE_UINT8;
    displayProps.pxlFormat         = DW_IMAGE_RGBA;
    displayProps.planeCount        = 1;

    result = dwImageFormatConverter_initialize(&g_yuv2RGBA, cameraProps.type, sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Error dwImageFormatConverter_initialize: " << dwGetStatusName(result) << std::endl;
    }

    result = dwImageStreamer_initialize(&g_image2GL, &displayProps, DW_IMAGE_GL, sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Error dwImageStreamer_initialize: " << dwGetStatusName(result) << std::endl;
    }

    result = dwRenderer_initialize(&g_renderer, sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Error dwRenderer_initialize: " << dwGetStatusName(result) << std::endl;
    }

#ifdef VIBRANTE
    g_nvmediaRGBA = new dwImageNvMedia();
    dwImageProperties properties = displayProps;
    properties.type = DW_IMAGE_NVMEDIA;
    properties.pxlFormat = DW_IMAGE_RGBA;
    properties.pxlType = DW_TYPE_UINT8;
    properties.planeCount = 1;
    dwImageNvMedia_create(g_nvmediaRGBA, &properties, sdk);

#else
    g_cudaRGBA = new dwImageCUDA;
    {
        void *dptr          = nullptr;
        size_t pitch        = 0;
        cudaError_t cudaErr = cudaMallocPitch(&dptr, &pitch, displayProps.width * 4, displayProps.height);
        if (cudaErr != cudaSuccess) {
            std::cerr << "cuda malloc failed." << std::endl;
        }
        dwImageCUDA_setFromPitch(g_cudaRGBA, dptr, displayProps.width,
                                 displayProps.height, static_cast<uint32_t>(pitch), DW_IMAGE_RGBA);
    }
#endif

    dwSensor_start(g_cameraSensor);

    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

    if (RENDER_ROTATION_ARROW) {
        dwRenderBuffer_initialize(&g_lineBuffer, layout, DW_RENDER_PRIM_LINELIST, 3, sdk);

        float32_t *map;
        uint32_t maxVerts, stride;
        dwVector3f back{0.0f, -0.02f, 0.0f};
        dwVector3f tip{0.0f, 0.02f, 0.0f};
        dwVector3f wing{0.02f, 0.0f, 0.0f};
        if (dwRenderBuffer_map(&map, &maxVerts, &stride, g_lineBuffer) == DW_SUCCESS) {
            map[0] = back.x;
            map[1] = back.y;
            map[2] = back.z;
            map[3] = tip.x;
            map[4] = tip.y;
            map[5] = tip.z;

            map[6]  = tip.x;
            map[7]  = tip.y;
            map[8]  = tip.z;
            map[9]  = wing.x;
            map[10] = wing.y;
            map[11] = wing.z;

            map[12] = tip.x;
            map[13] = tip.y;
            map[14] = tip.z;
            map[15] = -wing.x;
            map[16] = -wing.y;
            map[17] = -wing.z;

            dwRenderBuffer_unmap(maxVerts, g_lineBuffer);
        }
    }
}

//------------------------------------------------------------------------------
void release()
{
    if (g_lidarSensor != DW_NULL_HANDLE) {
        dwSAL_releaseSensor(&g_lidarSensor);
    }

    if (g_cameraSensor != DW_NULL_HANDLE) {
        dwSAL_releaseSensor(&g_cameraSensor);
    }

    if (g_canSensor != DW_NULL_HANDLE) {
        dwSAL_releaseSensor(&g_canSensor);
    }

    if (g_canInterpreter != DW_NULL_HANDLE) {
        dwCANInterpreter_release(&g_canInterpreter);
    }

    if (g_sal != DW_NULL_HANDLE) {
        dwSAL_release(&g_sal);
    }

    if (g_egomotion != DW_NULL_HANDLE) {
        dwEgomotion_release(&g_egomotion);
    }

#ifdef VIBRANTE
    if (g_nvmediaRGBA != nullptr) {
        NvMediaImageDestroy(g_nvmediaRGBA->img);
        delete g_nvmediaRGBA;
    }
#else
    if (g_cudaRGBA != nullptr) {
        cudaFree(g_cudaRGBA->dptr[0]);
        delete g_cudaRGBA;
    }
#endif

    if (g_yuv2RGBA != DW_NULL_HANDLE) {
        dwImageFormatConverter_release(&g_yuv2RGBA);
    }

    if (g_image2GL != DW_NULL_HANDLE) {
        dwImageStreamer_release(&g_image2GL);
    }

    if (g_renderer != DW_NULL_HANDLE) {
        dwRenderer_release(&g_renderer);
    }

    if (RENDER_ROTATION_ARROW) {
        if (g_lineBuffer) {
            dwRenderBuffer_release(&g_lineBuffer);
        }
    }

    if (g_occupancyGrid) {
        dwOccupancyGrid_release(&g_occupancyGrid);
    }
}

//------------------------------------------------------------------------------
void feedCANToEgomotion(dwTime_t *outTimestamp)
{
    dwTime_t canTimestamp   = 0;
    float32_t kph           = 0.0f;
    float32_t steeringAngle = 0.0f;
    bool hasSpeed           = false;
    bool hasSteering        = false;
    nextCANMeasurement(canTimestamp, kph, steeringAngle, hasSpeed, hasSteering);

    dwMotionModelMeasurement type = DW_EGOMOTION_MEASURMENT_VELOCITY;

    if (hasSteering) {
        float32_t rads  = static_cast<float32_t>((steeringAngle / 16.0f) * M_PI / 180.0f);
        type            = DW_EGOMOTION_MEASURMENT_STEERINGANGLE;
        dwStatus status = dwEgomotion_addOdometry(type, rads, canTimestamp, g_egomotion);
        if (status != DW_SUCCESS) {
            printf("Error dwEgomotion_motionMeasurement: %s\n", dwGetStatusName(status));
            return;
        }
    }

    if (hasSpeed) {
        float mph             = kph * 0.621731f;
        float metersPerSecond = kph / 3.6f;

        if (mph <= 0.0f) {
            g_arrowColor = &DW_RENDERER_COLOR_RED[0];
        } else if (mph > 10.0f && mph <= 25.0f) {
            g_arrowColor = &DW_RENDERER_COLOR_YELLOW[0];
        } else if (mph > 25.0f) {
            g_arrowColor = &DW_RENDERER_COLOR_GREEN[0];
        }
        type            = DW_EGOMOTION_MEASURMENT_VELOCITY;
        dwStatus status = dwEgomotion_addOdometry(type, metersPerSecond, canTimestamp, g_egomotion);
        if (status != DW_SUCCESS) {
            printf("Error dwEgomotion_motionMeasurement: %s\n", dwGetStatusName(status));
            return;
        }
    }

    if (!hasSpeed && !hasSteering) {
        return;
    }

    *outTimestamp = canTimestamp;
}

//------------------------------------------------------------------------------
dwStatus getCameraFrame(dwTime_t *outTimestamp, dwImageGL **outGLFrame, dwImageGL *lastGLFrame)
{
    dwCameraFrameHandle_t frameHandle = DW_NULL_HANDLE;

    if (lastGLFrame != nullptr) {
        dwImageStreamer_returnReceivedGL(lastGLFrame, g_image2GL);

#ifdef VIBRANTE
        dwImageNvMedia *temp = nullptr;
        dwImageStreamer_waitPostedNvMedia(&temp, 30000, g_image2GL);
#else
        dwImageCUDA *temp = nullptr;
        dwImageStreamer_waitPostedCUDA(&temp, 30000, g_image2GL);
#endif
    }

    dwStatus status = dwSensorCamera_readFrame(&frameHandle, 0, 100000, g_cameraSensor);
    if (status != DW_SUCCESS) {
        return status;
    }

#ifdef VIBRANTE
    dwImageNvMedia *nvmediaYUV = nullptr;

    status = dwSensorCamera_getImageNvMedia(&nvmediaYUV, DW_CAMERA_PROCESSED_IMAGE, frameHandle);

    if (status != DW_SUCCESS) {
        std::cerr << "Cannot get image: " << dwGetStatusName(status) << std::endl;
        return status;
    }

    *outTimestamp = nvmediaYUV->timestamp_us;
    // NvMedia copy conversion
    // convert NvMedia YUV image to RGBA
    status = dwImageFormatConverter_copyConvertNvMedia(g_nvmediaRGBA,
                                                       nvmediaYUV,
                                                       g_yuv2RGBA);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot convert frame YUV to RGBA: " << dwGetStatusName(status) << std::endl;
        return status;
    }
#else
    dwImageCUDA *cudaYUV = nullptr;

    status = dwSensorCamera_getImageCUDA(&cudaYUV, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot get image: " << dwGetStatusName(status) << std::endl;
        return status;
    }
    *outTimestamp = cudaYUV->timestamp_us;
    // CUDA copy conversion
    // convert CUDA YUV image to RGBA
    status = dwImageFormatConverter_copyConvertCUDA(g_cudaRGBA,
                                                    cudaYUV,
                                                    g_yuv2RGBA,
                                                    (cudaStream_t)0);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot convert frame YUV to RGBA: " << dwGetStatusName(status) << std::endl;
        return status;
    }
#endif

    dwSensorCamera_returnFrame(&frameHandle);

#ifdef VIBRANTE
    status = dwImageStreamer_postNvMedia(g_nvmediaRGBA, g_image2GL);
#else
    status = dwImageStreamer_postCUDA(g_cudaRGBA, g_image2GL);
#endif

    if (status != DW_SUCCESS) {
        std::cerr << "Cannot post RGBA image: " << dwGetStatusName(status) << std::endl;
    }

    dwImageGL *frameGL = nullptr;
    status             = dwImageStreamer_receiveGL(&frameGL, 30000, g_image2GL);
    if (status != DW_SUCCESS) {
        std::cerr << "Did not received GL frame within 30ms" << std::endl;
    }
    *outGLFrame = frameGL;

    return status;
}

//------------------------------------------------------------------------------
void drawCamera(dwImageGL *glFrame, WindowBase *window)
{

    if (glFrame == nullptr) {
        return;
    }
    int32_t camWidth  = static_cast<int32_t>(glFrame->prop.width / 8);
    int32_t camHeight = static_cast<int32_t>(glFrame->prop.height / 8);
    dwRect cameraRect{0, window->height() - camHeight,
                      camWidth,
                      camHeight};

    dwRenderer_setRect(cameraRect, g_renderer);
    dwRenderer_renderTexture(glFrame->tex, glFrame->target, g_renderer);
}

//------------------------------------------------------------------------------
void printMatrix(const dwTransformation &transformation)
{
    printf("MATRIX: \n");
    for (int32_t index = 0; index < 4; ++index) {
        int32_t i = index * 4;
        printf("    %f, %f, %f, %f\n", transformation.array[i + 0],
               transformation.array[i + 1],
               transformation.array[i + 2],
               transformation.array[i + 3]);
    }
}

//------------------------------------------------------------------------------
dwStatus computeOccupancyGrid(dwTime_t *outTimestamp, size_t *pointCount)
{
    const dwLidarDecodedPacket *nextPacket = nullptr;

    dwStatus status = dwSensorLidar_readPacket(&nextPacket, 10000, g_lidarSensor);
    if (status != DW_SUCCESS) {
        std::cerr << "Could not read from lidar..." << std::endl;
        *outTimestamp = 0;
        return status;
    }

    dwTransformation world2Rig{};
    Mat4_IsoInv(world2Rig.array, g_rig2World.array);

    *outTimestamp = nextPacket->hostTimestamp;

    dwSensorLidar_returnPacket(nextPacket, g_lidarSensor);

    status = dwOccupancyGrid_update(&world2Rig,
                                    0,
                                    g_occupancyGrid);

    if (status != DW_SUCCESS) {
        std::cerr << dwGetStatusName(status) << std::endl;
        return status;
    }

    status = dwOccupancyGrid_insertPointList(static_cast<const dwVector4f *>(nextPacket->points),
                                             nextPacket->nPoints,
                                             0,
                                             g_occupancyGrid);
    *pointCount = nextPacket->nPoints;
    if (status != DW_SUCCESS) {
        std::cerr << dwGetStatusName(status) << std::endl;
        return status;
    }

    status = dwOccupancyGrid_getGridLayerImage(&g_gridImage, 0, g_occupancyGrid);
    if (status != DW_SUCCESS) {
        std::cerr << dwGetStatusName(status) << std::endl;
        return status;
    }

    return status;
}

void calculateFPS()
{
    // Measure speed
    static auto lastTime       = std::chrono::high_resolution_clock::now();
    static uint64_t frameCount = 0;
    auto currentTime           = std::chrono::high_resolution_clock::now();
    frameCount++;
    dwTime_t microseconds = static_cast<dwTime_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            currentTime - lastTime)
            .count());
    if (microseconds >= 1000000) { // If last prinf() was more than 1 sec ago
        // printf and reset timer
        g_currentFPS = static_cast<float64_t>(frameCount) /
                       microseconds * 1000000.0f;

        frameCount = 0;
        lastTime   = currentTime;
    }
}

//------------------------------------------------------------------------------
void loop(WindowBase *window)
{
    Mat4_identity(g_rig2World.array);
    auto lastTime          = std::chrono::system_clock::now();
    g_gridImage.target     = GL_TEXTURE_2D;
    int64_t minDuration_ms = 1000 / g_frameRate;
    dwStatus status        = dwSensor_start(g_lidarSensor);

    if (status != DW_SUCCESS) {
        std::cerr << "Cannot start lidar sensor, exiting..." << std::endl;
        throw std::exception();
    }

    status = dwSensor_start(g_canSensor);

    if (status != DW_SUCCESS) {
        std::cerr << "Cannot start can sensor, exiting..." << std::endl;
        throw std::exception();
    }

    dwTime_t lastLidarTime = 0;
    dwTime_t lastCANTime   = 0;
    dwTime_t lastEgoUpdate = 0;
    dwImageGL *lastGLImage = nullptr;

    static float32_t rotate90AndFlipY[] = {0.0f, -1.0f, 0.0f,
                                           -1.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 1.0f};
    static float32_t id[] = {1.0f, 0.0f, 0.0f,
                             0.0f, 1.0f, 0.0f,
                             0.0f, 0.0f, 1.0f};
    while (g_run && !window->shouldClose()) {

        dwRect rect{0, 0, window->width(), window->height()};

        dwRect square{(window->width() - window->height()) / 2, 0,
                      window->height(), window->height()};

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        dwRenderer_setRect(rect, g_renderer);

        dwRenderer_set2DTransform(rotate90AndFlipY, g_renderer);

        dwRenderer_renderTexture(g_gridImage.tex, g_gridImage.target, g_renderer);

        std::ostringstream out;
        out << std::setprecision(5) << g_currentFPS;
        std::string fpsString = out.str() + " fps   " +
                                std::to_string(g_currentLidarPointCount) + " points";
        dwRenderer_setColor(DW_RENDERER_COLOR_BLACK, g_renderer);
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_48, g_renderer);
        dwRenderer_renderText(5, 5, fpsString.c_str(), g_renderer);

        dwRenderer_set2DTransform(id, g_renderer);

        drawCamera(lastGLImage, window);

        if (RENDER_ROTATION_ARROW) {
            dwRenderer_setRect(square, g_renderer);
            g_rig2World.array[12]     = g_rig2World.array[13] =
                g_rig2World.array[14] = 0.0f;
            dwRenderer_setModelView(g_rig2World.array, g_renderer);
            dwRenderer_setLineWidth(4.0f, g_renderer);
            dwRenderer_setColor(g_arrowColor, g_renderer);
            dwRenderer_renderBuffer(g_lineBuffer, g_renderer);
        }

        window->swapBuffers();

        auto currentTime     = std::chrono::system_clock::now();
        auto currentDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - lastTime);
        if (currentDuration.count() < minDuration_ms || g_pause) {
            std::this_thread::yield();
            continue;
        }

        lastTime = currentTime;

        // Camera drives the sync and is the master clock
        dwTime_t currentImageTime = 0;
        dwStatus status           = getCameraFrame(&currentImageTime, &lastGLImage, lastGLImage);
        if (status == DW_END_OF_STREAM) {
            currentImageTime = 0;
            lastCANTime      = 0;
            lastLidarTime    = 0;
            lastGLImage      = nullptr;
            dwSensor_reset(g_cameraSensor);
            dwSensor_reset(g_canSensor);
            dwSensor_reset(g_lidarSensor);
            dwOccupancyGrid_reset(g_occupancyGrid);
            dwEgomotion_reset(g_egomotion);
            Mat4_identity(g_rig2World.array);
            continue;
        }

        while (lastCANTime == 0 || lastCANTime < currentImageTime) {
            feedCANToEgomotion(&lastCANTime);
        }

        dwEgomotion_update(currentImageTime, g_egomotion);

        // update current absolute estimate of the pose using relative motion between now and last time
        {
            dwTransformation rigLast2rigNow;
            if (DW_SUCCESS == dwEgomotion_computeRelativeTransformation(&rigLast2rigNow, lastEgoUpdate, currentImageTime, g_egomotion))
            {
                // compute absolute pose given the relative motion between two last estimates
                dwTransformation newRig2World;
                dwEgomotion_applyRelativeTransformation(&newRig2World, &rigLast2rigNow, &g_rig2World);
                g_rig2World = newRig2World;
            }
        }
        lastEgoUpdate = currentImageTime;

        if (lastLidarTime < currentImageTime)
            g_currentLidarPointCount = 0;
        while (lastLidarTime == 0 || lastLidarTime < currentImageTime) {
            size_t pointCount = 0;
            computeOccupancyGrid(&lastLidarTime, &pointCount);
            g_currentLidarPointCount += pointCount;
        }

        calculateFPS();
    }
}

//------------------------------------------------------------------------------
int main(int argc, const char *argv[])
{
    //SDK objects
    WindowBase *window    = nullptr;
    dwContextHandle_t sdk = DW_NULL_HANDLE;

#if (!WINDOWS)
    struct sigaction action = {};
    action.sa_handler       = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
    sigaction(SIGSTOP, &action, NULL); // kill command
#endif
    g_run = true;

    ProgramArguments arguments(
        {ProgramArguments::Option_t("canFile",
                                    (DataPath::get() +
                                     "/samples/mapping/occupancy_grid/can_vehicle.bin")
                                        .c_str()),
         ProgramArguments::Option_t("dbcFile",
                                    (DataPath::get() +
                                     "/samples/mapping/occupancy_grid/DataspeedByWire.dbc")
                                        .c_str()),
         ProgramArguments::Option_t("lidarFile",
                                    (DataPath::get() +
                                     "/samples/mapping/occupancy_grid/lidar_0_front-center.bin")
                                        .c_str()),
         ProgramArguments::Option_t("videoFile",
                                    (DataPath::get() +
                                     "/samples/mapping/occupancy_grid/video_front_center.h264")
                                        .c_str()),
         ProgramArguments::Option_t("videoTimestampFile",
                                    (DataPath::get() +
                                     "/samples/mapping/occupancy_grid/video_time_0.txt")
                                        .c_str()),
         ProgramArguments::Option_t("fps", "30")});

    if (!arguments.parse(argc, argv))
        return -1; // Exit if not all require arguments are provided

    initGL(&window);
    initSdk(&sdk, window);

    g_frameRate = std::atof(arguments.get("fps").c_str());
    if (g_frameRate < 1) {
        std::cerr << "fps must be set to at least 1...setting fps=1." << std::endl;
        g_frameRate = 1;
    }
    initialize(sdk, arguments.get("lidarFile"), arguments.get("canFile"),
               arguments.get("dbcFile"), arguments.get("videoFile"),
               arguments.get("videoTimestampFile"));

    loop(window);
    release();

    // release used objects in correct order
    dwRelease(&sdk);
    dwLogger_release();
    delete window;

    return 0;
}
