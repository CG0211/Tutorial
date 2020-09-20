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

#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif


// Driveworks includes
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/features/Features.h>
#include <dw/renderer/Renderer.h>

// Driveworks sample includes
#include <framework/DataPath.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/Log.hpp>
#include <framework/Checks.hpp>

// System includes
#include <memory>
#include <thread>
#include <iostream>
#include <sstream>
#include <cstring>
#include <signal.h>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

// This sample includes
#include "ISensorIO.hpp"
#include "SensorIOCuda.hpp"

#ifdef VIBRANTE
#include "SensorIONvmedia.hpp"
#endif

//------------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------------
//Program arguments
std::unique_ptr<ProgramArguments> m_arguments;
uint32_t m_imageWidth;
uint32_t m_imageHeight;
uint32_t m_maxFeatureCount;
uint32_t m_historyCapacity;
cudaStream_t m_cudaStream;

// Main loop control
volatile bool m_run;
volatile bool m_pause;
volatile bool m_drawHistory;

typedef std::chrono::high_resolution_clock myclock_t;
typedef std::chrono::time_point<myclock_t> timepoint_t;
timepoint_t m_lastUpdateTime = myclock_t::now();

// GL
std::unique_ptr<WindowBase> m_window;

// SDK Context
dwContextHandle_t m_context;

//SAL and sensor
dwSALHandle_t m_sal;
dwSensorHandle_t m_cameraSensor;
std::unique_ptr<ISensorIO> m_sensorIO;

// Tracker
dwFeatureTrackerHandle_t m_tracker;
dwPyramidHandle_t m_pyramidPrevious;
dwPyramidHandle_t m_pyramidCurrent;

//Feature data
dwFeatureListHandle_t m_featureList;

//These point into the buffers of m_featureList
size_t m_featureDataSize;
void *m_d_featureDataBase;
dwFeatureListPointers m_d_featureData;

//CPU copies of the data in m_featureList
std::unique_ptr<uint8_t[]> m_featureDataBuffer;
dwFeatureListPointers m_featureData;

//Buffers used to select features for list compacting
uint32_t *m_d_validFeatureCount;
uint32_t *m_d_validFeatureIndexes;
uint32_t *m_d_invalidFeatureCount;
uint32_t *m_d_invalidFeatureIndexes;

//Draw data
dwRendererHandle_t m_renderer;
dwRenderBufferHandle_t m_featuresRenderBuffer;
dwRenderBufferHandle_t m_featureHistoryRenderBuffer;

//------------------------------------------------------------------------------
// Functions used by this sample
//------------------------------------------------------------------------------

bool initialize(int argc, const char **argv);
void cleanup();
bool processArguments(int argc, const char **argv);
void initGL(int width, int height);
void initSDK();
void initRenderer();
void initSensor();
void initImageBuffers(int imageWidth, int imageHeight);

void processKey(int key);

int run();
void trackFrame(dwImageCUDA *image);
void compactFeatures();
const float32_t *getFeatureRenderingColor(uint32_t age);
void draw(dwImageGL *imageRGBA);

int main(int argc, const char **argv);

//------------------------------------------------------------------------------
// Function implementations
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
bool initialize(int argc, const char **argv)
{
    //Default global variable init
    m_imageWidth      = 0;
    m_imageHeight     = 0;
    m_maxFeatureCount = 8000;
    m_historyCapacity = 10;
    m_cudaStream      = 0;
    m_drawHistory     = true;
    m_window          = nullptr;
    m_context         = DW_NULL_HANDLE;
    m_sal             = DW_NULL_HANDLE;
    m_cameraSensor = DW_NULL_HANDLE;
    m_d_validFeatureCount   = nullptr;
    m_d_validFeatureIndexes = nullptr;
    m_d_invalidFeatureCount   = nullptr;
    m_d_invalidFeatureIndexes = nullptr;
    cudaMalloc(&m_d_validFeatureCount, sizeof(uint32_t));
    cudaMalloc(&m_d_validFeatureIndexes, m_maxFeatureCount * sizeof(uint32_t));
    cudaMalloc(&m_d_invalidFeatureCount, sizeof(uint32_t));
    cudaMalloc(&m_d_invalidFeatureIndexes, m_maxFeatureCount * sizeof(uint32_t));

    //Process arguments
    if (!processArguments(argc, argv))
        return false;

    //Init components
    initGL(1280, 800);
    initSDK();
    initSensor();
    initRenderer();

    return true;
}

void cleanup()
{
    m_arguments.reset();

    //Renderer
    dwRenderer_release(&m_renderer);
    dwRenderBuffer_release(&m_featuresRenderBuffer);
    dwRenderBuffer_release(&m_featureHistoryRenderBuffer);

    // Tracker
    dwFeatureTracker_release(&m_tracker);
    dwPyramid_release(&m_pyramidPrevious);
    dwPyramid_release(&m_pyramidCurrent);

    //Feature data
    dwFeatureList_release(&m_featureList);

    //SAL and sensor
    m_sensorIO.reset();
    dwSAL_releaseSensor(&m_cameraSensor);
    dwSAL_release(&m_sal);

    // SDK Context
    dwRelease(&m_context);
    dwLogger_release();

    //CPU copies of the data in m_featureList
    m_featureDataBuffer.reset();

    //Buffers used to select features for list compacting
    if (m_d_validFeatureCount)
        cudaFree(m_d_validFeatureCount);
    if (m_d_validFeatureIndexes)
        cudaFree(m_d_validFeatureIndexes);

    if (m_d_invalidFeatureCount)
        cudaFree(m_d_invalidFeatureCount);
    if (m_d_invalidFeatureIndexes)
        cudaFree(m_d_invalidFeatureIndexes);

    // GL
    m_window.reset();

    if (m_cudaStream)
        cudaStreamDestroy(m_cudaStream);
}

//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;
    m_run = false;
}

//------------------------------------------------------------------------------
void processKey(int key)
{
    // stop application
    if (key == GLFW_KEY_ESCAPE)
        m_run = false;
    else if (key == GLFW_KEY_SPACE)
        m_pause = !m_pause;
    else if (key == GLFW_KEY_H)
        m_drawHistory = !m_drawHistory;
}

//------------------------------------------------------------------------------
bool processArguments(int argc, const char **argv)
{
    //Define expected arguments
    m_arguments.reset(new ProgramArguments({
        ProgramArguments::Option_t("video",
                                   (DataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
    }));

    //Parse
    if (!m_arguments->parse(argc, argv))
        return false;

    //Do extra stuff with arguments
    std::cout << "Program Arguments:\n" << m_arguments->printList() << std::endl;

    return true;
}

//------------------------------------------------------------------------------
void initGL(int width, int height)
{
    // Initialize the GL and GLFW
    m_window.reset(new WindowGLFW(width, height));

    m_window->makeCurrent();
    m_window->setOnKeypressCallback(processKey);

    //Clear
    glClearColor(0, 1, 0, 0);
    glViewport(0, 0, width, height);
    CHECK_GL_ERROR();
}

//------------------------------------------------------------------------------
void initSDK()
{
    dwStatus result = DW_SUCCESS;

    result = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init logger: ") + dwGetStatusName(result));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = m_window->getEGLDisplay();
#endif

    result = dwInitialize(&m_context, DW_VERSION, &sdkParams);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init driveworks: ") +
                                 dwGetStatusName(result));
}

//------------------------------------------------------------------------------
void initRenderer()
{
    dwStatus result;

    result = dwRenderer_initialize(&m_renderer, m_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init renderer: ") +
                                 dwGetStatusName(result));

    // Set some renderer defaults
    dwRect rect;
    rect.width   = m_window->width();
    rect.height  = m_window->height();
    rect.x = 0;
    rect.y = 0;

    dwRenderer_setRect(rect, m_renderer);

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

    dwRenderer_set2DTransform(rasterTransform, m_renderer);
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, m_renderer);

    // Point cloud
    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
    layout.colFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

    dwRenderBuffer_initialize(&m_featuresRenderBuffer, layout, DW_RENDER_PRIM_POINTLIST, m_maxFeatureCount, m_context);
    dwRenderBuffer_set2DCoordNormalizationFactors((float)m_imageWidth, (float)m_imageHeight, m_featuresRenderBuffer);

    dwRenderBuffer_initialize(&m_featureHistoryRenderBuffer, layout, DW_RENDER_PRIM_LINELIST, m_maxFeatureCount * 2 * m_historyCapacity, m_context);
    dwRenderBuffer_set2DCoordNormalizationFactors((float)m_imageWidth, (float)m_imageHeight, m_featureHistoryRenderBuffer);
}

//------------------------------------------------------------------------------
void initSensor()
{
    dwStatus result;

    // create HAL module of the SDK
    dwSAL_initialize(&m_sal, m_context);

    // create Camera virtual sensor
    {
        dwSensorParams params;
        std::string parameterString = m_arguments->parameterString();
        params.parameters           = parameterString.c_str();
        params.protocol             = "camera.virtual";
        result = dwSAL_createSensor(&m_cameraSensor, params, m_sal);
        if (result != DW_SUCCESS) {
            std::stringstream ss;
            ss << "Cannot create driver: camera.virtual with params: "
               << params.parameters << std::endl;
            throw std::runtime_error(ss.str());
        }
    }

    float32_t cameraFramerate;
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties,
                                     DW_CAMERA_PROCESSED_IMAGE,
                                     m_cameraSensor);

    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, m_cameraSensor);
    cameraFramerate = cameraProperties.framerate;

    uint32_t imageWidth = cameraImageProperties.width;
    uint32_t imageHeight = cameraImageProperties.height;
    dwImageType cameraImageType = cameraImageProperties.type;
    std::cout << "Camera image with " << imageWidth << "x"
              << imageHeight << " at " << cameraFramerate
              << " FPS" << std::endl;

    //Init buffers for rendering
    initImageBuffers(imageWidth, imageHeight);

    if (cameraImageType == DW_IMAGE_CUDA) {
        m_sensorIO.reset(new SensorIOCuda(m_context, m_cudaStream, m_cameraSensor,
                                          imageWidth, imageHeight));
    }
#ifdef VIBRANTE
    else if (cameraImageType == DW_IMAGE_NVMEDIA) {
        m_sensorIO.reset(new SensorIONvmedia(m_context, m_cudaStream, m_cameraSensor,
                                         imageWidth, imageHeight));
    }
#endif
    else {
        throw std::runtime_error("Camera image type is not support,"
                                 " expected DW_IMAGE_CUDA or DW_IMAGE_NVMEDIA");
    }
}

//------------------------------------------------------------------------------
void initImageBuffers(int imageWidth, int imageHeight)
{
    m_imageWidth  = imageWidth;
    m_imageHeight = imageHeight;

    //Streamer
    dwStatus result;

    //Tracker
    dwFeatureTrackerConfig trackerConfig;
    trackerConfig.imageWidth             = imageWidth;
    trackerConfig.imageHeight            = imageHeight;
    trackerConfig.detectorScoreThreshold = 0.1f;
    trackerConfig.windowSizeLK           = 10;
    trackerConfig.interationsLK          = 10;
    trackerConfig.maxFeatureCount        = m_maxFeatureCount;
    result = dwFeatureTracker_initialize(&m_tracker, m_context, m_cudaStream, trackerConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init tracker: ") + dwGetStatusName(result));

    result = dwFeatureList_initialize(&m_featureList, m_context, m_cudaStream,
                                      trackerConfig.maxFeatureCount, m_historyCapacity,
                                      imageWidth, imageHeight);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init features list: ") + dwGetStatusName(result));

    dwFeatureList_getDataBasePointer(&m_d_featureDataBase, &m_featureDataSize, m_featureList);
    dwFeatureList_getDataPointers(&m_d_featureData, m_d_featureDataBase, m_featureList);

    m_featureDataBuffer.reset(new uint8_t[m_featureDataSize]);
    dwFeatureList_getDataPointers(&m_featureData, m_featureDataBuffer.get(), m_featureList);

    dwPyramidConfig pyramidConfig{};
    pyramidConfig.width      = imageWidth;
    pyramidConfig.height     = imageHeight;
    pyramidConfig.levelCount = 3;
    pyramidConfig.dataType   = DW_TYPE_UINT8;
    result = dwPyramid_initialize(&m_pyramidPrevious, m_context, m_cudaStream, pyramidConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init pyramid: ") + dwGetStatusName(result));
    result = dwPyramid_initialize(&m_pyramidCurrent, m_context, m_cudaStream, pyramidConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init pyramid: ") + dwGetStatusName(result));
}

//------------------------------------------------------------------------------
int run()
{
    dwStatus result;

    //Run
    result = dwSensor_start(m_cameraSensor);
    if (result != DW_SUCCESS) {
        throw std::runtime_error("Cannot start sensor");
    }

    dwImageGL *frameGL = nullptr;

    m_run = true;
    while (m_run && !m_window->shouldClose()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        std::this_thread::yield();

        bool processImage = true;

        if (m_pause)
            processImage = false;

        //Check time
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(myclock_t::now() - m_lastUpdateTime);
        if (timeSinceUpdate < std::chrono::milliseconds(33))
            processImage = false;

        //Process
        if (processImage) {
            m_lastUpdateTime = myclock_t::now();

            if (frameGL) {
                m_sensorIO->releaseGLRgbaFrame();
                m_sensorIO->releaseFrame();
                frameGL = nullptr;
            }

            //Get next frame
            result = m_sensorIO->getFrame();

            // try different image types
            if (result == DW_END_OF_STREAM) {
                std::cout << "Camera reached end of stream" << std::endl;
                dwSensor_reset(m_cameraSensor);
                dwFeatureList_reset(m_featureList);
                dwPyramid_reset(m_pyramidCurrent);
                dwPyramid_reset(m_pyramidPrevious);
                continue;
            } else if (result != DW_SUCCESS) {
                std::cerr << "Cannot read frame: " << dwGetStatusName(result) << std::endl;
                m_run = false;
                continue;
            }

            //Track
            dwImageCUDA *frameCUDAyuv;
            frameCUDAyuv = m_sensorIO->getCudaYuv();
            trackFrame(frameCUDAyuv);
            m_sensorIO->releaseCudaYuv();

            // YUV -> RGBA -> GL
            frameGL = m_sensorIO->getGlRgbaFrame();
        }

        if (frameGL) {
            draw(frameGL);
        }
    }
    if (frameGL) {
        m_sensorIO->releaseGLRgbaFrame();
        m_sensorIO->releaseFrame();
        frameGL = nullptr;
    }

    dwSensor_stop(m_cameraSensor);

    return 0;
}

//------------------------------------------------------------------------------
void trackFrame(dwImageCUDA *image)
{
    std::swap(m_pyramidCurrent, m_pyramidPrevious);

    //Build pyramid
    dwImageCUDA planeY{};
    dwImageCUDA_getPlaneAsImage(&planeY, image, 0);

    dwStatus result;
    result = dwPyramid_build(&planeY, m_pyramidCurrent);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot build pyrmaid: ") + dwGetStatusName(result));

    dwFeatureTracker_trackFeatures(m_featureList, m_pyramidPrevious, m_pyramidCurrent, 0, m_tracker);

    dwFeatureList_proximityFilter(m_featureList);
    compactFeatures();

    dwFeatureTracker_detectNewFeatures(m_featureList, m_pyramidCurrent, m_tracker);

    //Get feature info to CPU
    cudaMemcpy(m_featureDataBuffer.get(), m_d_featureDataBase, m_featureDataSize, cudaMemcpyDeviceToHost);
}

//------------------------------------------------------------------------------
void compactFeatures()
{
    //Determine which features to throw away
    dwFeatureList_selectValid(m_d_validFeatureCount, m_d_validFeatureIndexes,
                              m_d_invalidFeatureCount, m_d_invalidFeatureIndexes, m_featureList);

    //Compact list
    dwFeatureList_compact(m_featureList,
                          m_d_validFeatureCount, m_d_validFeatureIndexes,
                          m_d_invalidFeatureCount, m_d_invalidFeatureIndexes);
}

// historyIdx should be smaller than m_historyCapacity
const dwVector2f &getFeatureLocation(uint32_t historyIdx, uint32_t featureIdx, uint32_t currentTimeIdx)
{
    uint32_t timeIdx = (currentTimeIdx + historyIdx) % m_historyCapacity;
    return m_featureData.locationHistory[timeIdx * m_maxFeatureCount + featureIdx];
}

const float32_t *getFeatureRenderingColor(uint32_t age)
{
    const float32_t *color;
    if (age < 5) {
        color = DW_RENDERER_COLOR_RED;
    }
    else if (age < 10) {
        color = DW_RENDERER_COLOR_YELLOW;
    }
    else if (age < 20) {
        color = DW_RENDERER_COLOR_GREEN;
    }
    else {
        color = DW_RENDERER_COLOR_LIGHTBLUE;
    }
    return color;
}

//------------------------------------------------------------------------------
void draw(dwImageGL *imageRGBA)
{
    ///////////////////
    //Draw quad
    dwRenderer_renderTexture(imageRGBA->tex, imageRGBA->target, m_renderer);
    CHECK_GL_ERROR();

    uint32_t maxVerts, stride;

    uint32_t currentTimeIdx;
    dwFeatureList_getCurrentTimeIdx(&currentTimeIdx, m_featureList);

    uint32_t drawCount = 0;

    ///////////////////
    //Draw features
    struct
    {
        float pos[2];
        float color[3];
    } * map;

    if (m_drawHistory) {
        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer);
        dwRenderer_setLineWidth(1, m_renderer);

        dwRenderBuffer_map((float **)&map, &maxVerts, &stride, m_featureHistoryRenderBuffer);

        for (uint32_t i = 0; i < *m_featureData.featureCount; i++) {
            uint32_t age = m_featureData.ages[i];

            const float32_t *color = getFeatureRenderingColor(age);

            // age is not capped by historyCapacity, so this operation is necessary when accessing locationHistroy.
            const uint32_t drawAge = std::min(age, m_historyCapacity);

            dwVector2f preFeature = getFeatureLocation(drawAge - 1, i, currentTimeIdx);
            for (int32_t histIdx = static_cast<int32_t>(drawAge) - 2; histIdx >= 0; histIdx--) {
                dwVector2f curFeature = getFeatureLocation(histIdx, i, currentTimeIdx);

                map[drawCount].pos[0]   = preFeature.x;
                map[drawCount].pos[1]   = preFeature.y;
                map[drawCount].color[0] = color[0];
                map[drawCount].color[1] = color[1];
                map[drawCount].color[2] = color[2];
                drawCount++;

                map[drawCount].pos[0]   = curFeature.x;
                map[drawCount].pos[1]   = curFeature.y;
                map[drawCount].color[0] = color[0];
                map[drawCount].color[1] = color[1];
                map[drawCount].color[2] = color[2];
                drawCount++;

                preFeature = curFeature;

                drawCount += 2;
            }
        }

        dwRenderBuffer_unmap(drawCount, m_featureHistoryRenderBuffer);
        dwRenderer_renderBuffer(m_featureHistoryRenderBuffer, m_renderer);
    }
    else {
        dwRenderer_setPointSize(4.0f, m_renderer);

        float2 *locationHistory = reinterpret_cast<float2*>(m_featureData.locationHistory);
        float2 *currentLocations = &locationHistory[currentTimeIdx * m_maxFeatureCount];

        dwRenderBuffer_map((float **)&map, &maxVerts, &stride, m_featuresRenderBuffer);

        if (stride != sizeof(*map) / sizeof(float))
            throw std::runtime_error("Unexpected stride");

        for (uint32_t i = 0; i < *m_featureData.featureCount; i++) {
            const float32_t *color = getFeatureRenderingColor(m_featureData.ages[i]);

            map[drawCount].pos[0]   = currentLocations[i].x;
            map[drawCount].pos[1]   = currentLocations[i].y;
            map[drawCount].color[0] = color[0];
            map[drawCount].color[1] = color[1];
            map[drawCount].color[2] = color[2];
            drawCount++;
        }

        dwRenderBuffer_unmap(drawCount, m_featuresRenderBuffer);
        dwRenderer_renderBuffer(m_featuresRenderBuffer, m_renderer);
    }

    m_window->swapBuffers();
    CHECK_GL_ERROR();
}

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
#if (!WINDOWS)
    struct sigaction action = {};
    action.sa_handler = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
    sigaction(SIGSTOP, &action, NULL); // kill command
#endif

    int res = -1;
    try {
        if (initialize(argc, argv)) {
            res = run();
        }
    } catch (const std::exception &ex) {
        std::cerr << "Unexpected exception: \n" << ex.what() << "\nTerminating app.\n";
        res = -1;
    }
    cleanup();
    return res;
}
