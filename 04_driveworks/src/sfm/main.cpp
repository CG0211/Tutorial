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

// Driveworks sample includes
#include <framework/WindowGLFW.hpp>
#include <framework/DataPath.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/MouseView3D.hpp>
#include <framework/MathUtils.hpp>
#include <framework/Mat4.hpp>
#include <framework/Log.hpp>
#include <framework/ProfilerCUDA.hpp>
#include <framework/Checks.hpp>

#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif

// Driveworks includes
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>
#include <dw/egomotion/Egomotion.h>
#include <dw/features/Features.h>
#include <dw/sfm/SFM.h>
#include <dw/renderer/Renderer.h>

// System includes
#include <memory>
#include <array>
#include <thread>
#include <iostream>
#include <sstream>
#include <cstring>
#include <string>
#include <signal.h>
#include <math.h>

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

//Profiling from tests_common
#include <framework/StatsCounter.hpp>
#include <framework/CudaTimer.hpp>

using namespace dw::common;

//------------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------------
const int CAMERA_COUNT = 4;

const float DETECTOR_THRESHOLD   = 0.01f;
const int TRACKER_WINDOW_SIZE_LK = 10;
const int TRACKER_ITERATIONS_LK  = 10;

const int FEATURE_HISTORY_SIZE = 32;

const int SFM_POSE_HISTORY_LENGTH       = 1000;
const float SFM_MIN_RIG_DISTANCE        = 0.05f;
const int SFM_MIN_TRIANGULATION_ENTRIES = 4;
const float SFM_MAX_REPROJECTION_ERROR  = DEG2RAD(1.2f);
const float SFM_MIN_OBSERVATION_ANGLE   = DEG2RAD(1.0f);

const uint32_t DEFAULT_MAX_FEATURE_COUNT = 2000;
uint32_t g_maxFeatureCount;

//Program arguments
std::unique_ptr<ProgramArguments> g_arguments;
std::string g_rigConfigFilename;
std::string g_canFilename;
std::string g_dbcFilename;
std::string g_dbcSpeed;
std::string g_dbcSteering;
std::string g_timestampsFilename;
std::string g_videoFilenames[4];

uint32_t g_imageWidth      = 0;
uint32_t g_imageHeight     = 0;
cudaStream_t g_cudaStream  = 0;

// Main loop control
volatile bool g_run   = false;
volatile bool g_pause = false;
volatile bool g_playSingleFrame = false;
volatile bool g_resetSensors = false;
volatile bool g_doDetection   = true;
volatile bool g_doVisualPoseEstimation   = true;

typedef std::chrono::high_resolution_clock myclock_t;
typedef std::chrono::time_point<myclock_t> timepoint_t;
timepoint_t g_lastUpdateTime = myclock_t::now();
dwTime_t g_lastImageTimestamp = 0;

// GL
std::unique_ptr<WindowBase> g_window;
MouseView3D g_mouseView;

// SDK Context
dwContextHandle_t g_context = DW_NULL_HANDLE;

// SAL and sensor
dwSALHandle_t g_sal = DW_NULL_HANDLE;
std::unique_ptr<ISensorIO> g_sensorIO;

// Rig configuration
dwRigConfigurationHandle_t g_rigConfig = DW_NULL_HANDLE;

// CAN
dwSensorHandle_t g_canSensor              = DW_NULL_HANDLE;
dwCANInterpreterHandle_t g_canInterpreter = DW_NULL_HANDLE;

// Egomotion
dwVehicle const *g_vehicle;
dwEgomotionHandle_t g_egomotion = DW_NULL_HANDLE;

// Tracker
dwFeatureTrackerHandle_t g_tracker = DW_NULL_HANDLE;

// Reconstructor
dwCameraRigHandle_t g_rig;
dwReconstructorHandle_t g_reconstructor = DW_NULL_HANDLE;

// Information for each camera
dwSensorHandle_t g_cameraSensor[CAMERA_COUNT] = {DW_NULL_HANDLE};

dwPyramidHandle_t g_pyramidPrevious[CAMERA_COUNT] = {DW_NULL_HANDLE};
dwPyramidHandle_t g_pyramidCurrent[CAMERA_COUNT]  = {DW_NULL_HANDLE};

dwFeatureListHandle_t g_featureList[CAMERA_COUNT]      = {DW_NULL_HANDLE};

//These point into the buffers of g_featureList
size_t g_d_featureSize[CAMERA_COUNT];
void *g_d_featureDataBase[CAMERA_COUNT];
dwFeatureListPointers g_d_featureData[CAMERA_COUNT];
uint32_t *g_d_featureCount[CAMERA_COUNT]             = {nullptr};
dwVector2f *g_d_featureLocationHistory[CAMERA_COUNT] = {nullptr};
dwFeatureStatus *g_d_featureStatuses[CAMERA_COUNT]   = {nullptr};

//CPU copies of the data in g_featureList
std::unique_ptr<uint8_t[]> g_featureDataBuffer[CAMERA_COUNT];
dwFeatureListPointers g_featureData[CAMERA_COUNT];
std::unique_ptr<float2[]> g_projectedLocations[CAMERA_COUNT];

//Predicted points
float *g_d_predictedFeatureLocations[CAMERA_COUNT] = {nullptr};
std::unique_ptr<float2[]> g_predictedFeatureLocations[CAMERA_COUNT];

//Reprojected points
float *g_d_projectedLocations[CAMERA_COUNT] = {nullptr};

//Calibrated camera
dwCalibratedCameraHandle_t g_calibrated[CAMERA_COUNT] = {DW_NULL_HANDLE};

//Reconstruction data
float32_t *g_d_worldPoints[CAMERA_COUNT] = {nullptr};
std::unique_ptr<float4[]> g_worldPoints[CAMERA_COUNT];

int32_t g_currentPoseIdx = -1;
dwTransformation g_currentRig2World;

//Pose variables
dwTransformation *g_d_previousRig2World;
dwTransformation *g_d_rig2WorldInitial;
dwTransformation *g_d_rig2WorldCorrection;


//Current sensor data
dwCANMessage g_newCANFrame;
dwImageCUDA *g_newFrames[CAMERA_COUNT] = {nullptr};

//Buffers used to select features for list compacting
uint32_t *g_d_validFeatureCount     = nullptr;
uint32_t *g_d_validFeatureIndexes   = nullptr;
uint32_t *g_d_invalidFeatureCount   = nullptr;
uint32_t *g_d_invalidFeatureIndexes = nullptr;

//Draw data
dwImageGL *g_currentDrawFrames[CAMERA_COUNT] = {nullptr};

dwRendererHandle_t g_renderer                 = DW_NULL_HANDLE;
dwRenderBufferHandle_t g_groundPlane          = DW_NULL_HANDLE;
dwRenderBufferHandle_t g_featuresRenderBuffer = DW_NULL_HANDLE;

dwRenderBufferHandle_t g_featurePredictionsRenderBuffer = DW_NULL_HANDLE;

const int MAX_RENDER_POSES                    = 2000;
dwRenderBufferHandle_t g_poseCANRenderBuffer = DW_NULL_HANDLE;
dwRenderBufferHandle_t g_poseCorrectionRenderBuffer = DW_NULL_HANDLE;
dwRenderBufferHandle_t g_poseRefinedRenderBuffer = DW_NULL_HANDLE;
uint32_t g_poseRenderCount                 = 0;
const float32_t *g_poseColor                  = DW_RENDERER_COLOR_DARKGREEN;

dwRenderBufferHandle_t g_worldPointsRenderBuffer = DW_NULL_HANDLE;

float g_imageRasterTransform[9]; //3x3 col major

const uint32_t VIEWPORT_COUNT = 5;
dwRect g_viewportRects[VIEWPORT_COUNT];
uint32_t g_viewportMaxIdx = 0;

//Profiler
std::unique_ptr<ProfilerCUDA> g_profiler;

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
void constructGrid();

void processKey(int key);

int run();
void trackFrame(int cameraIdx, dwImageCUDA *image, const dwTransformation *previousRigToWorld, const dwTransformation *predictedRigToWorld);
void compactFeatures(int cameraIdx);
void getFeaturesFromGPU(int cameraIdx);
void recordPose(const dwTransformation *poseCAN, const dwTransformation *poseRefined);
void updateViewportSizes();
void draw3D();
void draw(int cameraIdx, dwImageGL *imageRGBA);

int main(int argc, const char **argv);

//------------------------------------------------------------------------------
// Function implementations
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
bool initialize(int argc, const char **argv)
{
    //Process arguments
    if (!processArguments(argc, argv))
        return false;

    //Default global variable init
    cudaStreamCreate(&g_cudaStream);

    cudaMalloc(&g_d_validFeatureCount, sizeof(uint32_t));
    cudaMalloc(&g_d_validFeatureIndexes, g_maxFeatureCount * sizeof(uint32_t));
    cudaMalloc(&g_d_invalidFeatureCount, sizeof(uint32_t));
    cudaMalloc(&g_d_invalidFeatureIndexes, g_maxFeatureCount * sizeof(uint32_t));

    cudaMalloc(&g_d_previousRig2World, sizeof(dwTransformation));
    cudaMalloc(&g_d_rig2WorldInitial, sizeof(dwTransformation));
    cudaMalloc(&g_d_rig2WorldCorrection, sizeof(dwTransformation));

    for (int k = 0; k < CAMERA_COUNT; k++) {
        cudaMalloc(&g_d_predictedFeatureLocations[k], g_maxFeatureCount * 2 * sizeof(float32_t));
        g_predictedFeatureLocations[k].reset(new float2[g_maxFeatureCount]);

        cudaMalloc(&g_d_worldPoints[k], g_maxFeatureCount * 4 * sizeof(float));
        g_worldPoints[k].reset(new float4[g_maxFeatureCount]);

        cudaMalloc(&g_d_projectedLocations[k], g_maxFeatureCount * 2 * sizeof(float));
        g_projectedLocations[k].reset(new float2[g_maxFeatureCount]);
    }

    //Init vars
    Mat4_identity(g_currentRig2World.array);

    //Init components
    initGL(1280, 800);
    initSDK();
    initRenderer();
    initSensor();

    return true;
}

void cleanup()
{
    for (int k = 0; k < CAMERA_COUNT; k++) {
        dwCalibratedCamera_release(&g_calibrated[k]);
        dwPyramid_release(&g_pyramidPrevious[k]);
        dwPyramid_release(&g_pyramidCurrent[k]);
        dwFeatureList_release(&g_featureList[k]);
        dwSAL_releaseSensor(&g_cameraSensor[k]);
        g_featureDataBuffer[k].reset();
        cudaFree(g_d_worldPoints[k]);
        cudaFree(g_d_projectedLocations[k]);
        cudaFree(g_d_predictedFeatureLocations[k]);
        g_projectedLocations[k].reset();
    }

    g_arguments.reset();

    // Objects
    dwCameraRig_release(&g_rig);
    dwFeatureTracker_release(&g_tracker);
    dwReconstructor_release(&g_reconstructor);
    dwEgomotion_release(&g_egomotion);
    dwRigConfiguration_release(&g_rigConfig);
    dwSAL_releaseSensor(&g_canSensor);
    dwCANInterpreter_release(&g_canInterpreter);

    //SAL and sensor
    g_sensorIO.reset();
    dwSAL_release(&g_sal);

    // SDK Context
    dwRelease(&g_context);
    dwLogger_release();

    //Buffers used to select features for list compacting
    cudaFree(g_d_validFeatureCount);
    cudaFree(g_d_validFeatureIndexes);
    cudaFree(g_d_invalidFeatureCount);
    cudaFree(g_d_invalidFeatureIndexes);

    cudaFree(g_d_previousRig2World);
    cudaFree(g_d_rig2WorldInitial);
    cudaFree(g_d_rig2WorldCorrection);

    // GL
    dwRenderBuffer_release(&g_worldPointsRenderBuffer);
    dwRenderBuffer_release(&g_groundPlane);
    dwRenderBuffer_release(&g_featuresRenderBuffer);
    dwRenderBuffer_release(&g_featurePredictionsRenderBuffer);
    dwRenderBuffer_release(&g_poseRefinedRenderBuffer);
    dwRenderBuffer_release(&g_poseCANRenderBuffer);
    dwRenderBuffer_release(&g_poseCorrectionRenderBuffer);
    dwRenderer_release(&g_renderer);
    g_window.reset();
    cudaStreamDestroy(g_cudaStream);
}

//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;
    g_run = false;
}

//------------------------------------------------------------------------------
void processKey(int key)
{
    // stop application
    if (key == GLFW_KEY_ESCAPE)
        g_run = false;
    else if (key == GLFW_KEY_SPACE)
        g_pause = !g_pause;
    else if (key == GLFW_KEY_F5) {
        g_playSingleFrame = !g_playSingleFrame;
        g_pause = g_playSingleFrame;
    }
    else if (key == GLFW_KEY_D)
        g_doDetection = !g_doDetection;
    else if (key == GLFW_KEY_R)
        g_resetSensors = true;
    else if (key == GLFW_KEY_V)
        g_doVisualPoseEstimation = !g_doVisualPoseEstimation;
    else if (key == GLFW_KEY_Q)
        g_viewportMaxIdx = (g_viewportMaxIdx+1) % VIEWPORT_COUNT;
}

//------------------------------------------------------------------------------
bool processArguments(int argc, const char **argv)
{
    //Define expected arguments
    g_arguments.reset(new ProgramArguments({
        ProgramArguments::Option_t("rig",
                                   (DataPath::get() + "/samples/sfm/triangulation/rig.xml").c_str()),
        ProgramArguments::Option_t("can",
                                   (DataPath::get() + "/samples/sfm/triangulation/canbus.can").c_str()),
        ProgramArguments::Option_t("dbc",
                                   (DataPath::get() + "/samples/sfm/triangulation/canbus.dbc").c_str()),
        ProgramArguments::Option_t("dbcSpeed", "M_SPEED.CAN_CAR_SPEED"),
        ProgramArguments::Option_t("dbcSteering", "M_STEERING.CAN_CAR_STEERING"),
        ProgramArguments::Option_t("videoTimestamps",
                                   (DataPath::get() + "/samples/sfm/triangulation/VideoTimestamps.txt").c_str()),
        ProgramArguments::Option_t("video0",
                                   (DataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
        ProgramArguments::Option_t("video1",
                                   (DataPath::get() + "/samples/sfm/triangulation/video_1.h264").c_str()),
        ProgramArguments::Option_t("video2",
                                   (DataPath::get() + "/samples/sfm/triangulation/video_2.h264").c_str()),
        ProgramArguments::Option_t("video3",
                                   (DataPath::get() + "/samples/sfm/triangulation/video_3.h264").c_str()),
        ProgramArguments::Option_t("maxFeatureCount",
                                   std::to_string(DEFAULT_MAX_FEATURE_COUNT).c_str()),
    }));

    //Parse
    if (!g_arguments->parse(argc, argv))
        return false;

    //Do extra stuff with arguments
    std::cout << "Program Arguments:\n" << g_arguments->printList() << std::endl;

    g_rigConfigFilename  = g_arguments->get("rig");
    g_canFilename        = g_arguments->get("can");
    g_dbcFilename        = g_arguments->get("dbc");
    g_dbcSpeed           = g_arguments->get("dbcSpeed");
    g_dbcSteering        = g_arguments->get("dbcSteering");
    g_timestampsFilename = g_arguments->get("videoTimestamps");
    g_videoFilenames[0]  = g_arguments->get("video0");
    g_videoFilenames[1]  = g_arguments->get("video1");
    g_videoFilenames[2]  = g_arguments->get("video2");
    g_videoFilenames[3]  = g_arguments->get("video3");
    g_maxFeatureCount    = std::stoi(g_arguments->get("maxFeatureCount"));

    return true;
}

//------------------------------------------------------------------------------
void initGL(int width, int height)
{
// Initialize the GL and GLFW
    if (!g_window) g_window.reset(new WindowGLFW(width, height));

    g_window->makeCurrent();
    g_window->setOnKeypressCallback(processKey);
    g_window->setOnMouseUpCallback([](int button, float x, float y) { g_mouseView.mouseUp(button, x, y); });
    g_window->setOnMouseDownCallback([](int button, float x, float y) { g_mouseView.mouseDown(button, x, y); });
    g_window->setOnMouseMoveCallback([](float x, float y) { g_mouseView.mouseMove(x, y); });
    g_window->setOnMouseWheelCallback([](float dx, float dy) { g_mouseView.mouseWheel(dx, dy); });

    //Clear
    glClearColor(0, 0, 0, 0);
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
    sdkParams.eglDisplay = g_window->getEGLDisplay();
#endif

    result = dwInitialize(&g_context, DW_VERSION, &sdkParams);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init driveworks: ") + dwGetStatusName(result));

    // Init profiler
    g_profiler.reset(new ProfilerCUDA());

    // create HAL module of the SDK
    result = dwSAL_initialize(&g_sal, g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init SAL: ") + dwGetStatusName(result));

    //Load vehicle configuration
    result = dwRigConfiguration_initializeFromFile(&g_rigConfig, g_context, g_rigConfigFilename.c_str());
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error dwRigConfiguration_initialize: ") + dwGetStatusName(result));

    uint32_t cameraCount = 0;
    result = dwRigConfiguration_getSensorCount(&cameraCount, g_rigConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error dwRigConfiguration_getSensorCount: ") + dwGetStatusName(result));
    else if (cameraCount != CAMERA_COUNT)
        throw std::runtime_error(std::string("Rig configuration has wrong number of sensors, "
                                             "expected ") +
                                 std::to_string(CAMERA_COUNT) + ", actual " + std::to_string(cameraCount));

    result = dwRigConfiguration_getVehicle(&g_vehicle, g_rigConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error cannot load vehicle: ") + dwGetStatusName(result));

    //Egomotion
    dwEgoMotionParameters egoparams{};
    egoparams.wheelBase = g_vehicle->wheelbase;
    egoparams.motionModel = DW_EGOMOTION_ODOMETRY;
    result = dwEgomotion_initialize(&g_egomotion, &egoparams, g_context);

    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init egomotion: ") + dwGetStatusName(result));

    //Camera rig
    result = dwCameraRig_initializeFromConfig(&g_rig,
                                              &cameraCount,
                                              g_calibrated,
                                              CAMERA_COUNT,
                                              g_context,
                                              g_rigConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init rig: ") + dwGetStatusName(result));
    if (cameraCount != CAMERA_COUNT)
        throw std::runtime_error(std::string("Rig configuration has wrong number of cameras, "
                                             "expected ") +
                                 std::to_string(CAMERA_COUNT) + ", actual " + std::to_string(cameraCount));

    //CAN sensor
    dwSensorParams canParams;
    std::string canParamsStr = std::string("file=") + g_canFilename;
    canParams.protocol       = "can.virtual";
    canParams.parameters     = canParamsStr.c_str();
    result = dwSAL_createSensor(&g_canSensor, canParams, g_sal);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init can: ") + dwGetStatusName(result));

    //CAN interpreter
    result = dwCANInterpreter_buildFromDBC(&g_canInterpreter, g_dbcFilename.c_str(), g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error("Cannot create callback based CAN message interpreter");

    //Reconstructor
    dwReconstructorConfig reconstructorConfig;
    dwReconstructor_initConfig(&reconstructorConfig);
    reconstructorConfig.maxFeatureCount              = g_maxFeatureCount;
    reconstructorConfig.maxPoseHistoryLength         = SFM_POSE_HISTORY_LENGTH;
    reconstructorConfig.minRigDistance               = SFM_MIN_RIG_DISTANCE;
    reconstructorConfig.minTriangulationEntries      = SFM_MIN_TRIANGULATION_ENTRIES;
    reconstructorConfig.rig                          = g_rig;
    reconstructorConfig.maxReprojectionErrorAngleRad = SFM_MAX_REPROJECTION_ERROR;
    reconstructorConfig.minNewObservationAngleRad    = SFM_MIN_OBSERVATION_ANGLE;

    result = dwReconstructor_initialize(&g_reconstructor,
                                        g_context,
                                        g_cudaStream,
                                        reconstructorConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init reconstructor: ") + dwGetStatusName(result));
    
    //Experimental: disable cameras
    uint8_t enabledCameras[4] = { 1, 1, 1, 1 };
    dwReconstructor_enableCamerasForPoseEstimation(enabledCameras,g_reconstructor);
}

//------------------------------------------------------------------------------
void initRenderer()
{
    CHECK_GL_ERROR();
    //Init renderer
    dwStatus result;
    result = dwRenderer_initialize(&g_renderer, g_context);
    CHECK_GL_ERROR();
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init renderer: ") + dwGetStatusName(result));

    //Init state
    result = dwRenderer_setPointSize(2.0f, g_renderer);
    CHECK_GL_ERROR();
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot set point size: ") + dwGetStatusName(result));
    result = dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, g_renderer);
    CHECK_GL_ERROR();
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot set font: ") + dwGetStatusName(result));

    g_imageRasterTransform[0 + 0 * 3] = 1;
    g_imageRasterTransform[1 + 0 * 3] = 0;
    g_imageRasterTransform[2 + 0 * 3] = 0;

    g_imageRasterTransform[0 + 1 * 3] = 0;
    g_imageRasterTransform[1 + 1 * 3] = 1;
    g_imageRasterTransform[2 + 1 * 3] = 0;

    g_imageRasterTransform[0 + 2 * 3] = 0;
    g_imageRasterTransform[1 + 2 * 3] = 0;
    g_imageRasterTransform[2 + 2 * 3] = 1;

    result = dwRenderer_set2DTransform(g_imageRasterTransform, g_renderer);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error setting 2D transform: ") + dwGetStatusName(result));

    CHECK_GL_ERROR();
}

//------------------------------------------------------------------------------
void initSensor()
{
    dwStatus result;

    // create Camera virtual sensor
    dwSensorHandle_t sensors[CAMERA_COUNT];
    for (int k = 0; k < CAMERA_COUNT; k++) {
        std::string sensorParams = "video=" + g_videoFilenames[k] + ",timestamp=" + g_timestampsFilename;

        dwSensorParams params;
        params.protocol   = "camera.virtual";
        params.parameters = sensorParams.c_str();
        result = dwSAL_createSensor(&g_cameraSensor[k], params, g_sal);
        if (result != DW_SUCCESS) {
            std::stringstream ss;
            ss << "Cannot create driver: camera.virtual with params: "
               << params.parameters << std::endl;
            throw std::runtime_error(ss.str());
        }
        sensors[k] = g_cameraSensor[k];
    }

    // Note: all videos should be the same size!
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, g_cameraSensor[0]);

    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, g_cameraSensor[0]);

    std::cout << "Camera image with " << cameraImageProperties.width << "x" << cameraImageProperties.height
              << " at " << cameraProperties.framerate << " FPS" << std::endl;

    // Init buffers for rendering
    initImageBuffers(cameraImageProperties.width, cameraImageProperties.height);

    if (cameraImageProperties.type == DW_IMAGE_CUDA) {
        g_sensorIO.reset(new SensorIOCuda(g_context, g_cudaStream, CAMERA_COUNT, sensors,
                                          cameraImageProperties.width, cameraImageProperties.height));
    }
#ifdef VIBRANTE
    else if (cameraImageProperties.type == DW_IMAGE_NVMEDIA) {
        g_sensorIO.reset(new SensorIONvmedia(g_context, g_cudaStream, CAMERA_COUNT, sensors,
                                             cameraImageProperties.width, cameraImageProperties.height));
    }
#endif
    else {
        throw std::runtime_error("Camera image type is not supported,"
                                 " expected DW_IMAGE_CUDA or DW_IMAGE_NVMEDIA");
    }
}

//------------------------------------------------------------------------------
void initImageBuffers(int imageWidth, int imageHeight)
{
    CHECK_GL_ERROR();
    g_imageWidth  = imageWidth;
    g_imageHeight = imageHeight;

    //Streamer
    dwStatus result;

    //Tracker
    dwFeatureTrackerConfig trackerConfig;
    trackerConfig.imageWidth             = imageWidth;
    trackerConfig.imageHeight            = imageHeight;
    trackerConfig.detectorScoreThreshold = DETECTOR_THRESHOLD;
    trackerConfig.windowSizeLK           = TRACKER_WINDOW_SIZE_LK;
    trackerConfig.interationsLK          = TRACKER_ITERATIONS_LK;
    trackerConfig.maxFeatureCount        = g_maxFeatureCount;
    result = dwFeatureTracker_initialize(&g_tracker, g_context, g_cudaStream, trackerConfig);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init tracker: ") + dwGetStatusName(result));

    dwPyramidConfig pyramidConfig;
    pyramidConfig.dataType   = DW_TYPE_UINT8;
    pyramidConfig.width      = imageWidth;
    pyramidConfig.height     = imageHeight;
    pyramidConfig.levelCount = 3;

    for (int k = 0; k < CAMERA_COUNT; k++) {
        result = dwFeatureList_initialize(&g_featureList[k], g_context, g_cudaStream,
                                          trackerConfig.maxFeatureCount, FEATURE_HISTORY_SIZE,
                                          imageWidth, imageHeight);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot init features list: ") + dwGetStatusName(result));

        result = dwFeatureList_getDataBasePointer(&g_d_featureDataBase[k], &g_d_featureSize[k], g_featureList[k]);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot init features list: ") + dwGetStatusName(result));

        result = dwFeatureList_getDataPointers(&g_d_featureData[k], g_d_featureDataBase[k], g_featureList[k]);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot get feature locations: ") + dwGetStatusName(result));

        g_featureDataBuffer[k].reset(new uint8_t[g_d_featureSize[k]]);
        dwFeatureList_getDataPointers(&g_featureData[k], g_featureDataBuffer[k].get(), g_featureList[k]);

        result = dwPyramid_initialize(&g_pyramidPrevious[k], g_context, g_cudaStream, pyramidConfig);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot init pyramid: ") + dwGetStatusName(result));
        result = dwPyramid_initialize(&g_pyramidCurrent[k], g_context, g_cudaStream, pyramidConfig);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot init pyramid: ") + dwGetStatusName(result));

        g_d_featureCount[k] = g_d_featureData[k].featureCount;
        g_d_featureLocationHistory[k] = g_d_featureData[k].locationHistory;
        g_d_featureStatuses[k] = g_d_featureData[k].statuses;
    }

    // Features render buffer
    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

    result = dwRenderBuffer_initialize(&g_featuresRenderBuffer, layout, DW_RENDER_PRIM_POINTLIST, g_maxFeatureCount, g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error intializing render buffer: ") + dwGetStatusName(result));
    dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)g_imageWidth, (float32_t)g_imageHeight, g_featuresRenderBuffer);

    //Predictions render buffer
    result = dwRenderBuffer_initialize(&g_featurePredictionsRenderBuffer, layout, DW_RENDER_PRIM_LINELIST, g_maxFeatureCount, g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error intializing render buffer: ") + dwGetStatusName(result));
    dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)g_imageWidth, (float32_t)g_imageHeight, g_featurePredictionsRenderBuffer);

    //Car poses
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat = DW_RENDER_FORMAT_NULL;

    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    dwRenderBuffer_initialize(&g_poseRefinedRenderBuffer, layout, DW_RENDER_PRIM_LINESTRIP, MAX_RENDER_POSES, g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error intializing poses render buffer: ") + dwGetStatusName(result));

    dwRenderBuffer_initialize(&g_poseCorrectionRenderBuffer, layout, DW_RENDER_PRIM_LINELIST, MAX_RENDER_POSES, g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error intializing poses render buffer: ") + dwGetStatusName(result));

    dwRenderBuffer_initialize(&g_poseCANRenderBuffer, layout, DW_RENDER_PRIM_POINTLIST, MAX_RENDER_POSES, g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error intializing poses render buffer: ") + dwGetStatusName(result));

    // World grid
    constructGrid();

    // World points render buffer
    //const int MAX_WORLD_POINTS = 100 * g_maxFeatureCount;
    layout.posSemantic         = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.posFormat           = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.colSemantic         = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat           = DW_RENDER_FORMAT_NULL;
    layout.texSemantic         = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat           = DW_RENDER_FORMAT_NULL;

    dwRenderBuffer_initialize(&g_worldPointsRenderBuffer, layout, DW_RENDER_PRIM_POINTLIST, g_maxFeatureCount, g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error intializing render buffer: ") + dwGetStatusName(result));
}

//------------------------------------------------------------------------------
void constructGrid()
{
    const float WORLD_GRID_SIZE_IN_METERS = 300.0f;
    const float WORLD_GRID_RES_IN_METERS  = 5.0f;

    // World grid
    int gridResolution = static_cast<int>(WORLD_GRID_SIZE_IN_METERS / WORLD_GRID_RES_IN_METERS);

    // Rendering data
    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

    dwStatus result;
    result = dwRenderBuffer_initialize(&g_groundPlane, layout, DW_RENDER_PRIM_LINELIST, 2 * (gridResolution + 1), g_context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot create ground plane buffer: ") + dwGetStatusName(result));

    // update the data
    float32_t *map;
    uint32_t maxVerts, stride;

    dwRenderBuffer_map(&map, &maxVerts, &stride, g_groundPlane);

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

        y                           = 0.5f * WORLD_GRID_SIZE_IN_METERS;
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

        x                           = 0.5f * WORLD_GRID_SIZE_IN_METERS;
        map[stride * nVertices + 0] = x;
        map[stride * nVertices + 1] = y;
        map[stride * nVertices + 2] = -0.05f;

        nVertices++;
        y = y + WORLD_GRID_RES_IN_METERS;
    }

    dwRenderBuffer_unmap(maxVerts, g_groundPlane);
}

//------------------------------------------------------------------------------
int run()
{
    dwStatus result;
    int32_t frameIdx = -1;

    //Run
    for (int k = 0; k < CAMERA_COUNT; k++) {
        result = dwSensor_start(g_cameraSensor[k]);
        if (result != DW_SUCCESS) {
            throw std::runtime_error("Cannot start sensor");
        }
    }

    bool loadCAN    = true;
    bool loadImages = true;
    dwTime_t lastEgoUpdate = 0;

    // collect pose in absolute coordinates
    dwTransformation egoRig2World;
    Mat4_identity(egoRig2World.array);

    //Start with identity as correction
    dwTransformation previousEgoRig2World;
    Mat4_identity(previousEgoRig2World.array);

    //Record starting positions
    recordPose(&g_currentRig2World, &g_currentRig2World);

    // Starting CAN sensor
    dwSensor_start(g_canSensor);

    g_run = true;
    while (g_run && !g_window->shouldClose()) {
        bool processCAN    = false;
        bool processImages = false;

        std::this_thread::yield();

        //Load new sensor data
        if (loadCAN) {
            //std::cout << "Loading CAN...\n";
            result = dwSensorCAN_readMessage(&g_newCANFrame, 100000, g_canSensor);
            if (result == DW_SUCCESS) {
                loadCAN = false;
            }
            else if (result == DW_END_OF_STREAM) {
                std::cout << "CAN reached end of stream." << std::endl;
                g_resetSensors = true;
            } else {
                std::cerr << "Terminating. Cannot read CAN frame: " << dwGetStatusName(result) << std::endl;
                g_run = false;
            }
        }

        if (loadImages && !g_pause) {
            bool success = true;
            for (int k = 0; k < CAMERA_COUNT; k++) {
                //Release previous GL frame
                if (g_currentDrawFrames[k]) {
                    g_sensorIO->releaseGlRgbaFrame(k);
                    g_sensorIO->releaseFrame(k);
                }

                //Start new frame
                result = g_sensorIO->getFrame(k);
                if (result == DW_SUCCESS) {
                    g_newFrames[k] = g_sensorIO->getCudaYuv(k);
                    g_currentDrawFrames[k] = g_sensorIO->getGlRgbaFrame(k);
                } else {
                    success = false;

                    if (result == DW_END_OF_STREAM) {
                        std::cout << "Camera reached end of stream." << std::endl;
                        g_resetSensors = true;
                    } else if (result != DW_SUCCESS) {
                        std::cerr << "Cannot read image: " << dwGetStatusName(result) << std::endl;
                    }
                }
            }

            if (success) {
                loadImages = false;
            }
        }

        //Reset after end of stream
        if (g_resetSensors) {
            g_pause = true;
            processCAN = false;
            processImages = false;
            loadCAN = false;
            loadImages = false;
            g_resetSensors = false;
            continue;
        }

        //Check which sensor has newest data
        if (!g_pause) {
            if (!g_newFrames[0])
                processCAN = true;
            else if (g_newCANFrame.timestamp_us < g_sensorIO->getTimestamp())
                processCAN = true;
            else
                processImages = true;
        }

        //Check time
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(myclock_t::now() - g_lastUpdateTime);
        std::chrono::milliseconds timeBetweenFrames = std::chrono::milliseconds::max();
        if (g_newFrames[0]) {
            timeBetweenFrames = std::chrono::milliseconds(((int64_t)g_sensorIO->getTimestamp() - (int64_t)g_lastImageTimestamp)/1000);
        }
        if(timeBetweenFrames.count() < 0) {
            std::cerr << "Error: New frame is older than previous frame.\n";
            g_run = false;
        }
        if (g_lastImageTimestamp > 0 && timeSinceUpdate < timeBetweenFrames)
            processImages = false;

        //Process sensors
        if (processCAN && !loadCAN) {
            dwCANInterpreter_consume(&g_newCANFrame, g_canInterpreter);

            uint32_t numSignals = 0;
            if (dwCANInterpreter_getNumberSignals(&numSignals, g_canInterpreter) == DW_SUCCESS)
            {
                for (uint32_t i=0; i < numSignals; i++) {
                    const char* name = nullptr;
                    float32_t value         = 0;
                    dwTime_t valueTimestamp = 0;

                    dwCANInterpreter_getSignalName(&name, i, g_canInterpreter);
                    dwCANInterpreter_getf32(&value, &valueTimestamp, i, g_canInterpreter);

                    if (0 == strcmp(name, g_dbcSpeed.c_str()))
                    {
                        dwEgomotion_addOdometry(DW_EGOMOTION_MEASURMENT_VELOCITY,
                                                      value,
                                                      valueTimestamp,
                                                      g_egomotion);
                    }else if (0 == strcmp(name, g_dbcSteering.c_str())) {
                        // note: in this sample CAN data contains steering angle directly,
                        // hence the parsed value can be used directly
                        dwEgomotion_addOdometry(DW_EGOMOTION_MEASURMENT_STEERINGANGLE,
                                                      value,
                                                      valueTimestamp,
                                                      g_egomotion);
                    }
                }
            }

            loadCAN = true;
        }

        if (processImages && !loadImages) {
            frameIdx++;

            if(g_playSingleFrame)
                g_pause = true;

            //Update times
            g_lastUpdateTime = myclock_t::now();
            g_lastImageTimestamp = g_sensorIO->getTimestamp();

            {
                ProfileCUDASection s(g_profiler.get(), "ProcessFrame");

                dwEgomotion_update(g_sensorIO->getTimestamp(), g_egomotion);

                // update current absolute estimate of the pose using relative motion between now and last time
                {
                    dwTransformation rigLast2rigNow;
                    if (DW_SUCCESS == dwEgomotion_computeRelativeTransformation(&rigLast2rigNow, lastEgoUpdate, g_sensorIO->getTimestamp(), g_egomotion))
                    {
                        // compute absolute pose given the relative motion between two last estimates
                        dwTransformation rigNow2World;
                        dwEgomotion_applyRelativeTransformation(&rigNow2World, &rigLast2rigNow, &egoRig2World);
                        egoRig2World = rigNow2World;
                    }
                }
                lastEgoUpdate = g_sensorIO->getTimestamp();

                dwTransformation invPreviousEgoRig2World;
                Mat4_IsoInv(invPreviousEgoRig2World.array, previousEgoRig2World.array);
                
                dwTransformation egoLastRig2PredictedRig;
                Mat4_AxB(egoLastRig2PredictedRig.array, invPreviousEgoRig2World.array, egoRig2World.array);

                dwTransformation predictedRig2World;
                Mat4_AxB(predictedRig2World.array, g_currentRig2World.array, egoLastRig2PredictedRig.array);

                //Feature matching
                {
                    ProfileCUDASection s(g_profiler.get(), "Features2D");

                    for (int k = 0; k < CAMERA_COUNT; k++) {
                        //Track
                        trackFrame(k, g_newFrames[k], &g_currentRig2World, &predictedRig2World);
                        g_sensorIO->releaseCudaYuv(k);
                        g_newFrames[k] = nullptr;

                        if (g_doDetection)
                        {
                            ProfileCUDASection s(g_profiler.get(), "Detection");
                            dwFeatureTracker_detectNewFeatures(g_featureList[k], g_pyramidCurrent[k], g_tracker);
                        }
                    }
                }

                float32_t *d_trackedLocations[CAMERA_COUNT];
                for (int i = 0; i < CAMERA_COUNT; i++) {
                    uint32_t currentTimeIdx;
                    dwFeatureList_getCurrentTimeIdx(&currentTimeIdx, g_featureList[i]);

                    d_trackedLocations[i] = reinterpret_cast<float32_t*>(&(g_d_featureLocationHistory[i][currentTimeIdx * g_maxFeatureCount]));
                }

                //Refine
                dwTransformation currentCorrection;
                if (!g_doVisualPoseEstimation) {
                    g_poseColor = DW_RENDERER_COLOR_DARKGREEN;
                    currentCorrection = predictedRig2World;
                } else {
                    g_poseColor = DW_RENDERER_COLOR_DARKRED;

                    {
                        ProfileCUDASection s(g_profiler.get(), "copyPose2GPU");
                        cudaMemcpyAsync(g_d_rig2WorldInitial, &predictedRig2World.array, sizeof(dwTransformation), cudaMemcpyHostToDevice, g_cudaStream);
                        cudaMemcpyAsync(g_d_previousRig2World, &g_currentRig2World.array, sizeof(dwTransformation), cudaMemcpyHostToDevice, g_cudaStream);
                    }

                    {
                        ProfileCUDASection s(g_profiler.get(), "PoseEstimation");


                        dwReconstructor_estimatePose(g_d_rig2WorldCorrection,
                                                     g_d_previousRig2World,
                                                     g_d_rig2WorldInitial,
                                                     CAMERA_COUNT,
                                                     g_d_featureCount,
                                                     g_d_featureStatuses,
                                                     d_trackedLocations,
                                                     g_d_worldPoints,
                                                     g_reconstructor);
                    }
                    
                    //Blocking memcpy
                    {
                        ProfileCUDASection s(g_profiler.get(), "copyPose2CPU");
                        cudaMemcpyAsync(&currentCorrection, g_d_rig2WorldCorrection, sizeof(dwTransformation), cudaMemcpyDeviceToHost, g_cudaStream);
                        cudaStreamSynchronize(g_cudaStream);
                    }
                }
                g_currentRig2World = currentCorrection;
                Mat4_RenormR(g_currentRig2World.array);

                //Store pose in render buffer
                {
                    ProfileCUDASection s(g_profiler.get(), "RecordPose");

                    recordPose(&predictedRig2World, &g_currentRig2World);
                }
                previousEgoRig2World = egoRig2World;

                //Update feature history
                {
                    ProfileCUDASection s(g_profiler.get(), "History3D");
                    dwReconstructor_updateHistory(&g_currentPoseIdx, &g_currentRig2World, CAMERA_COUNT, g_d_featureCount, d_trackedLocations, g_reconstructor);
                }

                for (int k = 0; k < CAMERA_COUNT; k++) {
                    //Triangulate
                    ProfileCUDASection s(g_profiler.get(), "Triangulation");
                    dwReconstructor_triangulateFeatures(g_d_worldPoints[k], g_d_featureStatuses[k], g_d_featureCount[k], k, g_reconstructor);
                }


                //Project back onto camera for display
                {
                    ProfileCUDASection s(g_profiler.get(), "Reproject");
                    dwReconstructor_project(g_d_projectedLocations, &g_currentRig2World, g_d_featureCount, g_d_worldPoints, g_reconstructor);
                }

                {
                    ProfileCUDASection s(g_profiler.get(), "getFeaturesFromGPU");

                    for (int k = 0; k < CAMERA_COUNT; k++) {
                        //Get data to CPU
                        getFeaturesFromGPU(k);
                    }
                }

                {
                    ProfileCUDASection s(g_profiler.get(), "Compact");
                    for (int k = 0; k < CAMERA_COUNT; k++) {
                        //Compact
                        compactFeatures(k);
                    }
                }
            }
            g_profiler->collectTimers();

            loadImages = true;
        }

        //Draw (only when new image data is available)
        if (processImages || g_pause) {
            updateViewportSizes();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if(g_viewportMaxIdx == 0)
                dwRenderer_setRect(g_viewportRects[0], g_renderer);
            else
                dwRenderer_setRect(g_viewportRects[g_viewportMaxIdx], g_renderer);

            draw3D();

            //Individual cameras
            for (uint32_t k = 0; k < CAMERA_COUNT; k++) {
                if(g_viewportMaxIdx == k+1)
                    dwRenderer_setRect(g_viewportRects[0], g_renderer);
                else
                    dwRenderer_setRect(g_viewportRects[k+1], g_renderer);

                if (g_currentDrawFrames[k])
                    draw(k, g_currentDrawFrames[k]);
            }

            g_window->swapBuffers();
            CHECK_GL_ERROR();
        }
    }

    //Show timings
    //////////
    // Enable show totals to display timings per frame instead of per camera
    // g_profiler->setShowTotals(true);
    // g_profiler->setTotalsFactor(frameIdx);
    std::cout << "Timing results:\n" << *g_profiler << "\n";

    //Clean up
    for (int k = 0; k < CAMERA_COUNT; k++) {
        if (g_currentDrawFrames[k]) {
            g_sensorIO->releaseGlRgbaFrame(k);
            g_sensorIO->releaseFrame(k);
        }

        dwSensor_stop(g_cameraSensor[k]);
    }

    dwSensor_stop(g_canSensor);

    return 0;
}

//------------------------------------------------------------------------------
void trackFrame(int k, dwImageCUDA *image, const dwTransformation *previousRigToWorld, const dwTransformation *predictedRigToWorld)
{
    std::swap(g_pyramidCurrent[k], g_pyramidPrevious[k]);

    uint32_t currentTimeIdx;
    dwFeatureList_getCurrentTimeIdx(&currentTimeIdx, g_featureList[k]);

    const float32_t *featureLocations = reinterpret_cast<const float32_t*>(&(g_d_featureLocationHistory[k][currentTimeIdx * g_maxFeatureCount]));

    if (predictedRigToWorld) {
        //Predict
        ProfileCUDASection s(g_profiler.get(), "Predict");

        dwReconstructor_predictFeaturePosition(g_d_predictedFeatureLocations[k],
                                               k,
                                               previousRigToWorld,
                                               predictedRigToWorld,
                                               g_d_featureCount[k],
                                               g_d_featureStatuses[k],
                                               featureLocations,
                                               g_d_worldPoints[k],
                                               g_reconstructor);
    } else {
        cudaMemcpyAsync(g_d_predictedFeatureLocations[k], featureLocations, g_maxFeatureCount * sizeof(float2), cudaMemcpyDeviceToDevice, g_cudaStream);
    }

    //Build pyramid
    dwStatus result;
    {
        ProfileCUDASection s(g_profiler.get(), "Pyramid");

        dwImageCUDA planeY;
        dwImageCUDA_getPlaneAsImage(&planeY, image, 0);

        result = dwPyramid_build(&planeY, g_pyramidCurrent[k]);
    }
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot build pyrmaid: ") + dwGetStatusName(result));

    {
        ProfileCUDASection s(g_profiler.get(), "Tracking");
        dwFeatureTracker_trackFeatures(g_featureList[k], g_pyramidPrevious[k], g_pyramidCurrent[k],
                                       g_d_predictedFeatureLocations[k], g_tracker);
        dwFeatureList_proximityFilter(g_featureList[k]);
    }
}

//------------------------------------------------------------------------------
void compactFeatures(int k)
{
    //Determine which features to throw away
    {
        ProfileCUDASection s(g_profiler.get(), "SelectValid2D");
        dwFeatureList_selectValid(g_d_validFeatureCount, g_d_validFeatureIndexes,
                                  g_d_invalidFeatureCount, g_d_invalidFeatureIndexes,
                                  g_featureList[k]);
    }

    //Compact list
    {
        ProfileCUDASection s(g_profiler.get(), "CompactCall");
        dwFeatureList_compact(g_featureList[k],
                              g_d_validFeatureCount, g_d_validFeatureIndexes,
                              g_d_invalidFeatureCount, g_d_invalidFeatureIndexes);
        dwReconstructor_compactFeatureHistory(k,
                                              g_d_validFeatureCount, g_d_validFeatureIndexes,
                                              g_d_invalidFeatureCount, g_d_invalidFeatureIndexes, g_reconstructor);
        dwReconstructor_compactWorldPoints(g_d_worldPoints[k],
                                           g_d_validFeatureCount, g_d_validFeatureIndexes,
                                           g_d_invalidFeatureCount, g_d_invalidFeatureIndexes, g_reconstructor);
    }
}

//------------------------------------------------------------------------------
void getFeaturesFromGPU(int k)
{
    //Get feature info to CPU
    cudaMemcpyAsync(g_featureDataBuffer[k].get(), g_d_featureDataBase[k], g_d_featureSize[k], cudaMemcpyDeviceToHost, g_cudaStream);
    cudaMemcpyAsync(g_predictedFeatureLocations[k].get(), g_d_predictedFeatureLocations[k],
               g_maxFeatureCount * sizeof(float2), cudaMemcpyDeviceToHost, g_cudaStream);
    cudaMemcpyAsync(g_projectedLocations[k].get(), g_d_projectedLocations[k],
               g_maxFeatureCount * sizeof(float2), cudaMemcpyDeviceToHost, g_cudaStream);
    //Copy triangulated points
    cudaMemcpyAsync(g_worldPoints[k].get(), g_d_worldPoints[k], g_maxFeatureCount * sizeof(float4), cudaMemcpyDeviceToHost, g_cudaStream);
    cudaStreamSynchronize(g_cudaStream);
}

//------------------------------------------------------------------------------
void recordPose(const dwTransformation *poseCAN, const dwTransformation *poseRefined)
{
    if (g_poseRenderCount >= MAX_RENDER_POSES)
    {
        printColored(stderr, COLOR_YELLOW, "Warning: max pose count reached. Will not plot any more.");
        return;
    }

    auto positionCAN = make_float3(poseCAN->array[0 + 3 * 4], poseCAN->array[1 + 3 * 4], poseCAN->array[2 + 3 * 4]);
    auto positionRefined = make_float3(poseRefined->array[0 + 3 * 4], poseRefined->array[1 + 3 * 4], poseRefined->array[2 + 3 * 4]);

    float3 *map;
    uint32_t maxItems, stride;

    //Update CAN
    dwRenderBuffer_mapRange((float32_t **)&map, &maxItems, &stride, g_poseRenderCount, g_poseCANRenderBuffer);
    *map = positionCAN;
    dwRenderBuffer_unmap(1, g_poseCANRenderBuffer);

    //Update refined
    dwRenderBuffer_mapRange((float32_t **)&map, &maxItems, &stride, g_poseRenderCount, g_poseRefinedRenderBuffer);
    *map = positionRefined;
    dwRenderBuffer_unmap(1, g_poseRefinedRenderBuffer);

    //Update diff
    dwRenderBuffer_mapRange((float32_t **)&map, &maxItems, &stride, 2*g_poseRenderCount, g_poseCorrectionRenderBuffer);
    map[0] = positionCAN;
    map[1] = positionRefined;
    dwRenderBuffer_unmap(2, g_poseCorrectionRenderBuffer);

    g_poseRenderCount++;
}

//------------------------------------------------------------------------------
void updateViewportSizes()
{
    float32_t aspect = (float32_t)g_imageWidth / g_imageHeight;
    int32_t smallViewportHeight = g_window->height() / CAMERA_COUNT;
    int32_t columnWidth= (int32_t)(smallViewportHeight * aspect);

    g_viewportRects[0] = { columnWidth, 0, g_window->width() - columnWidth, g_window->height() };

    for (uint32_t i = 0; i < CAMERA_COUNT; i++)
        g_viewportRects[1+i] = { 0, static_cast<int32_t>((CAMERA_COUNT-i-1)) * smallViewportHeight, columnWidth, smallViewportHeight };

}

//------------------------------------------------------------------------------
void draw3D()
{
    //Set camera
    g_mouseView.setWindowAspect((float)g_window->width() / g_window->height());
    float modelview[16];
    memcpy(modelview, g_mouseView.getModelView(), 16 * sizeof(float));

    float pi[16];
    Mat4_IsoInv(pi, g_currentRig2World.array);

    float t[16];
    Mat4_AxB(t, modelview, pi);

    dwRenderer_setModelView(t, g_renderer);
    dwRenderer_setProjection(g_mouseView.getProjection(), g_renderer);

    //Render ground plane
    dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREY, g_renderer);
    dwRenderer_setLineWidth(1.0f, g_renderer);
    dwRenderer_renderBuffer(g_groundPlane, g_renderer);

    //Difference between CAN and refinement
    dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREEN, g_renderer);
    dwRenderer_setLineWidth(1.0f, g_renderer);
    dwRenderer_renderBuffer(g_poseCorrectionRenderBuffer, g_renderer);

    //CAN pose
    dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREEN, g_renderer);
    dwRenderer_setPointSize(2.0f, g_renderer);
    dwRenderer_renderBuffer(g_poseCANRenderBuffer, g_renderer);

    //Refined pose
    dwRenderer_setColor(g_poseColor, g_renderer);
    dwRenderer_setLineWidth(2.0f, g_renderer);
    dwRenderer_renderBuffer(g_poseRefinedRenderBuffer, g_renderer);
    
    //////////////////////////
    ////Draw world points
    const float32_t *colors[CAMERA_COUNT] = {DW_RENDERER_COLOR_BLUE, DW_RENDERER_COLOR_RED, DW_RENDERER_COLOR_GREEN, DW_RENDERER_COLOR_YELLOW};

    dwRenderer_setPointSize(2.0f, g_renderer);

    for (int k = 0; k < CAMERA_COUNT; k++) {
        float3 *map;
        uint32_t maxItems, stride;
        uint32_t drawCount;

        dwRenderBuffer_map((float32_t **)&map, &maxItems, &stride, g_worldPointsRenderBuffer);
        if (stride != sizeof(*map) / sizeof(float32_t))
            throw std::runtime_error("Stride of buffer is unexpected");

        drawCount = 0;
        for (uint32_t i = 0; i < *g_featureData[k].featureCount; i++) {
            if (g_featureData[k].statuses[i] != DW_FEATURE_STATUS_TRACKED)
                continue;

            float4 p = g_worldPoints[k][i];
            if (p.w == .0f)
                continue;

            map[drawCount] = make_float3(p.x, p.y, p.z);
            drawCount++;
        }
        dwRenderBuffer_unmap(drawCount, g_worldPointsRenderBuffer);
        dwRenderer_setColor(colors[k], g_renderer);
        dwRenderer_renderBuffer(g_worldPointsRenderBuffer, g_renderer);
    }

}

//------------------------------------------------------------------------------
void draw(int k, dwImageGL *imageRGBA)
{
    /////////////////////
    // Draw texture
    dwRenderer_renderTexture(imageRGBA->tex, imageRGBA->target, g_renderer);
    CHECK_GL_ERROR();

    // Map vars
    float2 *map;
    uint32_t maxItems, stride;
    uint32_t drawCount;

    uint32_t currentTimeIdx;
    dwFeatureList_getCurrentTimeIdx(&currentTimeIdx, g_featureList[k]);
    float2 *locations = reinterpret_cast<float2*>(&g_featureData[k].locationHistory[currentTimeIdx * g_maxFeatureCount]);

    //////////////////////////
    ////Draw tracked features
    dwRenderBuffer_map((float32_t **)&map, &maxItems, &stride, g_featuresRenderBuffer);
    if (stride != sizeof(*map) / sizeof(float32_t))
        throw std::runtime_error("Stride of buffer is unexpected");

    drawCount = 0;
    for (uint32_t i = 0; i < *g_featureData[k].featureCount; i++) {
        if (g_featureData[k].statuses[i] != DW_FEATURE_STATUS_TRACKED)
            continue;

        map[drawCount] = locations[i];
        drawCount++;
    }

    dwRenderBuffer_unmap(drawCount, g_featuresRenderBuffer);

    dwRenderer_setPointSize(2.0f, g_renderer);
    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, g_renderer);
    dwRenderer_renderBuffer(g_featuresRenderBuffer, g_renderer);

    CHECK_GL_ERROR();

    //////////////////////////
    ////Draw predicted features
    dwRenderBuffer_map((float32_t **)&map, &maxItems, &stride, g_featurePredictionsRenderBuffer);
    if (stride != sizeof(*map) / sizeof(float32_t))
        throw std::runtime_error("Stride of buffer is unexpected");

    drawCount = 0;
    for (uint32_t i = 0; i < *g_featureData[k].featureCount; i++) {
        if (g_featureData[k].statuses[i] != DW_FEATURE_STATUS_TRACKED)
            continue;

        map[drawCount] = locations[i];
        drawCount++;
        map[drawCount] = g_predictedFeatureLocations[k][i];
        drawCount++;
    }

    dwRenderBuffer_unmap(drawCount, g_featurePredictionsRenderBuffer);

    dwRenderer_setColor(DW_RENDERER_COLOR_YELLOW, g_renderer);
    dwRenderer_renderBuffer(g_featurePredictionsRenderBuffer, g_renderer);

    CHECK_GL_ERROR();

    //////////////////////////
    ////Draw reprojected features
    dwRenderBuffer_map((float32_t **)&map, &maxItems, &stride, g_featuresRenderBuffer);
    if (stride != sizeof(*map) / sizeof(float32_t))
        throw std::runtime_error("Stride of buffer is unexpected");

    drawCount = 0;
    for (uint32_t i = 0; i < *g_featureData[k].featureCount; i++) {
        if (g_featureData[k].statuses[i] != DW_FEATURE_STATUS_TRACKED)
            continue;

        if (g_worldPoints[k][i].w == 0)
            continue;

        map[drawCount] = g_projectedLocations[k][i];
        drawCount++;
    }

    dwRenderBuffer_unmap(drawCount, g_featuresRenderBuffer);

    dwRenderer_setPointSize(4.0f, g_renderer);
    dwRenderer_setColor(DW_RENDERER_COLOR_DARKRED, g_renderer);
    dwRenderer_renderBuffer(g_featuresRenderBuffer, g_renderer);

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
