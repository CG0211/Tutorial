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

//SDK
#include <dw/calibration/camera/CameraParams.h>
#include <dw/lanes/LaneDetector.h>
#include <dw/sensors/imu/IMU.h>

//Sample
#include "CameraCalibrationSample.hpp"
#include <framework/MathUtils.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Mat4.hpp>

namespace dw_samples
{
namespace calibration
{

// -----------------------------------------------------------------------------
CameraCalibrationSample::CameraCalibrationSample(const ProgramArguments &args)
    : common::DriveWorksSample(args)
    , m_context(nullptr)
    , m_sal(nullptr)
    , m_rigConfiguration(nullptr)
    , m_egoMotion(nullptr)
    , m_calibrationEngine(nullptr)
    , m_sensorIndex(std::numeric_limits<decltype(m_sensorIndex)>::max())
    , m_calibratedCamera(nullptr)
    , m_pendingCuda(nullptr)
    , m_canBus(nullptr)
    , m_canInterpreter(nullptr)
    , m_imuSensor(nullptr)
    , m_laneNet(nullptr)
    , m_renderer(nullptr)
    , m_nominalPitchRollRenderBuffer(nullptr)
    , m_calculatedPitchRollRenderBuffer(nullptr)
    , m_nominalYawRenderBuffer(nullptr)
    , m_calculatedYawRenderBuffer(nullptr)
{
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::onRelease()
{
    if (m_calculatedYawRenderBuffer) {
        CHECK_DW_ERROR(dwRenderBuffer_release(&m_calculatedYawRenderBuffer));
    }

    if (m_nominalYawRenderBuffer) {
        CHECK_DW_ERROR(dwRenderBuffer_release(&m_nominalYawRenderBuffer));
    }

    if (m_calculatedPitchRollRenderBuffer) {
        CHECK_DW_ERROR(dwRenderBuffer_release(&m_calculatedPitchRollRenderBuffer));
    }

    if (m_nominalPitchRollRenderBuffer) {
        CHECK_DW_ERROR(dwRenderBuffer_release(&m_nominalPitchRollRenderBuffer);)
    }

    if (m_renderer) {
        CHECK_DW_ERROR(dwRenderer_release(&m_renderer));
    }

    if (m_laneNet) {
        CHECK_DW_ERROR(dwLaneDetector_release(&m_laneNet));
    }

    m_streamerGL.reset();

    m_player.reset();

    if (m_imuSensor) {
        CHECK_DW_ERROR(dwSensor_stop(m_imuSensor));
        CHECK_DW_ERROR(dwSAL_releaseSensor(&m_imuSensor));
    }

    if (m_canBus) {
        CHECK_DW_ERROR(dwSensor_stop(m_canBus));
        CHECK_DW_ERROR(dwSAL_releaseSensor(&m_canBus));
    }

    m_camera.reset();

    if (m_calibratedCamera) {
        CHECK_DW_ERROR(dwCalibratedCamera_release(&m_calibratedCamera));
    }

    if (m_calibrationEngine) {
        CHECK_DW_ERROR(dwCalibrationEngine_stopCalibration(m_sensorIndex, m_calibrationEngine))
        CHECK_DW_ERROR(dwCalibrationEngine_release(&m_calibrationEngine));
    }

    if (m_egoMotion) {
        CHECK_DW_ERROR(dwEgomotion_release(&m_egoMotion));
    }

    if (m_rigConfiguration) {
        CHECK_DW_ERROR(dwRigConfiguration_release(&m_rigConfiguration));
    }

    if (m_sal) {
        CHECK_DW_ERROR(dwSAL_release(&m_sal));
    }

    if (m_context) {
        CHECK_DW_ERROR(dwRelease(&m_context));
    }
}

// -----------------------------------------------------------------------------
bool CameraCalibrationSample::onInitialize()
{
    initializeSDK();

    initializeModules();

    initializeCameraSensor();

    initializeCAN();

    initializeIMU();

    initializePlayer();

    initializeLaneNet();

    startCalibration();

    initializeRenderer();

    return true;
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::onReset()
{
    // restart on reset
    m_player->restart();

    CHECK_DW_ERROR(dwEgomotion_reset(m_egoMotion));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializeSDK()
{

    // initialize logger to print verbose message on console in color
    CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true)))
    CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE))

    // initialize SDK context, using data folder
    dwContextParameters sdkParams = {};
    sdkParams.dataPath            = DataPath::get_cstr();

#ifdef VIBRANTE
    sdkParams.eglDisplay = getEGLDisplay();
#endif

    CHECK_DW_ERROR(dwInitialize(&m_context, DW_VERSION, &sdkParams))

    CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializeModules()
{
    //initialize our rig configuration module
    CHECK_DW_ERROR(dwRigConfiguration_initializeFromFile(&m_rigConfiguration, m_context,
                                                         (getArgument("path") + getArgument("rig")).c_str()))

    uint32_t imuIndex{};
    CHECK_DW_ERROR(dwRigConfiguration_findSensorByName(&imuIndex, "imu", m_rigConfiguration));

    dwTransformation imutoRig{};

    CHECK_DW_ERROR(dwRigConfiguration_getSensorToRigTransformation(&imutoRig, imuIndex, m_rigConfiguration));

    dwEgoMotionParameters params{};

    params.imu2rig = imutoRig;

    m_vehicle = nullptr;
    CHECK_DW_ERROR(dwRigConfiguration_getVehicle(&m_vehicle, m_rigConfiguration))

    params.wheelBase        = m_vehicle->wheelbase;
    params.motionModel      = DW_EGOMOTION_IMU_ODOMETRY;
    params.historySize      = 1000.0f;
    params.estimationPeriod = 10000;
    params.gyroscopeBias[0] = 0.0f;
    params.gyroscopeBias[1] = 0.0f;
    params.gyroscopeBias[2] = 0.0f;

    //initialize our ego motion module
    CHECK_DW_ERROR(dwEgomotion_initialize(&m_egoMotion, &params, m_context))

    //finally initialize our calibration module
    CHECK_DW_ERROR(dwCalibrationEngine_initialize(&m_calibrationEngine, m_rigConfiguration, m_context))
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializeCameraSensor()
{
    m_sensorIndex = std::stoi(getArgument("sensorIndex"));

    std::string protocol;
    std::string parameters;

    protocol   = "camera.virtual";
    parameters = std::string("video=") + getArgument("path") + getArgument("video") +
                 ",timestamp=" + getArgument("path") + getArgument("video-time");

    //initialize our camera sensor
    dwSensorParams cameraSensorParams{};
    cameraSensorParams.protocol   = protocol.c_str();
    cameraSensorParams.parameters = parameters.c_str();

    auto outputProperties = dwImageProperties{};
    outputProperties.pxlType    = DW_TYPE_UINT8;
    outputProperties.pxlFormat  = DW_IMAGE_RGBA;
    outputProperties.planeCount = 1;
    outputProperties.type       = DW_IMAGE_CUDA;

    m_camera.reset(new dw_samples::common::SimpleCamera(outputProperties, cameraSensorParams, m_sal, m_context));

    outputProperties = m_camera->getOutputProperties();

    // Streamer
    m_streamerGL.reset(new dw_samples::common::SimpleImageStreamer<dwImageCUDA, dwImageGL>(
            outputProperties, 10000, m_context));

    dwCameraModel cameraModel{};
    CHECK_DW_ERROR(dwRigConfiguration_getCameraModel(&cameraModel, m_sensorIndex, m_rigConfiguration));

    switch (cameraModel)
    {
        case DW_CAMERA_MODEL_PINHOLE: {
            dwPinholeCameraConfig config{};
            CHECK_DW_ERROR(dwRigConfiguration_getPinholeCameraConfig(&config, m_sensorIndex, m_rigConfiguration));
            CHECK_DW_ERROR(dwCalibratedCamera_initializePinhole(&m_calibratedCamera, m_context, &config));
            m_cameraDimensions.x = config.width;
            m_cameraDimensions.y = config.height;
        }
        break;
        case DW_CAMERA_MODEL_OCAM: {
            dwOCamCameraConfig config{};
            CHECK_DW_ERROR(dwRigConfiguration_getOCamCameraConfig(&config, m_sensorIndex, m_rigConfiguration));
            CHECK_DW_ERROR(dwCalibratedCamera_initializeOCam(&m_calibratedCamera, m_context, &config);)
            m_cameraDimensions.x = config.width;
            m_cameraDimensions.y = config.height;
        }
        break;
        case DW_CAMERA_MODEL_FTHETA: {
            dwFThetaCameraConfig config{};
            CHECK_DW_ERROR(dwRigConfiguration_getFThetaCameraConfig(&config, m_sensorIndex, m_rigConfiguration));
            CHECK_DW_ERROR(dwCalibratedCamera_initializeFTheta(&m_calibratedCamera, m_context, &config);)
            m_cameraDimensions.x = config.width;
            m_cameraDimensions.y = config.height;
        }
        break;
        default:
            throw std::runtime_error("Unknown type of camera was provided by rig configuration");
    }

    m_scaledCameraDimensions.x  = static_cast<float32_t>(getWindowWidth()) / m_cameraDimensions.x;
    m_scaledCameraDimensions.y = static_cast<float32_t>(getWindowHeight()) / m_cameraDimensions.y;
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializeCAN()
{
    std::string protocol;
    std::string params;

    protocol = "can.virtual";
    params   = std::string("file=") + getArgument("path") + getArgument("can");

    //initialize CAN
    dwSensorParams sensorParams{};
    sensorParams.protocol   = protocol.c_str();
    sensorParams.parameters = params.c_str();

    CHECK_DW_ERROR(dwSAL_createSensor(&m_canBus, sensorParams, m_sal));

    //initialize our CAN interpreter
    m_dbcSpeed = getArgument("dbc-speed");

    m_dbcSteering = getArgument("dbc-steering");

    CHECK_DW_ERROR(dwCANInterpreter_buildFromDBC(&m_canInterpreter,
                                                 (getArgument("path") + getArgument("dbc")).c_str(), m_context));

    CHECK_DW_ERROR(dwSensor_start(m_canBus));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializeIMU()
{
    std::string protocol;
    std::string params;

    protocol = "imu.virtual";
    params   = std::string("file=") + getArgument("path") + getArgument("imu");

    dwSensorParams sensorParams{};
    sensorParams.protocol   = protocol.c_str();
    sensorParams.parameters = params.c_str();

    CHECK_DW_ERROR(dwSAL_createSensor(&m_imuSensor, sensorParams, m_sal));

    CHECK_DW_ERROR(dwSensor_start(m_imuSensor));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializePlayer()
{
    m_player.reset(new dw_samples::common::SimpleRecordingPlayer(this));

    if(m_canBus)
        m_player->setCAN(m_canBus);

    if(m_imuSensor)
        m_player->setIMU(m_imuSensor);

    m_player->addCamera(m_camera.get());
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializeLaneNet()
{
    CHECK_DW_ERROR(dwLaneDetector_initializeLaneNet(&m_laneNet, m_cameraDimensions.x,
                                                    m_cameraDimensions.y, m_context));

    CHECK_DW_ERROR(
        dwLaneDetectorLaneNet_setDetectionThreshold(std::stof(getArgument("lanenet-threshold")),
                                                    m_laneNet));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::startCalibration()
{

    dwCalibrationCameraParams cameraParams{};
    CHECK_DW_ERROR(dwCalibrationEngine_initializeCamera(m_sensorIndex, &cameraParams,
                                                        m_egoMotion, m_calibrationEngine));

    CHECK_DW_ERROR(dwRigConfiguration_getSensorToRigTransformation(&m_nominalSensor2Rig,
                                                                   m_sensorIndex, m_rigConfiguration))

    Mat4_IsoInv(m_nominalRig2Sensor.array, m_nominalSensor2Rig.array);

    CHECK_DW_ERROR(dwCalibrationEngine_startCalibration(m_sensorIndex, m_calibrationEngine))
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::onProcess()
{
    m_pendingCuda = nullptr;
    while(!m_pendingCuda)
        m_player->stepForward();

    CHECK_DW_ERROR(dwLaneDetector_processDeviceAsync(m_pendingCuda,m_laneNet));

    CHECK_DW_ERROR(dwLaneDetector_interpretHost(m_laneNet));

    dwLaneDetection laneDetections;

    CHECK_DW_ERROR(dwLaneDetector_getLaneDetections(&laneDetections, m_laneNet));

    // Feed detection to calibration
    CHECK_DW_ERROR(dwCalibrationEngine_addLaneDetections(&laneDetections, m_pendingCuda->timestamp_us,
                                                         m_sensorIndex, m_calibrationEngine));

    CHECK_DW_ERROR(dwCalibrationEngine_getSensorToRigTransformation(&m_calculatedSensor2Rig,
                                                                    m_sensorIndex, m_calibrationEngine))

    // Factorization: sensor2Rig = [correctionT] * sensor2RigNominal * [correctionR]
    Mat4_AxB(m_correctionR.array, m_nominalRig2Sensor.array, m_calculatedSensor2Rig.array);
    m_correctionR.array[3 * 4] = m_correctionR.array[3 * 4 + 1] = m_correctionR.array[3 * 4 + 2] = 0.f;

    m_correctionT.x = m_calculatedSensor2Rig.array[3 * 4] - m_nominalSensor2Rig.array[3 * 4];
    m_correctionT.y = m_calculatedSensor2Rig.array[3 * 4 + 1] - m_nominalSensor2Rig.array[3 * 4 + 1];
    m_correctionT.z = m_calculatedSensor2Rig.array[3 * 4 + 2] - m_nominalSensor2Rig.array[3 * 4 + 2];

    Mat4_IsoInv(m_calculatedRig2Sensor.array, m_calculatedSensor2Rig.array);

    CHECK_DW_ERROR(dwCalibrationEngine_getCalibrationStatus(&m_status, m_sensorIndex, m_calibrationEngine))
    CHECK_DW_ERROR(dwCalibrationEngine_getPercentageComplete(&m_percentageComplete, m_sensorIndex, m_calibrationEngine))
}


// -----------------------------------------------------------------------------
void CameraCalibrationSample::handleCAN(const dwCANMessage & canMsg)
{
    CHECK_DW_ERROR(dwCANInterpreter_consume(&canMsg, m_canInterpreter))

    uint32_t numSignals = 0;
    CHECK_DW_ERROR(dwCANInterpreter_getNumberSignals(&numSignals, m_canInterpreter))

    dwTime_t maxTimeStamp = 0;

    for (auto i = 0u; i < numSignals; i++) {

        const char *name        = nullptr;
        float value             = 0.f;
        dwTime_t valueTimeStamp = 0;

        CHECK_DW_ERROR(dwCANInterpreter_getSignalName(&name, i, m_canInterpreter))

        CHECK_DW_ERROR(dwCANInterpreter_getf32(&value, &valueTimeStamp, i, m_canInterpreter))

        if (m_dbcSpeed == name) {
            // this is an assumption that needs to hold for the DBC file
            auto const valueMS = value / 3.6f; // input value is in kph
            CHECK_DW_ERROR(dwEgomotion_addOdometry(DW_EGOMOTION_MEASURMENT_VELOCITY, valueMS,
                                                   valueTimeStamp, m_egoMotion))

            maxTimeStamp = std::max(maxTimeStamp, valueTimeStamp);
        } else if (m_dbcSteering == name) {
            // this is an assumption that needs to hold for the DBC file
            auto const valueRad = DEG2RAD(value / m_vehicle->steeringCoefficient); // input value is in scaled degrees
            CHECK_DW_ERROR(dwEgomotion_addOdometry(DW_EGOMOTION_MEASURMENT_STEERINGANGLE, valueRad,
                                                   valueTimeStamp, m_egoMotion))

            maxTimeStamp = std::max(maxTimeStamp, valueTimeStamp);
        }
    }
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::handleCamera(uint32_t /*idx*/, common::dwImageGeneric *frame)
{
    m_pendingCuda = dw_samples::common::GenericImage::toDW<dwImageCUDA>(frame);
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::handleIMU(const dwIMUFrame & imuFrame)
{
    CHECK_DW_ERROR(dwEgomotion_addIMUMeasurement(&imuFrame, m_egoMotion));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::handleEndOfStream()
{
    onReset();
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::initializeRenderer()
{
    CHECK_DW_ERROR(dwRenderer_initialize(&m_renderer, m_context))

    dwRect rect;
    rect.width  = getWindowWidth();
    rect.height = getWindowHeight();
    rect.x      = 0;
    rect.y      = 0;

    CHECK_DW_ERROR(dwRenderer_setRect(rect, m_renderer));

    CHECK_DW_ERROR(dwRenderer_setLineWidth(2.0f, m_renderer));

    auto layout = dwRenderBufferVertexLayout{DW_RENDER_FORMAT_R32G32_FLOAT, DW_RENDER_SEMANTIC_POS_XY,
                                             DW_RENDER_FORMAT_NULL, DW_RENDER_SEMANTIC_COL_NULL,
                                             DW_RENDER_FORMAT_NULL, DW_RENDER_SEMANTIC_TEX_NULL};

    auto pitchSize = NUM_PITCHROLL_POINTS ;
    auto yawSize   = NUM_YAW_POINTS;

    std::pair<dwRenderBufferHandle_t *, uint32_t> renderBuffers[] = {{&m_nominalPitchRollRenderBuffer, pitchSize},
                                                                     {&m_calculatedPitchRollRenderBuffer, pitchSize},
                                                                     {&m_nominalYawRenderBuffer, yawSize},
                                                                     {&m_calculatedYawRenderBuffer, yawSize}};

    for (auto i = 0u; i < std::extent<decltype(renderBuffers)>::value; ++i) {
        auto renderBuffer = renderBuffers[i];

        CHECK_DW_ERROR(dwRenderBuffer_initialize(&(*renderBuffer.first), layout, DW_RENDER_PRIM_LINESTRIP,
                                                 renderBuffer.second, m_context));

        CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)m_window->width(),
                                                                     (float32_t)m_window->height(),
                                                                     *renderBuffer.first));
    }

    CHECK_DW_ERROR(dwCalibratedCamera_pixel2Ray(&m_cameraOpticalCenterDir.x,
                                                &m_cameraOpticalCenterDir.y,
                                                &m_cameraOpticalCenterDir.z,
                                                m_calibratedCamera,
                                                m_cameraDimensions.x / 2.f,
                                                m_cameraDimensions.y / 2.f));

    calculatePitchRollPoints(m_nominalPitchRollRenderBuffer, m_nominalRig2Sensor);
    calculateYawPoints(m_nominalYawRenderBuffer, m_nominalRig2Sensor);

    glDepthFunc(GL_ALWAYS);
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::onRender()
{
    common::DriveWorksSample::onRender();

    glClear(GL_COLOR_BUFFER_BIT);

    if (m_pendingCuda) {
        auto const frameGL = m_streamerGL->post(m_pendingCuda);
        CHECK_DW_ERROR(dwRenderer_renderTexture(frameGL->tex, frameGL->target, m_renderer));
        m_streamerGL->release();
    }

    CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer));
    calculatePitchRollPoints(m_calculatedPitchRollRenderBuffer, m_calculatedRig2Sensor);
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_calculatedPitchRollRenderBuffer, m_renderer));
    calculateYawPoints(m_calculatedYawRenderBuffer, m_calculatedRig2Sensor);
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_calculatedYawRenderBuffer, m_renderer));

    CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_BLUE, m_renderer));
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_nominalPitchRollRenderBuffer, m_renderer));
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_nominalYawRenderBuffer, m_renderer));

    CHECK_DW_ERROR(dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer));

    //now render our matrices and text to the screen
    uint32_t currentHeight = BORDEROFFSET;

    if (m_pendingCuda)
    {
        std::stringstream ss;
        ss << "Timestamp: " << m_pendingCuda->timestamp_us << "\n";
        CHECK_DW_ERROR(dwRenderer_renderText(BORDEROFFSET, currentHeight, ss.str().c_str(), m_renderer));

        currentHeight += 1.2f*LINESPACING;
    }

    renderRotationAsText(currentHeight, m_correctionR);

    CHECK_DW_ERROR(dwRenderer_renderText(BORDEROFFSET, currentHeight, "Camera rotation correction (camera coordinates)", m_renderer));
    currentHeight += LINESPACING;

    renderFloatsText(BORDEROFFSET, currentHeight, &m_correctionT.x, 3);
    currentHeight += LINESPACING;

    CHECK_DW_ERROR(dwRenderer_renderText(BORDEROFFSET, currentHeight, "Camera position correction (rig coordinates)", m_renderer));
    currentHeight += LINESPACING;

    CHECK_DW_ERROR(dwRenderer_renderText(BORDEROFFSET, currentHeight, convertStatusToText().c_str(), m_renderer));
    currentHeight += LINESPACING;

    CHECK_DW_ERROR(dwRenderer_renderText(BORDEROFFSET, currentHeight, "Rendering: Estimated (Green) + Nominal (Blue)", m_renderer));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::onResizeWindow(int width, int height)
{
    dwRect rect;
    rect.width  = width;
    rect.height = height;
    rect.x      = 0;
    rect.y      = 0;

    CHECK_DW_ERROR(dwRenderer_setRect(rect, m_renderer));

    dwRenderBufferHandle_t *renderBuffers[] = {&m_nominalPitchRollRenderBuffer,
                                               &m_calculatedPitchRollRenderBuffer,
                                               &m_nominalYawRenderBuffer,
                                               &m_calculatedYawRenderBuffer};

    for (auto i = 0u; i < std::extent<decltype(renderBuffers)>::value; ++i) {
        auto renderBuffer = renderBuffers[i];

        CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)width,
                                                                     (float32_t)height,
                                                                     *renderBuffer));
    }

    m_scaledCameraDimensions.x = static_cast<float32_t>(width) / m_cameraDimensions.x;

    m_scaledCameraDimensions.y = static_cast<float32_t>(height) / m_cameraDimensions.y;

    calculatePitchRollPoints(m_nominalPitchRollRenderBuffer, m_nominalRig2Sensor);

    calculateYawPoints(m_nominalYawRenderBuffer, m_nominalRig2Sensor);
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::renderRotationAsText(uint32_t &currentHeight,
                                                   const dwTransformation &transformation)
{
    std::stringstream ss;

    ss.precision(4);
    ss.setf(std::ios::fixed, std::ios::floatfield);

    for (auto row = 0u;  row < 3; ++row) {
        for(auto col = 0u; col < 3; ++col)
            ss << transformation.array[row+col*4] << " ";
        ss << "\n";
    }

    currentHeight += LINESPACING;

    dwRenderer_renderText(BORDEROFFSET, currentHeight, ss.str().c_str(), m_renderer);

    currentHeight += LINESPACING;
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::renderFloatsText(const int32_t x, const int32_t y, const float *floats, uint32_t numFloats)
{
    std::stringstream ss;

    ss.precision(4);
    ss.setf(std::ios::fixed, std::ios::floatfield);

    for (uint32_t i = 0; i < numFloats; ++i) {
        ss << floats[i] << " ";
    }

    std::string text = ss.str();

    dwRenderer_renderText(x, y, text.c_str(), m_renderer);
}

// -----------------------------------------------------------------------------
std::string CameraCalibrationSample::convertStatusToText()
{
    std::stringstream ss;

    switch (m_status) {
    case dwCalibrationStatus::DW_CALIBRATION_STATUS_STARTED:
        ss << "Status: Started";
        break;
    case dwCalibrationStatus::DW_CALIBRATION_STATUS_ACCEPTED:
        ss << "Status: Accepted";
        break;
    case dwCalibrationStatus::DW_CALIBRATION_STATUS_FAILED:
        ss << "Status: Failed";
        break;
    case dwCalibrationStatus::DW_CALIBRATION_STATUS_STOPPED:
        ss << "Status: Stopped";
        break;
    default:
        ss << "Status: Unknown";
        break;
    };

    ss.precision(2);
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss << " (" << 100.f * m_percentageComplete << "%)";

    return ss.str();
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::calculatePitchRollPoints(dwRenderBufferHandle_t renderBuffer,
                                                       const dwTransformation &rig2Sensor)
{
    // Sample a "half circle" of directions in front of car in the xy ground plane

    std::vector<dwVector2f> imagePoints;

    float32_t angleIncrement = M_PI / (NUM_PITCHROLL_POINTS - 1);
    float32_t angle          = 0.0f;

    for (auto i = 0u; i < NUM_PITCHROLL_POINTS; ++i, angle += angleIncrement) {
        float32_t rigDir[3];

        rigDir[0] = sin(angle);
        rigDir[1] = cos(angle);
        rigDir[2] = 0;

        float32_t cameraDir[3];

        Mat4_Rxp(cameraDir, rig2Sensor.array, rigDir);

        // discards points "behind" camera
        if (cameraDir[0] * m_cameraOpticalCenterDir.x +
            cameraDir[1] * m_cameraOpticalCenterDir.y +
            cameraDir[2] * m_cameraOpticalCenterDir.z >= 0.1f) {

            float32_t u{0.0f}, v{0.0f};

            CHECK_DW_ERROR(dwCalibratedCamera_ray2Pixel(&u, &v,
                                                        m_calibratedCamera,
                                                        cameraDir[0],
                                                        cameraDir[1],
                                                        cameraDir[2]));

            u *= m_scaledCameraDimensions.x;
            v *= m_scaledCameraDimensions.y;

            imagePoints.push_back({u, v});
        }
    }

    float32_t *coords     = nullptr;
    uint32_t maxVertices  = 0;
    uint32_t vertexStride = 0;
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer));

    for (uint32_t i = 0; i < imagePoints.size() && i < maxVertices; ++i) {
        coords[0] = imagePoints[i].x;
        coords[1] = imagePoints[i].y;
        coords += vertexStride;
    }

    CHECK_DW_ERROR(dwRenderBuffer_unmap(imagePoints.size(), renderBuffer));
}

// -----------------------------------------------------------------------------
void CameraCalibrationSample::calculateYawPoints(dwRenderBufferHandle_t renderBuffer,
                                                 const dwTransformation &rig2Sensor)
{
    // Sample points in front of car in the xy ground plane

    std::vector<dwVector2f> imagePoints;

    float32_t offset = 0.f;

    for (auto i = 0u; i < NUM_YAW_POINTS; ++i, offset += offset + 1) {
        float32_t rigPoint[3];

        rigPoint[0] = offset;
        rigPoint[1] = 0.f;
        rigPoint[2] = 0.f;

        float32_t cameraPoint[3];

        Mat4_Axp(cameraPoint, rig2Sensor.array, rigPoint);

        // discards points "behind" camera
        if (cameraPoint[0] * m_cameraOpticalCenterDir.x +
            cameraPoint[1] * m_cameraOpticalCenterDir.y +
            cameraPoint[2] * m_cameraOpticalCenterDir.z >= 0.1f) {
            float32_t u{0.0f}, v{0.0f};

            CHECK_DW_ERROR(dwCalibratedCamera_ray2Pixel(&u, &v,
                                                        m_calibratedCamera,
                                                        cameraPoint[0],
                                                        cameraPoint[1],
                                                        cameraPoint[2]));

            u *= m_scaledCameraDimensions.x;
            v *= m_scaledCameraDimensions.y;

            imagePoints.push_back({u, v});
        }
    }

    float32_t *coords     = nullptr;
    uint32_t maxVertices  = 0;
    uint32_t vertexStride = 0;
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer));

    for (uint32_t i = 0; i < imagePoints.size() && i < maxVertices; ++i) {
        coords[0] = imagePoints[i].x;
        coords[1] = imagePoints[i].y;
        coords += vertexStride;
    }

    CHECK_DW_ERROR(dwRenderBuffer_unmap(imagePoints.size(), renderBuffer));
}

} //namespace calibration
} //namespace dw_samples
