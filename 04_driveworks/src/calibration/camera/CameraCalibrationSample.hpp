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

#ifndef SAMPLES_CALIBRATION_CAMERA_CAMERACALIBRATIONSAMPLE_HPP__
#define SAMPLES_CALIBRATION_CAMERA_CAMERACALIBRATIONSAMPLE_HPP__

//SDK
#include <dw/renderer/Renderer.h>
#include <dw/rigconfiguration/RigConfiguration.h>
#include <dw/egomotion/Egomotion.h>
#include <dw/calibration/CalibrationEngine.h>
#include <dw/renderer/Renderer.h>

//Sensor Input
#include <dw/sensors/camera/Camera.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>

//Rendering
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>

//Sample common files
#include <framework/DriveWorksSample.hpp>
#include <framework/Checks.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/SimpleRecordingPlayer.hpp>
#include <framework/SimpleStreamer.hpp>

#include <string>

namespace dw_samples
{
namespace calibration
{
class CameraCalibrationSample : public common::DriveWorksSample, public common::ISensorEventHandler
{
public:
    CameraCalibrationSample(const ProgramArguments &args);

    virtual ~CameraCalibrationSample() = default;

    virtual bool onInitialize() override;

    virtual void onRender() override;

    virtual void onProcess() override;

    virtual void onResizeWindow(int width, int height) override;

    virtual void onRelease() override;

    virtual void onReset() override;

private:
    static constexpr uint32_t LINESPACING          = 20;
    static constexpr uint32_t BORDEROFFSET         = 20;
    static constexpr uint32_t NUM_PITCHROLL_POINTS = 12;
    static constexpr uint32_t NUM_YAW_POINTS       = 12;

    //Initialization
    void initializeSDK();
    void initializeModules();
    void initializeCameraSensor();
    void initializeCAN();
    void initializeIMU();
    void initializePlayer();
    void initializeLaneNet();
    void initializeRenderer();
    void startCalibration();

    //Update
    virtual void handleCAN(const dwCANMessage & canMsg) override;
    virtual void handleIMU(const dwIMUFrame & imuFrame) override;
    virtual void handleCamera(uint32_t idx, common::dwImageGeneric *frame) override;
    virtual void handleEndOfStream() override;

    //Renderer
    void renderRotationAsText(uint32_t &currentHeight, const dwTransformation &transformation);
    void renderFloatsText(const int32_t x, const int32_t y, const float *floats, uint32_t numFloats);
    std::string convertStatusToText();

    void calculatePitchRollPoints(dwRenderBufferHandle_t renderBuffer, const dwTransformation &rig2Sensor);
    void calculateYawPoints(dwRenderBufferHandle_t renderBuffer, const dwTransformation &rig2Sensor);

    //Modules
    dwContextHandle_t m_context;
    dwSALHandle_t m_sal;
    dwRigConfigurationHandle_t m_rigConfiguration;
    dwEgomotionHandle_t m_egoMotion;
    dwCalibrationEngineHandle_t m_calibrationEngine;
    const dwVehicle * m_vehicle;

    // Calibration Info
    dwTransformation m_nominalSensor2Rig;
    dwTransformation m_nominalRig2Sensor;
    dwTransformation m_calculatedSensor2Rig;
    dwTransformation m_calculatedRig2Sensor;
    dwTransformation m_correctionR = {};
    dwVector3f m_correctionT       = {};

    uint32_t m_sensorIndex;
    dwCalibrationStatus m_status;
    float32_t m_percentageComplete = 0.f;
    dwCalibratedCameraHandle_t m_calibratedCamera;

    //Camera Sensor
    dwImageCUDA * m_pendingCuda;

    //CAN Sensor
    dwSensorHandle_t m_canBus;
    dwCANInterpreterHandle_t m_canInterpreter;
    std::string m_dbcSpeed;
    std::string m_dbcSteering;

    //IMU Sensor
    dwSensorHandle_t m_imuSensor;
    //LaneNet
    dwLaneDetectorHandle_t m_laneNet;

    //Rendering
    dwRendererHandle_t m_renderer;

    dwRenderBufferHandle_t m_nominalPitchRollRenderBuffer;
    dwRenderBufferHandle_t m_calculatedPitchRollRenderBuffer;

    dwRenderBufferHandle_t m_nominalYawRenderBuffer;
    dwRenderBufferHandle_t m_calculatedYawRenderBuffer;

    dwVector2f m_scaledCameraDimensions;
    dwVector2f m_cameraDimensions;

    dwVector3f m_cameraOpticalCenterDir;

    std::unique_ptr<dw_samples::common::SimpleCamera> m_camera;
    std::unique_ptr<dw_samples::common::SimpleRecordingPlayer> m_player;
    std::unique_ptr<dw_samples::common::SimpleImageStreamer<dwImageCUDA, dwImageGL>> m_streamerGL;

    CameraCalibrationSample(const CameraCalibrationSample &) = delete;
    CameraCalibrationSample &operator=(const CameraCalibrationSample &) = delete;
};

} // namespace calibration
} // namespace dw_samples

#endif // SAMPLES_CALIBRATION_CAMERA_CAMERACALIBRATIONSAMPLE_HPP__
