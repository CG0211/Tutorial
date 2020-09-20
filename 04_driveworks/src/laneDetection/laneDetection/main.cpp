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

#include <thread>

// Sample
#include <laneDetection/laneDetection_common/LaneDetectionCommon.hpp>

// Calibrated camera
#include <dw/rigconfiguration/RigConfiguration.h>
#include <dw/rigconfiguration/Camera.h>

class LaneNetCalibratedCamera : public LaneNet
{
public:
    LaneNetCalibratedCamera(uint32_t windowWidth,
                            uint32_t windowHeight,
                            const std::string &inputType)
        : LaneNet(windowWidth, windowHeight, inputType)
    {}
    virtual bool initializeModules() override
    {
        if (!initDriveworks()) return false;
        if (!initCameras()) return false;
        if (!initRenderer()) return false;
        if (!initRigConfiguration()) return false;
        if (!initPipeline()) return false;
        if (!initLaneNetWithRig()) return false;

        // start cameras as late as possible, so that all initialization routines are finished before
        gRun = gRun && dwSensor_start(m_cameraSensor) == DW_SUCCESS;

        return true;
    }

    virtual void releaseModules() override
    {
        releaseLaneNetCalibratedCamera();
        LaneNet::releaseModules();
    }

protected:
    bool initRigConfiguration()
    {
        dwStatus result = DW_SUCCESS;
        //Load vehicle configuration
        result = dwRigConfiguration_initializeFromFile(&m_rigConfig, m_sdk, gArguments.get("rig").c_str());
        if (result != DW_SUCCESS) {
            std::cerr << "Error dwRigConfiguration_initialize: " << dwGetStatusName(result) << std::endl;
            return false;
        }
        uint32_t cameraCount;
        result = dwCameraRig_initializeFromConfig(&m_cameraRig, &cameraCount,
                                                  &m_calibratedCam,
                                                  1U, m_sdk, m_rigConfig);
        if (result != DW_SUCCESS) {
            std::cerr << "Error dwCameraRig_initializeFromConfig: " << dwGetStatusName(result) << std::endl;
            return false;
        }
        return true;
    }

    //#######################################################################################
    bool initLaneNetWithRig()
    {
        dwStatus res = DW_FAILURE;
        dwTransformation transformation{};
        res = dwRigConfiguration_getSensorToRigTransformation(&transformation, 0, m_rigConfig);

        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot parse rig configuration: " << dwGetStatusName(res) << std::endl;
            return false;
        }

        float32_t maxDistance = 50.0f;
        std::string maxDistanceStr = gArguments.get("maxDistance");
        if(maxDistanceStr!="50.0") {
            try{
                maxDistance = std::stof(maxDistanceStr);
                if (maxDistance < 0.0f) {
                    std::cerr << "maxDistance cannot be negative." << std::endl;
                    return false;
                }
            } catch(...) {
                std::cerr << "Given maxDistance can't be parsed" << std::endl;
                return false;
            }
        }

        res = dwLaneDetector_initializeLaneNet(&m_laneDetector,
                                               m_cameraWidth, m_cameraHeight,
                                               m_sdk);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot initialize LaneNet: " << dwGetStatusName(res) << std::endl;
            return false;
        }

        res = dwLaneDetector_setCameraHandle(m_calibratedCam, m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot initialize lane detector calibrated camera: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        res = dwLaneDetector_setCameraExtrinsics(transformation, m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot initialize lane detector camera extrinsics: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        res = dwLaneDetector_setMaxLaneDistance(maxDistance, m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set lane detector maximum detection distance in meter: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        float32_t fov;
        res = dwCalibratedCamera_getHorizontalFOV(&fov, m_calibratedCam);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot get camera horizontal FOV: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        res =  dwLaneDetectorLaneNet_setHorizontalFOV(RAD2DEG(fov), m_laneDetector);
        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set camera horizontal FOV: "
                      << dwGetStatusName(res) << std::endl;
            return false;
        }

        m_threshold = 0.3f;
        std::string inputThreshold = gArguments.get("threshold");
        if(inputThreshold!="0.3"){
            try{
                m_threshold = std::stof(inputThreshold);
            } catch(...) {
                std::cerr << "Given threshold can't be parsed" << std::endl;
                return false;
            }
        }

        res = dwLaneDetectorLaneNet_setDetectionThreshold(m_threshold, m_laneDetector);

        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set LaneNet threshold: " << dwGetStatusName(res) << std::endl;
            return false;
        }

        //Default to 0.25, uncomment this block to customize lane temporal smoothing factor
        /*
        res = dwLaneDetectorLaneNet_setTemporalSmoothFactor(m_temporalSmoothFactor, m_laneDetector);

        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set LaneNet temporal smooth factor: " << dwGetStatusName(res) << std::endl;
            return false;
        }
        */


        //Default to full frame, uncomment this block to customize lane detection ROI
        /*
        res = dwLaneDetector_setDetectionROI(&m_roi, m_laneDetector);

        if (res != DW_SUCCESS)
        {
            std::cerr << "Cannot set LaneNet detection ROI: " << dwGetStatusName(res) << std::endl;
            return false;
        }
        */

        return true;
    }

    //#######################################################################################
    void releaseLaneNetCalibratedCamera()
    {
        dwCalibratedCamera_release(&m_calibratedCam);
        dwRigConfiguration_release(&m_rigConfig);
        dwCameraRig_release(&m_cameraRig);
    }

    dwRigConfigurationHandle_t m_rigConfig       = DW_NULL_HANDLE;
    dwCameraRigHandle_t m_cameraRig              = DW_NULL_HANDLE;
    dwCalibratedCameraHandle_t m_calibratedCam   = DW_NULL_HANDLE;
};


//#######################################################################################
int main(int argc, const char **argv)
{
    const ProgramArguments arguments = ProgramArguments({
#ifdef DW_USE_NVMEDIA
            ProgramArguments::Option_t("camera-type", "ar0231-rccb-ssc"),
            ProgramArguments::Option_t("csi-port", "ab"),
            ProgramArguments::Option_t("camera-index", "0"),
            ProgramArguments::Option_t("slave", "0"),
            ProgramArguments::Option_t("input-type", "video"),
#endif
            ProgramArguments::Option_t("video",
                                       (DataPath::get() +
                    std::string{"/samples/laneDetection/video_lane.h264"}).c_str()),
            ProgramArguments::Option_t("threshold", "0.3"),
            ProgramArguments::Option_t("width", "960"),
            ProgramArguments::Option_t("height", "576"),
            ProgramArguments::Option_t("maxDistance", "50.0"),
            ProgramArguments::Option_t("rig", (DataPath::get() + "/samples/laneDetection/rig.xml").c_str()),
        });

    // Default window width and height
    uint32_t windowWidth = 960;
    uint32_t windowHeight = 576;

    // init framework
    initSampleApp(argc, argv, &arguments, NULL, windowWidth, windowHeight);

    std::string inputType = "video";
#ifdef DW_USE_NVMEDIA
    inputType = gArguments.get("input-type");
#endif

    LaneNetCalibratedCamera laneNet(windowWidth, windowHeight, inputType);

    // init driveworks
    if (!laneNet.initializeModules())
    {
        std::cerr << "Cannot initialize DW subsystems" << std::endl;
        gRun = false;
    }

    typedef std::chrono::high_resolution_clock myclock_t;
    typedef std::chrono::time_point<myclock_t> timepoint_t;
    timepoint_t lastUpdateTime = myclock_t::now();

    // main loop
    while (gRun && !gWindow->shouldClose()) {
        std::this_thread::yield();

        bool processImage = true;

        // run with at most 30FPS
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(myclock_t::now() - lastUpdateTime);
        if (timeSinceUpdate < std::chrono::milliseconds(33)) //33
            processImage = false;

        if (processImage) {

            lastUpdateTime = myclock_t::now();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            laneNet.runSingleCameraPipeline();

            gWindow->swapBuffers();
        }
    }

    // Release modules and driveworks.
    laneNet.releaseModules();

    // release framework
    releaseSampleApp();

    return 0;
}
