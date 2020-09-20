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

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <thread>

// SAMPLE framework
#include <framework/DriveWorksSample.hpp>
#include <framework/ProgramArguments.hpp>
// TODO: deprecated
#include <framework/Checks.hpp>
#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif

#include <framework/DataPath.hpp>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>

// Renderer
#include <dw/renderer/Renderer.h>

// IMAGE
#include <dw/image/ImageStreamer.h>
#include <dw/image/FormatConverter.h>

#define FRAME_SKIP 20
#define TIME_SKIP 100000


using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Sample application
//------------------------------------------------------------------------------

class CameraSeekSample : public DriveWorksSample
{
public:

    CameraSeekSample(const ProgramArguments &args)
        : DriveWorksSample(args)
        , m_sdk(DW_NULL_HANDLE)
        , m_renderer(DW_NULL_HANDLE)
        , m_sal(DW_NULL_HANDLE)
        , m_cameraSensor(DW_NULL_HANDLE)
        , m_streamer(DW_NULL_HANDLE)
        , m_converter(DW_NULL_HANDLE)
        , m_imageType(DW_IMAGE_CUDA)
        , m_frame(nullptr)
        , m_startTimestamp(0)
        , m_endTimestamp(0)
        , m_frameCount(0)
        , m_seekMode(SEEKING_MODE_FRAME_EVENT)
        , m_frameEvent(0)
        , m_prevFrameEvent(0)
        , m_timestamp(0)
        , m_prevTimestamp(0)
        , m_forceProcessFrame(false)
    {}

    ~CameraSeekSample()
    {}

    typedef enum {
        SEEKING_MODE_FRAME_EVENT = 0,
        SEEKING_MODE_TIMESTAMP = 1
    } SeekingMode;

private:

    dwContextHandle_t m_sdk;
    dwRendererHandle_t m_renderer;
    dwSALHandle_t m_sal;
    dwSensorHandle_t m_cameraSensor;
    dwImageStreamerHandle_t m_streamer;
    dwImageFormatConverterHandle_t m_converter;

    dwImageCUDA m_imageCUDA;

#ifdef VIBRANTE
    dwImageNvMedia m_imageNvMedia;
#endif

    dwImageType m_imageType;

    dwImageGL* m_frame;

    dwTime_t m_startTimestamp;
    dwTime_t m_endTimestamp;
    size_t m_frameCount;

    SeekingMode m_seekMode;
    size_t m_frameEvent;
    size_t m_prevFrameEvent;
    dwTime_t m_timestamp;
    dwTime_t m_prevTimestamp;

    bool m_forceProcessFrame;

public:

    bool onInitialize() override final
    {
        // Initialize DW context
        initSdk();

        // Initialize renderer
        initRenderer();

        // Initialize SAL and camera
        initCamera();

        return true;
    }

    void onRelease() override final
    {
        if(m_converter != DW_NULL_HANDLE) {
            dwImageFormatConverter_release(&m_converter);
        }

        if(m_streamer != DW_NULL_HANDLE) {
            dwImageStreamer_release(&m_streamer);
        }

        if(m_cameraSensor) {
            dwSensor_stop(m_cameraSensor);
            dwSAL_releaseSensor(&m_cameraSensor);
        }

        if(m_sal) {
            dwSAL_release(&m_sal);
        }

        if(m_renderer) {
            dwRenderer_release(&m_renderer);
        }

        if(m_sdk) {
            dwRelease(&m_sdk);
        }

        dwLogger_release();
    }

    void onResizeWindow(int width, int height) override final
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, m_renderer);
    }

    void onProcessKey(int key) override final
    {
        switch(key)
        {
        case GLFW_KEY_T:
            if(m_seekMode != SEEKING_MODE_TIMESTAMP)
            {
                m_seekMode = SEEKING_MODE_TIMESTAMP;
                std::cout << "Switch to timestamp seek" << std::endl;
            }
            break;

        case GLFW_KEY_F:
            if(m_seekMode != SEEKING_MODE_FRAME_EVENT)
            {
                m_seekMode = SEEKING_MODE_FRAME_EVENT;
                std::cout << "Switch to frame event seek" << std::endl;
            }
            break;

        case GLFW_KEY_LEFT:
            if(m_seekMode == SEEKING_MODE_FRAME_EVENT)
            {
                if (m_frameEvent > FRAME_SKIP)
                    m_frameEvent -= FRAME_SKIP;
                else
                    m_frameEvent = 0;

                std::cout << "Move to frame event: " << m_frameEvent << std::endl;
            }
            else
            {
                if (m_timestamp > TIME_SKIP + m_startTimestamp)
                    m_timestamp -= TIME_SKIP;
                else
                    m_timestamp = m_startTimestamp;

                std::cout << "Move to timestamp: " << m_timestamp << std::endl;
            }

            m_forceProcessFrame = m_pause;

            break;

        case GLFW_KEY_RIGHT:
            if (m_seekMode == SEEKING_MODE_FRAME_EVENT)
            {
                if(m_frameEvent + FRAME_SKIP < m_frameCount)
                    m_frameEvent += FRAME_SKIP;
                else
                    m_frameEvent = 0;

                std::cout << "Move to frame event: " << m_frameEvent << std::endl;
            }
            else
            {
                if(m_timestamp + TIME_SKIP < m_endTimestamp)
                    m_timestamp += TIME_SKIP;
                else
                    m_timestamp = m_startTimestamp;

                std::cout << "Move to timestamp: " << m_timestamp << std::endl;
            }

            m_forceProcessFrame = m_pause;

            break;

        default:
            break;
        }
    }

    void onProcess() override final
    {
        // Return from previous frame
        if (m_frame != nullptr)
        {
            dwImageStreamer_returnReceivedGL(m_frame, m_streamer);


#ifdef VIBRANTE
            if(m_imageType == DW_IMAGE_NVMEDIA)
            {
                dwImageNvMedia *imgNVMRet;
                dwImageStreamer_waitPostedNvMedia(&imgNVMRet, 3000, m_streamer);
            }
            else if(m_imageType == DW_IMAGE_CUDA)
#endif // VIBRANTE
            {
                dwImageCUDA *imgCUDARet;
                dwImageStreamer_waitPostedCUDA(&imgCUDARet, 3000, m_streamer);
            }
        }

        // Seek if requested
        if (m_timestamp != m_prevTimestamp)
        {
            std::cout << "Seeking to timestamp: " << m_timestamp << std::endl;
            dwStatus res = dwSensor_seekToTime(m_timestamp, m_cameraSensor);

            if(res != DW_SUCCESS) {
                std::cerr << "SeekToTime failed with " << dwGetStatusName(res) << std::endl;
            }
        }
        else if (m_frameEvent != m_prevFrameEvent)
        {
            std::cout << "Seeking to frame event: " << m_frameEvent << std::endl;
            dwStatus res = dwSensor_seekToEvent(m_frameEvent, m_cameraSensor);

            if(res != DW_SUCCESS) {
                std::cerr << "SeekToEvent failed with " << dwGetStatusName(res) << std::endl;
            }
        }

        dwCameraFrameHandle_t frameHandle = DW_NULL_HANDLE;
        dwStatus result = dwSensorCamera_readFrame(&frameHandle, 0, 3000, m_cameraSensor);
        if (result == DW_END_OF_STREAM)
        {
            m_frame = nullptr;
            std::cout << "Camera reached end of stream" << std::endl;

            dwSensor_reset(m_cameraSensor);
            m_frameEvent = 0;

            return;
        }

        if (result != DW_SUCCESS)
        {
            m_frame = nullptr;
            std::cerr << "Cannot read frame: " << dwGetStatusName(result) << std::endl;
            return;
        }

        switch(m_imageType)
        {
        case DW_IMAGE_CUDA:
            dwImageCUDA *imgCUDA;
            result = dwSensorCamera_getImageCUDA(&imgCUDA, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
            dwImageFormatConverter_copyConvertCUDA(&m_imageCUDA, imgCUDA, m_converter, 0);
            dwImageStreamer_postCUDA(&m_imageCUDA, m_streamer);
            break;

#ifdef VIBRANTE
        case DW_IMAGE_NVMEDIA:
            dwImageNvMedia *imgNVM;
            result = dwSensorCamera_getImageNvMedia(&imgNVM, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
            dwImageFormatConverter_copyConvertNvMedia(&m_imageNvMedia, imgNVM, m_converter);
            dwImageStreamer_postNvMedia(&m_imageNvMedia, m_streamer);
            break;
#endif

        default:
            break;
        }

        dwImageStreamer_receiveGL(&m_frame, 3000, m_streamer);

        if(m_frame != nullptr)
        {
            m_timestamp = m_frame->timestamp_us;
            m_prevTimestamp = m_timestamp;

            m_frameEvent++;
            m_prevFrameEvent = m_frameEvent;
        }

        dwSensorCamera_returnFrame(&frameHandle);
    }

    void onRender()
    {
        // Processing one frame if seek requested on pause
        if(m_forceProcessFrame) {
            onProcess();
            m_forceProcessFrame = false;
        }

        // Render the frame if available
        if(m_frame != nullptr) {
            CHECK_DW_ERROR(dwRenderer_renderTexture(m_frame->tex, m_frame->target, m_renderer));
        }
    }

private:

    void initSdk()
    {
        // create a Logger to log to console
        // we keep the ownership of the logger at the application level
        dwLogger_initialize(getConsoleLoggerCallback(true));
        dwLogger_setLogLevel(DW_LOG_VERBOSE);

        // initialize SDK context
        dwContextParameters sdkParams = {};

    #ifdef VIBRANTE
        sdkParams.eglDisplay = getEGLDisplay();
    #endif

        dwInitialize(&m_sdk, DW_VERSION, &sdkParams);
    }

    void initRenderer()
    {
        CHECK_DW_ERROR_MSG(dwRenderer_initialize(&m_renderer, m_sdk),
                           "Cannot initialize Renderer, make sure GL context is available");
        dwRect rect;
        rect.width  = getWindowWidth();
        rect.height = getWindowHeight();
        rect.x      = 0;
        rect.y      = 0;

        dwRenderer_setRect(rect, m_renderer);
    }

    void initCamera()
    {
        CHECK_DW_ERROR_MSG(dwSAL_initialize(&m_sal, m_sdk),
                           "Cannot initialize SAL.");

        // create GMSL Camera interface
        dwSensorParams params;
        std::string parameterString = m_args.parameterString();
        parameterString += ",create_seek=true";
        params.parameters           = parameterString.c_str();
        params.protocol             = "camera.virtual";
        CHECK_DW_ERROR_MSG(dwSAL_createSensor(&m_cameraSensor, params, m_sal),
                           "Cannot create camera sensor.");

        CHECK_DW_ERROR_MSG(dwSensor_getSeekRange(&m_frameCount, &m_startTimestamp, &m_endTimestamp, m_cameraSensor),
                           "Cannot obtain seek range from the camera.");

        dwCameraProperties cameraProperties;
        dwSensorCamera_getSensorProperties(&cameraProperties, m_cameraSensor);
        setProcessRate(cameraProperties.framerate);

        // Initialize streamer and format converter

        dwImageProperties imageProps;
        dwSensorCamera_getImageProperties(&imageProps,
                                          DW_CAMERA_PROCESSED_IMAGE,
                                          m_cameraSensor);

        dwImageProperties displayProps = imageProps;
        displayProps.pxlFormat = DW_IMAGE_RGBA;
        displayProps.planeCount = 1;

        m_imageType = displayProps.type;
        switch(m_imageType)
        {
        case DW_IMAGE_CUDA:

            CHECK_DW_ERROR(dwImageStreamer_initialize(&m_streamer,
                                                      &displayProps,
                                                      DW_IMAGE_GL,
                                                      m_sdk));

            CHECK_DW_ERROR(dwImageFormatConverter_initialize(&m_converter,
                                                             DW_IMAGE_CUDA,
                                                             m_sdk));

            CHECK_DW_ERROR(dwImageCUDA_create(&m_imageCUDA,
                                              &displayProps,
                                              DW_IMAGE_CUDA_PITCH));
            break;

    #ifdef VIBRANTE
        case DW_IMAGE_NVMEDIA:

            CHECK_DW_ERROR(dwImageStreamer_initialize(&m_streamer,
                                                      &displayProps,
                                                      DW_IMAGE_GL,
                                                      m_sdk));

            CHECK_DW_ERROR(dwImageFormatConverter_initialize(&m_converter,
                                                             DW_IMAGE_NVMEDIA,
                                                             m_sdk));

            CHECK_DW_ERROR(dwImageNvMedia_create(&m_imageNvMedia,
                                                 &displayProps,
                                                 m_sdk));
        break;
    #endif

        case DW_IMAGE_CPU:
        case DW_IMAGE_GL:
        default:
            break;
        };

        // Starting camera
        CHECK_DW_ERROR_MSG(dwSensor_start(m_cameraSensor),
                              "Cannot start camera sensor.");
    }
};



//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    ProgramArguments arguments(argc, argv,
        {
            ProgramArguments::Option_t("video", (DataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str())
        });

    CameraSeekSample app(arguments);
    app.initializeWindow("Camera seek sample", 1280, 800, arguments.enabled("offscreen"));

    return app.run();
}
