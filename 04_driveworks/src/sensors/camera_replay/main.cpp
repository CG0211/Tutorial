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
// Copyright (c) 2016-2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
///////////////////////////////////////////////////////////////////////////////////////


#include <framework/DriveWorksSample.hpp>

#include <dw/renderer/Renderer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/FormatConverter.h>
#include <dw/image/Image.h>
#include <dw/image/ImageStreamer.h>

using namespace dw_samples::common;


//------------------------------------------------------------------------------
// Camera replay samples
// The Video replay sample demonstrates H.264 playback using a hardware decoder.
//
// The sample opens an X window to play back the provided video file. The playback
// does not support any container formats (MP4 or similar); a pure H.264 stream is
// required.
//------------------------------------------------------------------------------
class CameraReplay : public DriveWorksSample
{
private:

    static constexpr size_t IMAGE_BUFFER_SIZE = 4;

    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t               context             = DW_NULL_HANDLE;
    dwSALHandle_t                   sal                 = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwRendererHandle_t              renderer            = DW_NULL_HANDLE;
    dwSensorHandle_t                camera              = DW_NULL_HANDLE;
    dwImageStreamerHandle_t         image2GL            = DW_NULL_HANDLE;
    dwImageFormatConverterHandle_t  formatConverter     = DW_NULL_HANDLE;
    dwCameraProperties              cameraProps         = {};
    dwImageProperties               cameraImageProps    = {};
    dwCameraFrameHandle_t           frame               = DW_NULL_HANDLE;

    std::vector<dwImageCUDA>        cudaPool;
#ifdef VIBRANTE
    std::vector<dwImageNvMedia>     nvmediaPool;
#endif

public:

    CameraReplay(const ProgramArguments& args) : DriveWorksSample(args) {}


    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            dwLogger_initialize(getConsoleLoggerCallback(true));
            dwLogger_setLogLevel(DW_LOG_VERBOSE);

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};
            sdkParams.dataPath = DataPath::get_cstr();

            #ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
            #endif

            dwInitialize(&context, DW_VERSION, &sdkParams);
            dwSAL_initialize(&sal, context);
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            CHECK_DW_ERROR_MSG(dwRenderer_initialize(&renderer, context),
                               "Cannot initialize Renderer, maybe no GL context available?");
            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, renderer);
        }


        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            std::string file = "video=" + getArgument("video");

            dwSensorParams sensorParams;
            sensorParams.protocol   = "camera.virtual";
            sensorParams.parameters = file.c_str();
            CHECK_DW_ERROR_MSG(dwSAL_createSensor(&camera, sensorParams, sal),
                               "Cannot create virtual camera sensor, maybe wrong video file?");

            dwSensorCamera_getSensorProperties(&cameraProps, camera);
            dwSensorCamera_getImageProperties(&cameraImageProps, DW_CAMERA_PROCESSED_IMAGE, camera);

            printf("Camera image with %dx%d at %f FPS\n", cameraImageProps.width,
                   cameraImageProps.height, cameraProps.framerate);


            // we would like the application run as fast as the original video
            setProcessRate(cameraProps.framerate);
        }


        // -----------------------------
        // initialize streamer pipeline
        // -----------------------------
        {
            auto displayImageProps  = cameraImageProps;
            switch (cameraImageProps.type)
            {
            case DW_IMAGE_CUDA:
                displayImageProps.pxlFormat  = DW_IMAGE_RGBA;
                displayImageProps.planeCount = 1;
                CHECK_DW_ERROR(dwImageStreamer_initialize(&image2GL,
                                                          &displayImageProps,
                                                          DW_IMAGE_GL,
                                                          context));
                CHECK_DW_ERROR(dwImageFormatConverter_initialize(&formatConverter,
                                                                 DW_IMAGE_CUDA,
                                                                 context));

                cudaPool.resize(IMAGE_BUFFER_SIZE);
                for (size_t i = 0; i < cudaPool.size(); ++i) {
                    CHECK_DW_ERROR_MSG(dwImageCUDA_create(&cudaPool[i], &displayImageProps, DW_IMAGE_CUDA_PITCH),
                                       "Cannot allocate CUDA image, is the GPU CUDA compatible?");
                }

                break;

            #ifdef VIBRANTE
                case DW_IMAGE_NVMEDIA: {
                    displayImageProps.pxlFormat  = DW_IMAGE_RGBA;
                    displayImageProps.planeCount = 1;
                    CHECK_DW_ERROR(dwImageStreamer_initialize(&image2GL,
                                                              &displayImageProps,
                                                              DW_IMAGE_GL,
                                                              context));
                    CHECK_DW_ERROR(dwImageFormatConverter_initialize(&formatConverter,
                                                                     DW_IMAGE_NVMEDIA,
                                                                     context));

                    nvmediaPool.resize(IMAGE_BUFFER_SIZE);
                    for (size_t i = 0; i < nvmediaPool.size(); ++i) {
                        CHECK_DW_ERROR_MSG(dwImageNvMedia_create(&nvmediaPool[i], &displayImageProps, context),
                                           "Cannot create nvmedia image, potentially this is not a supported PDK");
                    }
                } break;
            #endif

            case DW_IMAGE_CPU:
            case DW_IMAGE_GL:
            default:
                break;
            }
        }

        // -----------------------------
        // Start Sensors
        // -----------------------------
        dwSensor_start(camera);

        return true;
    }


    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        dwSensor_reset(camera);
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        // stop sensor
        dwSensor_stop(camera);
        dwSAL_releaseSensor(&camera);

        // release renderer and streamer
        dwRenderer_release(&renderer);
        dwImageStreamer_release(&image2GL);
        dwImageFormatConverter_release(&formatConverter);

        // release buffers
    #ifdef VIBRANTE
        for (auto &each : nvmediaPool) {
            dwImageNvMedia_destroy(&each);
        }
    #else
        for (auto &each : cudaPool) {
            dwImageCUDA_destroy(&each);
        }
    #endif

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        {
            dwSAL_release(&sal);
            dwRelease(&context);
        }
    }


    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, renderer);
    }


    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - grab a frame from the camera
    ///     - convert frame to RGB
    ///     - push frame through the streamer to convert it into GL
    ///     - render frame on screen
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        glClearColor(0.0, 0.0, 1.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT);

        // ---------------------------
        // grab frame from camera
        // ---------------------------
        auto status = dwSensorCamera_readFrame(&frame, 0, 60000, camera);
        if (status == DW_END_OF_STREAM) {
            dwSensor_reset(camera);
            return;
        } else if (status == DW_TIME_OUT) {
            return;
        } else if (status != DW_SUCCESS) {
            CHECK_DW_ERROR(status);
        }

        // ---------------------------
        // convert frame to RGB and push to GL
        // ---------------------------
        dwImageGL *imageGL = nullptr;
        switch (cameraImageProps.type)
        {
        case DW_IMAGE_CUDA: {
            dwImageCUDA *imgCUDA = nullptr;
            dwSensorCamera_getImageCUDA(&imgCUDA, DW_CAMERA_PROCESSED_IMAGE,frame);

            auto rgbCUDA = cudaPool.back();
            cudaPool.pop_back();
            dwImageFormatConverter_copyConvertCUDA(&rgbCUDA, imgCUDA, formatConverter, 0);
            dwImageStreamer_postCUDA(&rgbCUDA, image2GL);
            dwImageStreamer_receiveGL(&imageGL, 30000, image2GL);

            dwRenderer_renderTexture(imageGL->tex, imageGL->target, renderer);

            dwImageStreamer_returnReceivedGL(imageGL, image2GL);
            dwImageStreamer_waitPostedCUDA(&imgCUDA, 33000, image2GL);
            cudaPool.push_back(*imgCUDA);

        } break;

    #ifdef VIBRANTE
        case DW_IMAGE_NVMEDIA: {
            dwImageNvMedia *imgNvMedia = nullptr;
            dwSensorCamera_getImageNvMedia(&imgNvMedia, DW_CAMERA_PROCESSED_IMAGE, frame);

            auto rgbNvMedia = nvmediaPool.back();
            nvmediaPool.pop_back();
            dwImageFormatConverter_copyConvertNvMedia(&rgbNvMedia, imgNvMedia, formatConverter);
            dwImageStreamer_postNvMedia(&rgbNvMedia, image2GL);
            dwImageStreamer_receiveGL(&imageGL, 30000, image2GL);

            dwRenderer_renderTexture(imageGL->tex, imageGL->target, renderer);
            dwImageStreamer_returnReceivedGL(imageGL, image2GL);

            dwImageStreamer_waitPostedNvMedia(&imgNvMedia, 33000, image2GL);
            nvmediaPool.push_back(*imgNvMedia);

        } break;
    #endif

        case DW_IMAGE_CPU:
        case DW_IMAGE_GL:
        default:
            break;
        }

        // we are done processing the frame, it is safe to return it to the camera now
        dwSensorCamera_returnFrame(&frame);
    }

};


//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
    {
       ProgramArguments::Option_t("video", (DataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
    },
    "Camera replay sample which playback .h264 video streams in a GL window.");


    // -------------------
    // initialize and start a window application
    CameraReplay app(args);

    app.initializeWindow("Camera replay", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
