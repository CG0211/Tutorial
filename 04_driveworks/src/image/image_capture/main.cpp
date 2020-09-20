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
// status from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_WARNINGS

#include <memory>
#include <thread>
#include <string>

// IMAGE
#include <dw/image/FrameCapture.h>

#include <image_common/utils.hpp>

#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <dw/renderer/Renderer.h>

/**
 * Class that holds functions anda variables common to all stereo samples
 */
using namespace dw_samples::common;

class ImageStreamerCaptureApp : public DriveWorksSample
{
public:
    static const uint32_t WINDOW_HEIGHT = 800;
    static const uint32_t WINDOW_WIDTH = 1280;

    ImageStreamerCaptureApp(const ProgramArguments& args) : DriveWorksSample(args) {}

    bool onInitialize() override final;
    void onProcess() override final;
    void onRelease() override final;

    std::unique_ptr<SimpleImageStreamer<dwImageCUDA, dwImageGL>> m_streamer;

    // CUDA image with RGBA format
    dwImageCUDA m_rgbaCUDA{};
private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t               m_context             = DW_NULL_HANDLE;
    dwSALHandle_t                   m_sal                 = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwRendererHandle_t              m_renderer            = DW_NULL_HANDLE;

    dwFrameCaptureHandle_t          m_frameCapture        = DW_NULL_HANDLE;

    bool m_screeCap;
    uint32_t m_frameCount;
};

//#######################################################################################
void ImageStreamerCaptureApp::onRelease()
{
    // destroy the CPU image we created
    if (m_rgbaCUDA.dptr[0] != nullptr) {
        dwStatus status = dwImageCUDA_destroy(&m_rgbaCUDA);
        if (status != DW_SUCCESS) {
            logError("Cannot destroy m_rgbaCUDA: %s\n", dwGetStatusName(status));
        }
    }

    dwFrameCapture_release(&m_frameCapture);

    dwRenderer_release(&m_renderer);
    dwSAL_release(&m_sal);
    dwRelease(&m_context);
}

//#######################################################################################
bool ImageStreamerCaptureApp::onInitialize()
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

        dwInitialize(&m_context, DW_VERSION, &sdkParams);
        dwSAL_initialize(&m_sal, m_context);
    }


    // -----------------------------
    // Initialize Renderer
    // -----------------------------
    {
        CHECK_DW_ERROR_MSG(dwRenderer_initialize(&m_renderer, m_context),
                           "Cannot initialize Renderer, maybe no GL context available?");
        dwRect rect;
        rect.width  = getWindowWidth();
        rect.height = getWindowHeight();
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, m_renderer);
    }

    uint32_t imageWidth = WINDOW_WIDTH;
    uint32_t imageHeight = WINDOW_HEIGHT;

    // set capture params
    dwFrameCaptureParams frameParams{};

    std::string params = "type=disk";
    params += ",format=h264";
    params += ",bitrate=" + getArgument("capture-bitrate");
    params += ",framerate=" + getArgument("capture-framerate");
    params += ",file=" + getArgument("capture-file");
    frameParams.params.parameters = params.c_str();
    frameParams.width = imageWidth;
    frameParams.height = imageHeight;

    m_screeCap = getArgument("capture-screen") == "1";

    if ( m_screeCap ) {
        frameParams.type = DW_IMAGE_GL;
    } else {
        frameParams.type = DW_IMAGE_CUDA;
    }

    CHECK_DW_ERROR(dwFrameCapture_initialize(&m_frameCapture, &frameParams, m_sal, m_context));

    m_frameCount = 0;

    // the image is going to be format DW_IMAGE_RGBA and type DW_TYPE_UINT_8
    // we create a interleaved RGBA image
    uint32_t planeCount = 1;
    dwImageProperties cudaProp{DW_IMAGE_CUDA, imageWidth, imageHeight, DW_IMAGE_RGBA, DW_TYPE_UINT8, planeCount};

    // create a synthetic cuda image
    dwStatus status = dwImageCUDA_create(&m_rgbaCUDA, &cudaProp, DW_IMAGE_CUDA_PITCH);
    if (status != DW_SUCCESS) {
        logError("Cannot create m_rgbaCPU: %s\n", dwGetStatusName(status));
        return false;
    }

    // streamer used for display
    m_streamer.reset(new SimpleImageStreamer<dwImageCUDA, dwImageGL>(cudaProp, 10000, m_context));

    return true;
}

//#######################################################################################
void ImageStreamerCaptureApp::onProcess()
{
    // cuda kernel for generating a synthetic image
    runKernel(&m_rgbaCUDA, m_frameCount);


    dwImageGL *glImage = m_streamer->post(&m_rgbaCUDA);;

    dwRenderer_renderTexture(glImage->tex, glImage->target, m_renderer);

    // render
    dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, m_renderer);
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_64, m_renderer);
    dwRenderer_renderText(m_frameCount%WINDOW_WIDTH, m_frameCount%WINDOW_HEIGHT, "DriveWorks", m_renderer);

    // if screen capture was chosen, record what is displayed on the window. Otherwise serialize the cuda image directly
    if (m_screeCap) {
        // capture from the window the region defined in roi
        dwRect roiCapture = {0, 0, static_cast<int32_t>(WINDOW_WIDTH), static_cast<int32_t>(WINDOW_HEIGHT)};
        const dwImageGL* imgGL = nullptr;
        CHECK_DW_ERROR(dwFrameCapture_screenCapture(&imgGL, roiCapture, m_frameCapture));
        CHECK_DW_ERROR(dwFrameCapture_appendFrameGL(imgGL, m_frameCapture));
    } else {
        CHECK_DW_ERROR(dwFrameCapture_appendFrameCUDA(&m_rgbaCUDA, m_frameCapture));
    }

    m_frameCount++;
}

//#######################################################################################
int main(int argc, const char **argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
    {
       ProgramArguments::Option_t{"capture-bitrate", "10000000", "Capture bitrate."},
       ProgramArguments::Option_t{"capture-framerate", "30", "Capture framerate."},
       ProgramArguments::Option_t{"capture-file", "capture.h264", "Capture path."},
       ProgramArguments::Option_t{"capture-screen", "1", "Capture screen or serialize synthetic cuda image."}
    });


    // -------------------
    // initialize and start a window application
    ImageStreamerCaptureApp app(args);

    app.initializeWindow("Image capture", ImageStreamerCaptureApp::WINDOW_WIDTH, ImageStreamerCaptureApp::WINDOW_HEIGHT);
    app.setProcessRate(std::stoi(args.get("capture-framerate")));

    return app.run();
}
