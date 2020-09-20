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

#include <framework/DriveWorksSample.hpp>

#include <memory>
#include <thread>
#include <string>

// Context, SAL
#include <dw/core/Context.h>
#include <dw/sensors/Sensors.h>

// IMAGE
#include <dw/image/ImageStreamer.h>

// Renderer
#include <dw/renderer/Renderer.h>


#include <image_common/utils.hpp>

/**
 * Class that holds functions anda variables common to all stereo samples
 */
using namespace dw_samples::common;

class ImageStreamerSimpleApp : public DriveWorksSample
{
public:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t               context             = DW_NULL_HANDLE;
    dwSALHandle_t                   sal                 = DW_NULL_HANDLE;
    dwRendererHandle_t              renderer            = DW_NULL_HANDLE;

    const uint32_t WINDOW_HEIGHT = 800;
    const uint32_t WINDOW_WIDTH = 1280;

    // streamer from CPU to CUDA
    dwImageStreamerHandle_t m_streamerCPU2CUDA = DW_NULL_HANDLE;

    // streamer from CUDA to GL, used for display
    dwImageStreamerHandle_t m_streamerCUDA2GL = DW_NULL_HANDLE;

    // CPU image with RGBA format
    dwImageCPU m_rgbaCPU{};

    uint32_t m_frameCount;

public:
    ImageStreamerSimpleApp(const ProgramArguments& args);

    // Sample framework
    bool onInitialize() override final;
    void onProcess() override final;
    void onRelease() override final;
};

//#######################################################################################
ImageStreamerSimpleApp::ImageStreamerSimpleApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
    std::cout << "This sample illustrates how to use an image streamer given a CPU image. This will create an "
                 "empty dwImageCPU, stream it to a dwImageCUDA, apply some simple operations in a kernel and "
                 "then stream it to a dwImageGL for rendering. The purpose is to show how to properly "
                 "create, use and destroy an image streamer." << std::endl;
}

//#######################################################################################
bool ImageStreamerSimpleApp::onInitialize()
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

    m_frameCount = 0;

    // we create a dwImageCPU on the CPU by allocating memory
    uint32_t imageWidth = 800;
    uint32_t imageHeight = 600;
    dwStatus status;

    // the image is going to be format DW_IMAGE_RGBA and type DW_TYPE_UINT_8
    // we create a interleaved RGBA image
    uint32_t planeCount = 1;
    dwImageProperties cpuProp{DW_IMAGE_CPU, imageWidth, imageHeight, DW_IMAGE_RGBA, DW_TYPE_UINT8, planeCount};

    // rgbCPU->data points to the allocated planes
    status = dwImageCPU_create(&m_rgbaCPU, &cpuProp);
    if (status != DW_SUCCESS) {
        logError("Cannot create m_rgbaCPU: %s\n", dwGetStatusName(status));
        return false;
    }

    // we now create an ImageStreamer to stream the CPU image to CUDA
    // the streamer is setup to stream an image with properties cpuProp to a dwImageCUDA with the
    // same properties, except for the type, which is going to be DW_IMAGE_CUDA
    // the streamer needs to be released at the end
    status = dwImageStreamer_initialize(&m_streamerCPU2CUDA, &cpuProp, DW_IMAGE_CUDA, context);
    if (status != DW_SUCCESS) {
        logError("Cannot init image streamer m_streamerCPU2CUDA: %s\n", dwGetStatusName(status));
        return false;
    }

    // create a new imageStreamer to stream CUDA to GL and get a openGL texture to render on screen
    // properties are the same as cpu image except for the type
    dwImageProperties cudaProp = cpuProp;
    cudaProp.type = DW_IMAGE_CUDA;

    status = dwImageStreamer_initialize(&m_streamerCUDA2GL, &cudaProp, DW_IMAGE_GL, context);
    if (status != DW_SUCCESS) {
        logError("Cannot init gl image streamer m_streamerCUDA2GL: %s\n", dwGetStatusName(status));
        return false;
    }

    // -----------------------------
    // Initialize Renderer
    // -----------------------------
    {
        dwRenderer_initialize(&renderer, context);
        dwRect rect;
        rect.width  = getWindowWidth();
        rect.height = getWindowHeight();
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, renderer);
    }

    return true;
}

//#######################################################################################
void ImageStreamerSimpleApp::onProcess()
{
    dwStatus status;

    // post the cpu image. This will push the the image through the stream for type conversion.
    status = dwImageStreamer_postCPU(&m_rgbaCPU, m_streamerCPU2CUDA);
    if (status != DW_SUCCESS) {
        std::stringstream err;
        err << "Cannot post m_rgbaCPU in m_streamerCPU2CUDA: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // use a pointer to a dwImageCUDA to receive the converted image from the streamer. We receive
    // a pointer as the image is only "borrowed" from the streamer who has the ownership. Since the image is
    // borrowed we need to return it when we are done using it
    dwImageCUDA *rgbaCuda;
    // receive the converted CUDA image from the stream, timeout set to 1000 ms
    status = dwImageStreamer_receiveCUDA(&rgbaCuda, 1000, m_streamerCPU2CUDA);
    if (status != DW_SUCCESS) {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // now the converted image can be as a cuda image, for example we run the cuda kernel that is called
    // by this function (implemented in utils.cu)
    runKernel(rgbaCuda, m_frameCount);

    // the borrowed cuda image is now posted on the gl streamer. Until we return the gl image we cannot return
    // the cuda image
    status = dwImageStreamer_postCUDA(rgbaCuda, m_streamerCUDA2GL);

    if (status != DW_SUCCESS) {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // pointer to the dwImageGL we get from the streamer, it contains the texture and target we need for
    // rendering
    dwImageGL *glImage;

    // receive a dwImageGL that we can render
    status = dwImageStreamer_receiveGL(&glImage, 1000, m_streamerCUDA2GL);
    if (status != DW_SUCCESS) {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // render
    {
        glClearColor(0.0, 0.0, 1.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT);
        dwRenderer_renderTexture(glImage->tex, glImage->target, renderer);
    }

    // return the received gl since we don't use it anymore
    status = dwImageStreamer_returnReceivedGL(glImage, m_streamerCUDA2GL);
    if (status != DW_SUCCESS) {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // wait to get back the cuda image we posted in the cuda->gl stream. We will receive a pointer to it and,
    // to be sure we are getting back the same image we posted, we compare the pointer to our dwImageCPU
    dwImageCUDA *rgbaCudaBack;
    status = dwImageStreamer_waitPostedCUDA(&rgbaCudaBack, 1000, m_streamerCUDA2GL);
    if ((status != DW_SUCCESS) || (rgbaCudaBack != rgbaCuda)) {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // now that we are done with gl, we can return the dwImageCUDA to the streamer, which is the owner of it
    status = dwImageStreamer_returnReceivedCUDA(rgbaCuda, m_streamerCPU2CUDA);
    if (status != DW_SUCCESS) {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    // wait to get back the cpu image we posted at the beginning. We will receive a pointer to it and,
    // to be sure we are getting back the same image we posted, we compare the pointer to our dwImageCPU
    dwImageCPU *m_rgbaCPUBack;
    status = dwImageStreamer_waitPostedCPU(&m_rgbaCPUBack, 1000, m_streamerCPU2CUDA);
    if ((status != DW_SUCCESS) || (m_rgbaCPUBack != &m_rgbaCPU)) {
        std::stringstream err;
        err << "Cannot return gl image to m_streamerCUDA2GL: " << dwGetStatusName(status);
        throw std::runtime_error(err.str().c_str());
    }

    m_frameCount++;
}

//#######################################################################################
void ImageStreamerSimpleApp::onRelease()
{
    dwStatus status;

    // destroy the CPU image we created
    if (m_rgbaCPU.data[0] != nullptr) {
        status = dwImageCPU_destroy(&m_rgbaCPU);
        if (status != DW_SUCCESS) {
            logError("Cannot destroy m_rgbaCPU: %s\n", dwGetStatusName(status));
        }
    }

    // release streamers
    status = dwImageStreamer_release(&m_streamerCUDA2GL);
    if (status != DW_SUCCESS) {
        logError("Cannot release m_streamerCUDA2GL: %s\n", dwGetStatusName(status));
    }

    status = dwImageStreamer_release(&m_streamerCPU2CUDA);
    if (status != DW_SUCCESS) {
        logError("Cannot release m_streamerCPU2CUDA: %s\n", dwGetStatusName(status));
    }

    dwRenderer_release(&renderer);

    // -----------------------------------
    // Release SDK
    // -----------------------------------
    dwSAL_release(&sal);
    dwRelease(&context);
}

//#######################################################################################
int main(int argc, const char **argv)
{
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
    {
        // the sample supports offscreen rendering
        ProgramArguments::Option_t("offscreen", "0"),
    },
    "Sample illustrating how to use an image streamer given a CPU image");

    // Window/GL based application
    ImageStreamerSimpleApp app(args);
    app.initializeWindow("Image Streamer Simple", 1280, 800, args.enabled("offscreen"));
    return app.run();
}
