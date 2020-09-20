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
#include <unordered_map>
#include <string>

// CORE
#include <dw/core/Context.h>
#include <dw/sensors/Sensors.h>

// Renderer
#include <dw/renderer/Renderer.h>

// IMAGE
#include <dw/image/ImageStreamer.h>

#include <framework/ProgramArguments.hpp>
#include <framework/SampleFramework.hpp>

#include <image_common/utils.hpp>

/**
 * Class that holds functions anda variables common to all stereo samples
 */
using namespace dw_samples::common;


class ImageStreamerMultiThreadApp : public DriveWorksSample
{
private:
    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t               context             = DW_NULL_HANDLE;
    dwSALHandle_t                   sal                 = DW_NULL_HANDLE;
    dwRendererHandle_t              renderer            = DW_NULL_HANDLE;

    const uint32_t WINDOW_HEIGHT = 800;
    const uint32_t WINDOW_WIDTH = 1280;

    std::thread m_producer;

    // streamer from CPU to CUDA
    dwImageStreamerHandle_t m_streamerCPU2CUDA = DW_NULL_HANDLE;

    // streamer from CUDA to GL, used for display
    dwImageStreamerHandle_t m_streamerCUDA2GL = DW_NULL_HANDLE;

    // CPU image with RGBA format
    dwImageCPU m_rgbaCPU{};

    // pointer to the dwImageGL we get from the streamer, it contains the texture and target we need for
    // rendering
    dwImageGL *m_glImage = nullptr;

    const dwTime_t m_timeout = 1000000;

    void producerThread(dwImageCPU *rgbaCPU,
                        dwImageStreamerHandle_t *streamerCPU2CUDA,
                        dwImageStreamerHandle_t *streamerCUDA2GL, dwTime_t timeout,
                        WindowBase* window);

public:

    ImageStreamerMultiThreadApp(const ProgramArguments& args);

    // Sample framework
    bool onInitialize() override final;
    void onProcess() override final;
    void onRelease() override final;
};

//#######################################################################################
ImageStreamerMultiThreadApp::ImageStreamerMultiThreadApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
    std::cout << "This sample illustrates how to use an image streamer given a CPU image. This will create an "
                 "empty dwImageCPU, stream it to a dwImageCUDA, apply some simple operations in a kernel and "
                 "then stream it to a dwImageGL for rendering. The purpose is to show how to properly "
                 "create, use and destroy an image streamer." << std::endl;
}

//#######################################################################################
bool ImageStreamerMultiThreadApp::onInitialize()
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

    dwStatus status;

    // we create a dwImageCPU on the CPU by allocating memory
    uint32_t imageWidth = 800;
    uint32_t imageHeight = 600;

    // the image is going to be format DW_IMAGE_RGBA and type DW_TYPE_UINT_8
    // we create a interleaved RGBA image
    uint32_t planeCount = 1;

    dwImageProperties cpuProp{DW_IMAGE_CPU, imageWidth, imageHeight, DW_IMAGE_RGBA,
                              DW_TYPE_UINT8,
                              planeCount};

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

    // start producer
    m_producer = std::thread(&ImageStreamerMultiThreadApp::producerThread, this,
                             &m_rgbaCPU, &m_streamerCPU2CUDA,
                             &m_streamerCUDA2GL, m_timeout, m_window.get());

    return true;
}

//#######################################################################################
void ImageStreamerMultiThreadApp::onRelease()
{
    dwStatus status;

    // if the glImage is still under use, return it
    if (m_glImage != nullptr) {
        // return the received gl since we don't use it anymore
        status = dwImageStreamer_returnReceivedGL(m_glImage, m_streamerCUDA2GL);
        if (status != DW_SUCCESS) {
            logError("Cannot return gl image streamer: %s\n", dwGetStatusName(status));
        }
    }

    if (m_producer.joinable()){
        // wait a little bit and stop producer
        std::this_thread::sleep_for(std::chrono::seconds(2));
        m_producer.join();
    }

    // destroy the CPU image we created
    status = dwImageCPU_destroy(&m_rgbaCPU);
    if (status != DW_SUCCESS) {
        logError("Cannot destroy m_rgbaCPUr: %s\n", dwGetStatusName(status));
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
void ImageStreamerMultiThreadApp::producerThread(dwImageCPU *rgbaCPU,
                    dwImageStreamerHandle_t *streamerCPU2CUDA,
                    dwImageStreamerHandle_t *streamerCUDA2GL,
                    dwTime_t timeout, WindowBase *window)
{
    log("Starting producer...\n");

    // Share window EGL context with this thread (required on vibrante)
    window->createSharedContext();

    bool running = true;
    while (running && shouldRun()) {
        log("Producer, posting...\n");

        dwStatus prodStatus = dwImageStreamer_postCPU(rgbaCPU, *streamerCPU2CUDA);
        if (prodStatus != DW_SUCCESS) {
            logError("Cannot post image streamer: %s\n", dwGetStatusName(prodStatus));
            running = false;
        }

        // use a pointer to a dwImageCUDA to receive the converted image from the streamer. We receive
        // a pointer as the image is only "borrowed" from the streamer who has the ownership. Since the image is
        // borrowed we need to return it when we are done using it
        dwImageCUDA *rgbaCuda;
        // receive the converted CUDA image from the stream
        prodStatus = dwImageStreamer_receiveCUDA(&rgbaCuda, timeout, *streamerCPU2CUDA);
        if (prodStatus != DW_SUCCESS) {
            logError("Cannot receive rgbaCuda from m_streamerCPU2CUDA: %s\n", dwGetStatusName(prodStatus));
            running = false;
        }

        // now the converted image can be as a cuda image, for example we run the cuda kernel that is called
        // by this function (implemented in utils.cu)
        runKernel(rgbaCuda, 0);

        // the borrowed cuda image is now posted on the gl streamer. Until we return the gl image we cannot return
        // the cuda image
        prodStatus = dwImageStreamer_postCUDA(rgbaCuda, *streamerCUDA2GL);
        if (prodStatus != DW_SUCCESS) {
            logError("Cannot post rgbaCuda in m_streamerCUDA2GL: %s\n", dwGetStatusName(prodStatus));
            running = false;
        }

        if (running) {
            log("Producer, posted, now waiting...\n");

            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 1000));

            // wait to get back the cuda image we posted in the cuda->gl stream. We will receive a pointer to it and,
            // to be sure we are getting back the same image we posted, we compare the pointer to our dwImageCPU
            dwImageCUDA *rgbaCudaBack;
            prodStatus = dwImageStreamer_waitPostedCUDA(&rgbaCudaBack, timeout, *streamerCUDA2GL);
            if ((prodStatus != DW_SUCCESS) || (rgbaCudaBack != rgbaCuda)) {
                logError("Waited for streamerCUDA2GL after posting, no reponse: %s\n", dwGetStatusName(prodStatus));
                //running = false;
            }

            // now that we are done with gl, we can return the dwImageCUDA to the streamer, which is the owner of it
            prodStatus = dwImageStreamer_returnReceivedCUDA(rgbaCuda, *streamerCPU2CUDA);
            if (prodStatus != DW_SUCCESS) {
                logError("Cannot return image to streamerCPU2CUDA: %s\n", dwGetStatusName(prodStatus));
                running = false;
            }

            // wait to get back the cpu image we posted at the beginning. We will receive a pointer to it and,
            // to be sure we are getting back the same image we posted, we compare the pointer to our dwImageCPU
            dwImageCPU *rgbaCPUBack;
            prodStatus = dwImageStreamer_waitPostedCPU(&rgbaCPUBack, timeout, *streamerCPU2CUDA);

            if ((prodStatus != DW_SUCCESS) || (rgbaCPUBack != rgbaCPU)) {
                logError("Cannot get back original image from streamerCPU2CUDA: %s\n", dwGetStatusName(prodStatus));
                running = false;
            }

            log("Producer, completed.\n");
        }
    }

    if (!running) stop();

    log("Stopping producer...\n");
}

//#######################################################################################
void ImageStreamerMultiThreadApp::onProcess()
{
    dwStatus status;

    // acting as the consumer, receives a GL image from the streamer of the producer, renders and returns
    log("Consumer, acquiring...\n");

    // receive a dwImageGL that we can render
    status = dwImageStreamer_receiveGL(&m_glImage, m_timeout, m_streamerCUDA2GL);
    if (status != DW_SUCCESS) {
        logError("Consumer: cannot receive m_glImage from m_streamerCUDA2GL: %s\n", dwGetStatusName(status));
        return;
    }
    else {
        // render
        glClearColor(0.0, 0.0, 1.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT);
        dwRenderer_renderTexture(m_glImage->tex, m_glImage->target, renderer);

        // return the received gl since we don't use it anymore
        status = dwImageStreamer_returnReceivedGL(m_glImage, m_streamerCUDA2GL);
        if (status != DW_SUCCESS) {
            logError("Consumer: cannot return gl image to m_streamerCUDA2GL: %s\n", dwGetStatusName(status));
        }
    }

    // when the image is returned, the pointer is set to nullptr, to be explicit that it is returned
    m_glImage = nullptr;
}

//#######################################################################################
int main(int argc, const char **argv)
{    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
    {

        // the sample supports offscreen rendering
        ProgramArguments::Option_t("offscreen", "0"),
    },
    "Sample illustrating how to use an image streamer given a CPU image");

    // Window/GL based application
    ImageStreamerMultiThreadApp app(args);
    app.initializeWindow("Image Streamer Multi", 1280, 800, args.enabled("offscreen"));
    return app.run();
}
