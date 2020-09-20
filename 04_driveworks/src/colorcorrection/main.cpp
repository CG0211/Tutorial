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
// Copyright (c) 2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <signal.h>
#include <iostream>

#include <cstring>
#include <thread>

// SAMPLE COMMON
#include <framework/Checks.hpp>
#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#else
#endif
#include <framework/Grid.hpp>
#include <framework/DataPath.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/Log.hpp>

// SDK
#include <dw/core/Logger.h>
#include <dw/core/Context.h>
#include <dw/renderer/Renderer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/ImageStreamer.h>
#include <dw/image/FormatConverter.h>

#include <dw/rigconfiguration/RigConfiguration.h>
#include <dw/colorcorrection/ColorCorrection.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static bool volatile g_run = true;

ProgramArguments g_arguments({
    ProgramArguments::Option_t("video1",
                                (DataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
    ProgramArguments::Option_t("video2",
                                (DataPath::get() + "/samples/sfm/triangulation/video_1.h264").c_str()),
    ProgramArguments::Option_t("video3",
                                (DataPath::get() + "/samples/sfm/triangulation/video_2.h264").c_str()),
    ProgramArguments::Option_t("video4",
                                (DataPath::get() + "/samples/sfm/triangulation/video_3.h264").c_str()),
    ProgramArguments::Option_t("rig",
                                (DataPath::get() + "/samples/sfm/triangulation/rig.xml").c_str()),

    ProgramArguments::Option_t("ref", "2"),
    ProgramArguments::Option_t("factor", "0.8")
});

uint32_t g_imageWidth;
uint32_t g_imageHeight;
GridData_t g_grid;

/// @brief color correct types supported by the module
typedef enum
{
    /**
     * No color correction, original image
     */
     DW_COLOR_CORRECT_NONE = 0,

    /**
     * global correction using all the reprojected topview
     */
     DW_COLOR_CORRECT_GLOBAL,
}dwColorCorrectType;

dwColorCorrectType g_ccType = DW_COLOR_CORRECT_GLOBAL;

// 0-front, 1-rear, 2-left, 3-right,
// {0, 2, 1, 3} means front-left-rear-right sequence
// so that each 2 neighbor cameras will have
// overlapping region in reprojected topview.
uint32_t g_nextCamera[] = {0, 2, 1, 3};

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void parseArguments(int argc, const char **argv);
void initGL(WindowBase **window);
void initRenderer(dwRendererHandle_t *renderer,
                  dwContextHandle_t context, WindowBase *window);
void initSdk(dwContextHandle_t *context, WindowBase *window);
void initSAL(dwSALHandle_t *sal, dwContextHandle_t context);
dwStatus initSensor(dwSensorHandle_t *sensor,
                uint32_t *imageWidth, uint32_t *imageHeight,
                float32_t *framerate,
                dwImageType *cameraImageType,
                dwSALHandle_t sal,
                const std::string &videoFName);

dwStatus runSingleCameraPipeline(int curIdx,
                                 dwImageCUDA *frameCUDArgba,
                                 dwSensorHandle_t cameraSensor,
                                 dwImageFormatConverterHandle_t yuv2rgba,
                                 dwColorCorrectHandle_t stitcher,
#ifdef VIBRANTE
                                 dwImageStreamerHandle_t nvm2cuda,
#endif
                                 dwImageStreamerHandle_t cuda2gl,
                                 dwRendererHandle_t renderer,
                                 float factor, bool isRef);

void renderFrame(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer);

void sig_int_handler(int sig);
void keyPressCallback(int key);
void resizeCallback(int width, int height);


//------------------------------------------------------------------------------
// Method implementations
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    //SDK objects
    WindowBase *window            = nullptr;
    dwContextHandle_t sdk         = DW_NULL_HANDLE;
    dwRendererHandle_t renderer   = DW_NULL_HANDLE;
    dwSALHandle_t sal             = DW_NULL_HANDLE;
    dwSensorHandle_t cameraSensor[4] = { DW_NULL_HANDLE };

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

    dwStatus result;

    g_run = true;

    parseArguments(argc, argv);
    initGL(&window);
    initSdk(&sdk, window);
    initRenderer(&renderer, sdk, window);
    initSAL(&sal, sdk);

    // create sensors
    dwImageType imageType;
    float32_t framerate;

    for (int i = 0; i < 4; i++) {
        std::string param = std::string("video") + std::to_string(i + 1);

        result = initSensor(&cameraSensor[i], &g_imageWidth, &g_imageHeight,
                          &framerate,
                          &imageType,
                          sal,
                          g_arguments.get(param.c_str()));
        if(result != DW_SUCCESS) {
            std::cerr << "Cannot create sensor\n";
            exit(1);
        }
    }

    //Configure grid
    configureGrid(&g_grid, window->width(), window->height(), g_imageWidth, g_imageHeight, 4);

    // format converter
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensor[0]);
    cameraImageProperties.type = DW_IMAGE_CUDA;
    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
    displayImageProperties.planeCount = 1;
    dwImageFormatConverterHandle_t yuv2rgba = DW_NULL_HANDLE;
    result = dwImageFormatConverter_initialize(&yuv2rgba, cameraImageProperties.type, sdk);

    if (result != DW_SUCCESS) {
        std::cerr << "Cannot create pixel format converter : yuv->rgba" << std::endl;
        exit(1);
    }

    // image API translator
#ifdef VIBRANTE
    dwImageStreamerHandle_t nvm2cuda = DW_NULL_HANDLE;
    // TODO: align this with cameraImageProperties instead of hard-coding
    dwImageProperties nvm2cudaProps = cameraImageProperties;
    nvm2cudaProps.type = DW_IMAGE_NVMEDIA;
    nvm2cudaProps.pxlFormat = DW_IMAGE_YUV420;
    nvm2cudaProps.pxlType = DW_TYPE_UINT8;
    result = dwImageStreamer_initialize(&nvm2cuda, &nvm2cudaProps, DW_IMAGE_CUDA, sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot init NvMedia2Cuda image streamer: " << dwGetStatusName(result) << std::endl;
        g_run = false;
    }

#endif

    dwImageStreamerHandle_t cuda2gl = DW_NULL_HANDLE;
    // TODO: align this with displayImageProperties instead of hard-coding
    dwImageProperties cuda2glProps = displayImageProperties;
    cuda2glProps.type = DW_IMAGE_CUDA;
    cuda2glProps.pxlFormat = DW_IMAGE_RGBA;
    cuda2glProps.planeCount = 1;
    cuda2glProps.pxlType = DW_TYPE_UINT8;
    result = dwImageStreamer_initialize(&cuda2gl, &cuda2glProps, DW_IMAGE_GL, sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot init Cuda2GL image streamer: " << dwGetStatusName(result) << std::endl;
        g_run = false;
    }

    dwImageCUDA frameRGBA[4];
    for (int i = 0; i < 4 && g_run; i++) {
        void *dptr   = nullptr;
        size_t pitch = 0;
        cudaMallocPitch(&dptr, &pitch, g_imageWidth * 4, g_imageHeight);
        dwImageCUDA_setFromPitch(&frameRGBA[i], dptr,
                                 g_imageWidth, g_imageHeight, pitch,
                                 DW_IMAGE_RGBA);
    }

    for (int i = 0; i < 4 && g_run; i++) {
        g_run = g_run && dwSensor_start(cameraSensor[i]) == DW_SUCCESS;
    }


    dwRigConfigurationHandle_t rigConfig = DW_NULL_HANDLE;
    result = dwRigConfiguration_initializeFromFile(&rigConfig, sdk, g_arguments.get("rig").c_str());
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot init rig configuration from file: " << dwGetStatusName(result) << std::endl;
        exit(1);
    }
    dwColorCorrectHandle_t cc = DW_NULL_HANDLE;
    dwColorCorrectParameters ccParams{};
    ccParams.cameraWidth = g_imageWidth;
    ccParams.cameraHeight = g_imageHeight;
    result = dwColorCorrect_initializeFromRig(&cc, sdk, rigConfig, &ccParams);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot init color correct object: " << dwGetStatusName(result) << std::endl;
        exit(1);
    }
    dwRigConfiguration_release(&rigConfig);

    typedef std::chrono::high_resolution_clock myclock_t;
    typedef std::chrono::time_point<myclock_t> timepoint_t;
    timepoint_t lastUpdateTime = myclock_t::now();
    auto frameDuration = std::chrono::milliseconds((int)(1000.0f / framerate));

    int refIdx = std::stoi(g_arguments.get("ref"));
    if (refIdx < 0 || refIdx > 3) {
        std::cerr << "--ref [n] expect to be within [0, 3]!" << std::endl;
        g_run = 0;
    }

    int refPos = g_nextCamera[refIdx];
    float factor = std::stof(g_arguments.get("factor"));
    while (g_run && !window->shouldClose()) {
        std::this_thread::yield();

        // run with at most 30FPS
        std::chrono::milliseconds timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(myclock_t::now() - lastUpdateTime);
        if (timeSinceUpdate < frameDuration)
            continue;

        lastUpdateTime = myclock_t::now();
        dwRect rect;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // first add the ref view
        gridCellRect(&rect, g_grid, refIdx);
        dwRenderer_setRect(rect, renderer);

        result = runSingleCameraPipeline(refIdx, &frameRGBA[refIdx],
                                         cameraSensor[refIdx],
                                         yuv2rgba, cc,
#ifdef VIBRANTE
                                         nvm2cuda,
#endif
                                         cuda2gl, renderer,
                                         factor, true);
        if (result == DW_END_OF_STREAM) {
            dwSensor_reset(cameraSensor[refIdx]);
        }


        for (int i = 1; i < 4; i++) {
            //Set area
            int curCam = g_nextCamera[(refPos+i)%4];

            gridCellRect(&rect, g_grid, curCam);
            dwRenderer_setRect(rect, renderer);

            //Run sensor
            result = runSingleCameraPipeline(curCam, &frameRGBA[curCam],
                                             cameraSensor[curCam],
                                             yuv2rgba, cc,
#ifdef VIBRANTE
                                             nvm2cuda,
#endif
                                             cuda2gl, renderer,
                                             factor, false);

            if (result == DW_END_OF_STREAM) {
                dwSensor_reset(cameraSensor[curCam]);
            }
        }

        window->swapBuffers();
        glGetError();
    }

    //Clean up
    // release used objects in correct order

    for (int i = 0; i < 4; i++) {
        cudaFree(frameRGBA[i].dptr[0]);
    }

    if (cc != DW_NULL_HANDLE)
        dwColorCorrect_release(&cc);

#ifdef VIBRANTE
    dwImageStreamer_release(&nvm2cuda);
#endif
    dwImageStreamer_release(&cuda2gl);
    dwImageFormatConverter_release(&yuv2rgba);

    for (int i = 0; i < 4; i++) {
        dwSensor_stop(cameraSensor[i]);
        dwSAL_releaseSensor(&cameraSensor[i]);
    }

    // release used objects in correct order
    dwSAL_release(&sal);
    dwRenderer_release(&renderer);
    dwRelease(&sdk);
    dwLogger_release();
    delete window;
    return 0;
}

//------------------------------------------------------------------------------
void parseArguments(int argc, const char **argv)
{
    if (!g_arguments.parse(argc, argv))
        exit(-1); // Exit if not all require arguments are provided

    std::cout << "Program Arguments:\n" << g_arguments.printList() << std::endl;
}

//------------------------------------------------------------------------------
void initGL(WindowBase **window)
{
    if(!*window)
        *window = new WindowGLFW(1280, 800);

    (*window)->makeCurrent();
    (*window)->setOnKeypressCallback(keyPressCallback);
    (*window)->setOnResizeWindowCallback(resizeCallback);
}

//------------------------------------------------------------------------------
void initSdk(dwContextHandle_t *context, WindowBase *window)
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams{};

#ifdef VIBRANTE
    sdkParams.eglDisplay = window->getEGLDisplay();
#else
    (void)window;
#endif

    dwInitialize(context, DW_VERSION, &sdkParams);
}

//------------------------------------------------------------------------------
void initRenderer(dwRendererHandle_t *renderer,
                  dwContextHandle_t context, WindowBase *window)
{
    dwStatus result;

    result = dwRenderer_initialize(renderer, context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init renderer: ") +
                                 dwGetStatusName(result));

    // Set some renderer defaults
    (void)window;
}

//------------------------------------------------------------------------------
void initSAL(dwSALHandle_t *sal, dwContextHandle_t context)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot initialize SAL: "
                    << dwGetStatusName(result) << std::endl;
        exit(1);
    }
}

//------------------------------------------------------------------------------
dwStatus initSensor(dwSensorHandle_t *sensor,
                uint32_t *imageWidth, uint32_t *imageHeight,
                float32_t *framerate,
                dwImageType *cameraImageType,
                dwSALHandle_t sal,
                const std::string &videoFName)
{
    std::string arguments = "video=" + videoFName;
#ifdef VIBRANTE
    arguments += ",yuv2rgb=cuda";
#endif

    dwStatus result;

    dwSensorParams params;
    params.parameters = arguments.c_str();
    params.protocol = "camera.virtual";
    result = dwSAL_createSensor(sensor, params, sal);
    if(result != DW_SUCCESS)
        return result;

    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties,
                                     DW_CAMERA_PROCESSED_IMAGE,
                                    *sensor);
    *imageWidth = cameraImageProperties.width;
    *imageHeight = cameraImageProperties.height;
    *cameraImageType = cameraImageProperties.type;

    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, *sensor);
    *framerate = cameraProperties.framerate;
    std::cout << "Camera image with " << *imageWidth << "x" << *imageHeight << " at "
              << *framerate << " FPS" << std::endl;

    return DW_SUCCESS;
}

//------------------------------------------------------------------------------
dwStatus runSingleCameraPipeline(int curIdx,
                                 dwImageCUDA *frameCUDArgba,
                                 dwSensorHandle_t cameraSensor,
                                 dwImageFormatConverterHandle_t yuv2rgba,
                                 dwColorCorrectHandle_t cc,
#ifdef VIBRANTE
                                 dwImageStreamerHandle_t nvm2cuda,
#endif
                                 dwImageStreamerHandle_t cuda2gl,
                                 dwRendererHandle_t renderer,
                                 float factor, bool isRef)
{
    dwCameraFrameHandle_t frameHandle;
    dwImageCUDA *frameCUDAyuv = nullptr;
    dwStatus result = DW_FAILURE;

    result = dwSensorCamera_readFrame(&frameHandle, 0, 30000, cameraSensor);
    if (result != DW_SUCCESS) {
        std::cout << "readFrameCUDA: " << dwGetStatusName(result) << std::endl;
        return result;
    }
#ifdef VIBRANTE
    dwImageNvMedia *frameNVMyuv = nullptr;
    result = dwSensorCamera_getImageNvMedia(&frameNVMyuv, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
    if (result != DW_SUCCESS) {
        std::cout << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
        return result;
    }
    dwImageStreamer_postNvMedia(frameNVMyuv, nvm2cuda);
    result = dwImageStreamer_receiveCUDA(&frameCUDAyuv, 30000, nvm2cuda);
    if (result != DW_SUCCESS) {
        std::cerr << "did not received CUDA frame within 30ms" << std::endl;
        return result;
    }
#else
    result = dwSensorCamera_getImageCUDA(&frameCUDAyuv, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
    if (result != DW_SUCCESS) {
        std::cout << "readFrameCUDA: " << dwGetStatusName(result) << std::endl;
        return result;
    }
#endif

    if (g_ccType != DW_COLOR_CORRECT_NONE) {
        if (isRef) {
            result = dwColorCorrect_setReferenceCameraView(frameCUDAyuv, curIdx, cc);
        }
        else if (g_ccType == DW_COLOR_CORRECT_GLOBAL) {
                result = dwColorCorrect_correctByReferenceView(frameCUDAyuv, curIdx, factor, cc);
        }

        if (result != DW_SUCCESS) {
            std::cout << "Failed to correct color " << dwGetStatusName(result) << std::endl;
            return result;
        }
    }

    dwImageFormatConverter_copyConvertCUDA(frameCUDArgba, frameCUDAyuv, yuv2rgba, 0);

#ifdef VIBRANTE
    dwImageNvMedia *processedNVM;
    dwImageStreamer_returnReceivedCUDA(frameCUDAyuv, nvm2cuda);
    dwImageStreamer_waitPostedNvMedia(&processedNVM, 30000, nvm2cuda);
#endif

    dwSensorCamera_returnFrame(&frameHandle);

    dwImageStreamer_postCUDA(frameCUDArgba, cuda2gl);
    renderFrame(cuda2gl, renderer);

    dwImageCUDA *processedCUDA;
    if (dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, cuda2gl) !=
            DW_SUCCESS ||
        processedCUDA != frameCUDArgba) {
        std::cerr << "consumer did not gave frame back" << std::endl;
    }

    return result;
}


//------------------------------------------------------------------------------
void renderFrame(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer)
{
    dwImageGL *frameGL = nullptr;

    if (dwImageStreamer_receiveGL(&frameGL, 30000, streamer) != DW_SUCCESS) {
        std::cerr << "did not received GL frame within 30ms" << std::endl;
    } else {
        // render received texture
        dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);

        //std::cout << "received GL: " << frameGL->prop.timestamp_us << std::endl;
        dwImageStreamer_returnReceivedGL(frameGL, streamer);

        // Overlay text
        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
        switch(g_ccType)
        {
        case DW_COLOR_CORRECT_NONE: dwRenderer_renderText(20, 20, "No color correction", renderer); break;
        case DW_COLOR_CORRECT_GLOBAL: dwRenderer_renderText(20, 20, "Color corrected", renderer); break;
        }
    }
}

//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;

    g_run = false;
}

//------------------------------------------------------------------------------
void keyPressCallback(int key)
{
    // stop application
    if (key == GLFW_KEY_ESCAPE)
        g_run = false;

    if (key == GLFW_KEY_SPACE) {
        switch (g_ccType)
        {
        case DW_COLOR_CORRECT_NONE: g_ccType = DW_COLOR_CORRECT_GLOBAL; break;
        case DW_COLOR_CORRECT_GLOBAL: g_ccType = DW_COLOR_CORRECT_NONE; break;
        }
    }
}
//------------------------------------------------------------------------------
void resizeCallback(int width, int height)
{
    configureGrid(&g_grid, width, height, g_imageWidth, g_imageHeight, 4);
}
