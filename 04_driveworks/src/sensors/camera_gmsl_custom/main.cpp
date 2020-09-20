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
// Copyright (c) 2015-2017 NVIDIA Corporation. All rights reserved.
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
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#endif
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>
#include <thread>
#include <queue>

#include <chrono>
#include <mutex>
#include <condition_variable>
#include <lodepng.h>

#include <framework/Checks.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/WindowEGL.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/Grid.hpp>
#include <framework/Log.hpp>

#include <framework/DataPath.hpp>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// RENDERER
#include <dw/renderer/Renderer.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>

// IMAGE
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>

#include "captureConfig.h"

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool g_run = true;
static bool gTakeScreenshot = false;
static int gScreenshotCount = 0;
GridData_t g_grid;

uint32_t g_imageWidth;
uint32_t g_imageHeight;
uint32_t g_numCameras;

// Program arguments
ProgramArguments g_arguments(
    {
        ProgramArguments::Option_t("config-file", (DataPath::get() + "/samples/sensors/camera/camera.cfg").c_str()),
        ProgramArguments::Option_t("fifo-size", "3"),

    });

std::vector<dwImageNvMedia*> g_frameRGBAPtr;
std::queue<dwImageNvMedia *> rgbaPool;

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void parseArguments(int argc, const char **argv);
void initGL(WindowBase **window);
void initRenderer(dwRendererHandle_t *renderer,
                  dwContextHandle_t context, WindowBase *window);
void initSdk(dwContextHandle_t *context, WindowBase *window);
void initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera,
                 dwImageType *cameraImageType,
                 dwContextHandle_t context, ProgramArguments &arguments, std::string configFile);

void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk,
                         dwSensorHandle_t camera);

dwStatus captureCamera(dwImageNvMedia *frameNVMrgba, dwSensorHandle_t cameraSensor,
                       uint32_t sibling, dwImageFormatConverterHandle_t yuv2rgba);

void renderFrame(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer);
void takeScreenshot(dwImageNvMedia *frameNVMrgba, uint32_t sibling);
void threadCameraPipeline(dwSensorHandle_t cameraSensor, dwImageFormatConverterHandle_t yuv2rgba,
                          dwImageStreamerHandle_t streamer, dwContextHandle_t sdk, WindowBase* window);

ExtImgDevParam setUpCustomCamera(std::string& configurationFile);

void sig_int_handler(int sig);
void keyPressCallback(int key);
void resizeCallback(int width, int height);

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    //SDK objects
    WindowBase *window            = nullptr;
    dwContextHandle_t sdk         = DW_NULL_HANDLE;
    dwRendererHandle_t renderer   = DW_NULL_HANDLE;
    dwSALHandle_t sal             = DW_NULL_HANDLE;
    dwSensorHandle_t cameraSensor = DW_NULL_HANDLE;

    // Set up linux signal handlers
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command

    //Init
    g_run = true;

    parseArguments(argc, argv);
    initGL(&window);
    initSdk(&sdk, window);
    initRenderer(&renderer, sdk, window);

    // create HAL and camera
    dwImageType cameraImageType;
    initSensors(&sal, &cameraSensor, &cameraImageType, sdk, g_arguments, g_arguments.get("config-file"));

    if(cameraImageType != DW_IMAGE_NVMEDIA)
    {
        std::cerr << "Error: Expected nvmedia image type, received "
                  << cameraImageType << " instead." << std::endl;
        exit(-1);
    }

    runNvMedia_pipeline(window, renderer, sdk, cameraSensor);

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
    dwContextParameters sdkParams;
    memset(&sdkParams, 0, sizeof(dwContextParameters));

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
    dwRect rect;
    rect.width   = window->width();
    rect.height  = window->height();
    rect.x = 0;
    rect.y = 0;

    dwRenderer_setRect(rect, *renderer);
}

//------------------------------------------------------------------------------
void initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera,
                 dwImageType *cameraImageType, dwContextHandle_t context,
                 ProgramArguments &arguments, std::string configFile)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot initialize SAL: "
                    << dwGetStatusName(result) << std::endl;
        exit(1);
    }

    std::string parameterString;
    parameterString            += "fifo-size=" + arguments.get("fifo-size");
    parameterString             += ",output-format=yuv";
    parameterString		+= ",custom-board=1";

    // create GMSL Camera interface
    dwSensorParams params;
    params.parameters           = parameterString.c_str();
    params.protocol             = "camera.gmsl";

    ExtImgDevParam extImgDevParam {};
    CaptureConfigParams dataset = parseConfigFile(configFile);

    extImgDevParam.board = const_cast<char*>(dataset.board);
    extImgDevParam.moduleName = const_cast<char*>(dataset.moduleName);
    extImgDevParam.resolution = const_cast<char*>(dataset.resolution);
    extImgDevParam.inputFormat = const_cast<char*>(dataset.inputFormat);
    extImgDevParam.sensorsNum = dataset.cameraCount;
    extImgDevParam.interface = const_cast<char*>(dataset.interface);
    extImgDevParam.i2cDevice = dataset.i2cDevice;
    extImgDevParam.desAddr = dataset.desAddr;
    extImgDevParam.brdcstSerAddr = dataset.brdcstSerAddr;
    extImgDevParam.brdcstSensorAddr = dataset.brdcstSensorAddr;
    extImgDevParam.reqFrameRate = dataset.requiredFrameRate;
    extImgDevParam.slave = dataset.slave;
    extImgDevParam.enableEmbLines = dataset.enableEmbLines;
    extImgDevParam.enableSimulator = dataset.enableSimulator;
    extImgDevParam.initialized = dataset.initialized;
    extImgDevParam.enableExtSync = dataset.crossCSISync;

    ExtImgDevMapInfo camMap{};
    camMap.csiOut = CSI_OUT_DEFAULT;
    camMap.enable = CAM_ENABLE_DEFAULT;
    camMap.mask = CAM_MASK_DEFAULT;

    camMap.enable = EXTIMGDEV_MAP_N_TO_ENABLE(dataset.cameraCount);
    if ((sscanf(dataset.csiOut, "%x", &camMap.csiOut)) != 1) {
        std::cerr << "An invalid value was specified for driveworks.csiOut" << std::endl;
        std::cerr << "using default value of 0x3210" << std::endl;
        camMap.csiOut = CSI_OUT_DEFAULT;
    }

    if ((sscanf(dataset.cameraMask, "%x", &camMap.mask)) != 1) {
        std::cerr << "An invalid value was specified for driveworks.mask" << std::endl;
        std::cerr << "using default value of 0x0000" << std::endl;
        camMap.mask = CAM_MASK_DEFAULT;
    }

    extImgDevParam.camMap = &camMap;

    params.auxiliarydata = reinterpret_cast<void*>(&extImgDevParam);

    result = dwSAL_createSensor(camera, params, *sal);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot create driver: camera.gmsl with params: "
                    << params.parameters << std::endl
                    << "Error: " << dwGetStatusName(result) << std::endl;
        exit(1);
    }

    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties,
                                     DW_CAMERA_PROCESSED_IMAGE,
                                    *camera);
    g_imageWidth = cameraImageProperties.width;
    g_imageHeight = cameraImageProperties.height;
    *cameraImageType = cameraImageProperties.type;

    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, *camera);
    g_numCameras = cameraProperties.siblings;

    std::cout << "Camera image with " << g_imageWidth << "x" << g_imageHeight
              << " at " << cameraProperties.framerate << " FPS" << std::endl;
}

//------------------------------------------------------------------------------
void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk,
                         dwSensorHandle_t cameraSensor)
{
    // RGBA image pool for conversion from YUV camera output
    for (size_t cameraIdx = 0; cameraIdx < g_numCameras; cameraIdx++) {
        g_frameRGBAPtr.push_back(nullptr);
    }

    // format converter
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensor);
    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
    displayImageProperties.planeCount = 1;

    dwImageFormatConverterHandle_t yuv2rgba = DW_NULL_HANDLE;
    dwStatus status                         = dwImageFormatConverter_initialize(&yuv2rgba,
                                                        cameraImageProperties.type,
                                                        sdk);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot initialize pixel format converter" << std::endl;
        exit(1);
    }

    // image API translator
    dwImageStreamerHandle_t nvm2gl = DW_NULL_HANDLE;
    status = dwImageStreamer_initialize(&nvm2gl, &displayImageProperties, DW_IMAGE_GL, sdk);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot init image streamer: " << dwGetStatusName(status) << std::endl;
        g_run = false;
    }

    std::thread camThread(threadCameraPipeline, cameraSensor, yuv2rgba, nvm2gl, sdk, window);

    configureGrid(&g_grid, window->width(), window->height(), g_imageWidth, g_imageHeight, g_numCameras);

    // loop through all cameras check if they have provided the first frame
    for (uint32_t cameraIdx = 0;
         cameraIdx < g_numCameras && g_run;
         cameraIdx++) {

        while (!g_frameRGBAPtr[cameraIdx] && g_run) {
            std::this_thread::yield();
        }
    }

    // all cameras have provided at least one frame, this thread can now start rendering
    window->makeCurrent();

    while(g_run) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render
        int32_t cellIdx = -1;

        for (uint32_t cameraIdx = 0;
             cameraIdx < g_numCameras && !rgbaPool.empty() && g_run;
             cameraIdx++) {

            cellIdx++;

            if (!g_run) {
                break;
            }

            status = dwImageStreamer_postNvMedia(g_frameRGBAPtr[cameraIdx],
                                                 nvm2gl);
            if (status != DW_SUCCESS) {
                std::cerr << "Cannot post nvmedia: " << dwGetStatusName(status) << std::endl;
                g_run = false;
                break;
            }

            //Set area
            dwRect rect;
            gridCellRect(&rect, g_grid, cellIdx);
            dwRenderer_setRect(rect, renderer);

            renderFrame(nvm2gl, renderer);

            // return frame which has been lately processed by the streamer
            dwImageNvMedia *processed = nullptr;
            status = dwImageStreamer_waitPostedNvMedia(&processed, 60000,
                                                       nvm2gl);
            if (status != DW_SUCCESS) {
                std::cerr << "Cannot waitpost nvmedia: " << dwGetStatusName(status) << std::endl;
                g_run = false;
                break;
            }
        }

        window->swapBuffers();
        CHECK_GL_ERROR();
    }

    camThread.join();

}

//------------------------------------------------------------------------------
dwStatus captureCamera(dwImageNvMedia *frameNVMrgba,
                       dwSensorHandle_t cameraSensor,
                       uint32_t sibling,
                       dwImageFormatConverterHandle_t yuv2rgba)
{
    dwCameraFrameHandle_t frameHandle;
    dwImageNvMedia *frameNVMyuv = nullptr;

    dwStatus result = DW_FAILURE;
    result = dwSensorCamera_readFrame(&frameHandle, sibling, 300000, cameraSensor);
    if (result != DW_SUCCESS) {
        std::cout << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
        return result;
    }

    result = dwSensorCamera_getImageNvMedia(&frameNVMyuv, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
    if( result != DW_SUCCESS ){
        std::cout << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;

    }

    result = dwImageFormatConverter_copyConvertNvMedia(frameNVMrgba, frameNVMyuv, yuv2rgba);
    if( result != DW_SUCCESS ){
        std::cout << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;

    }

    result = dwSensorCamera_returnFrame(&frameHandle);
    if( result != DW_SUCCESS ){
        std::cout << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;
    }

    return DW_SUCCESS;
}

//------------------------------------------------------------------------------
void renderFrame(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer)
{
    dwImageGL *frameGL = nullptr;

    if (dwImageStreamer_receiveGL(&frameGL, 60000, streamer) != DW_SUCCESS) {
        std::cerr << "did not received GL frame within 30ms" << std::endl;
    } else {
        // render received texture
        dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);

        //std::cout << "received GL: " << frameGL->prop.timestamp_us << std::endl;
        dwImageStreamer_returnReceivedGL(frameGL, streamer);
    }
}

//------------------------------------------------------------------------------
void takeScreenshot(dwImageNvMedia *frameNVMrgba, uint32_t sibling)
{
    NvMediaImageSurfaceMap surfaceMap;
    if (NvMediaImageLock(frameNVMrgba->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
    {
        char fname[128];
        sprintf(fname, "screenshot_%d_%04d.png", sibling, gScreenshotCount);
        lodepng_encode32_file(fname, (unsigned char*)surfaceMap.surface[0].mapping,
                frameNVMrgba->prop.width, frameNVMrgba->prop.height);
        NvMediaImageUnlock(frameNVMrgba->img);
        std::cout << "SCREENSHOT TAKEN to " << fname << "\n";
    }else
    {
        std::cout << "CANNOT LOCK NVMEDIA IMAGE - NO SCREENSHOT\n";
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

    // take screenshot
    if (key == GLFW_KEY_S)
        gTakeScreenshot = true;
}

//------------------------------------------------------------------------------
void resizeCallback(int width, int height)
{
    configureGrid(&g_grid, width, height, g_imageWidth, g_imageHeight, g_numCameras);
}

//#######################################################################################
void threadCameraPipeline(dwSensorHandle_t cameraSensor, dwImageFormatConverterHandle_t yuv2rgba,
                          dwImageStreamerHandle_t streamer, dwContextHandle_t sdk, WindowBase* window)
{
    dwStatus result;

    int32_t pool_size = 8;

    uint32_t numFramesRGBA = pool_size * g_numCameras;

    bool eof;
    // RGBA image pool for conversion from YUV camera output
    // two RGBA frames per camera per sibling for a pool
    // since image streamer might hold up-to one frame when using egl streams
    std::vector<dwImageNvMedia> frameRGBA;
    frameRGBA.reserve(numFramesRGBA);
    {
        dwImageProperties cameraImageProperties;
        dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE,
                                          cameraSensor);

        dwImageProperties displayImageProperties = cameraImageProperties;
        displayImageProperties.pxlFormat         = DW_IMAGE_RGBA;
        displayImageProperties.planeCount        = 1;

        // format converter
        result = dwImageFormatConverter_initialize(&yuv2rgba, cameraImageProperties.type, sdk);

        if (result != DW_SUCCESS) {
            std::cerr << "Cannot create pixel format converter : yuv->rgba" <<
                         dwGetStatusName(result) <<  std::endl;
            g_run = false;
        }
        // allocate pool
        for (uint32_t cameraIdx = 0; cameraIdx < g_numCameras; cameraIdx++) {
            for (int32_t k = 0; k < pool_size; k++) {
                dwImageNvMedia rgba{};
                result = dwImageNvMedia_create(&rgba, &displayImageProperties, sdk);
                if (result != DW_SUCCESS) {
                    std::cerr << "Cannot create nvmedia image for pool:" <<
                                 dwGetStatusName(result) << std::endl;
                    g_run = false;
                    break;
                }

                frameRGBA.push_back(rgba);
                rgbaPool.push(&frameRGBA.back());
            }
        }

        g_run = g_run && dwSensor_start(cameraSensor) == DW_SUCCESS;
        eof = false;
    }

    // main loop
    while (g_run && !window->shouldClose()) {
        bool eofAny = false;

        {
            if (eof) {
                eofAny = true;
                continue;
            }

            if (rgbaPool.empty()) {
                std::cerr << "Ran out of RGBA buffers, continuing" << std::endl;
                continue;
            }

            // capture from all cameras
            for (uint32_t cameraIdx = 0;
                 cameraIdx < g_numCameras && !rgbaPool.empty();
                 cameraIdx++) {


                // capture, convert to rgba and return it
                eof = captureCamera(rgbaPool.front(),
                                    cameraSensor, cameraIdx,
                                    yuv2rgba);
                g_frameRGBAPtr[cameraIdx] = rgbaPool.front();
                rgbaPool.pop();

                if (!eof) {
                    rgbaPool.push(g_frameRGBAPtr[cameraIdx]);
                }

                eofAny |= eof;
            }
        }

        // stop to take screenshot (will cause a delay)
        if (gTakeScreenshot) {
            {
                for (uint32_t cameraIdx = 0;
                    cameraIdx < g_numCameras && !rgbaPool.empty();
                     cameraIdx++) {

                    takeScreenshot(g_frameRGBAPtr[cameraIdx], cameraIdx);
                }
            }
            gScreenshotCount++;
            gTakeScreenshot = false;
        }

        // computation
        std::this_thread::sleep_for(std::chrono::milliseconds(15));

        g_run = g_run && !eofAny;
    }

    //Clean up
    // release used objects in correct order
    {
        dwSensor_stop(cameraSensor);
        dwSAL_releaseSensor(&cameraSensor);

        dwImageStreamer_release(&streamer);
        dwImageFormatConverter_release(&yuv2rgba);
    }
    for (dwImageNvMedia& frame : frameRGBA) {
        dwStatus result = dwImageNvMedia_destroy(&frame);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot destroy nvmedia: " << dwGetStatusName(result) << std::endl;
            g_run = false;
            break;
        }
    }
}

