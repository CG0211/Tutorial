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

#include <signal.h>

#include <cstring>
#include <cmath>

#include <chrono>
#include <thread>

// SAMPLE COMMON
#include <framework/Checks.hpp>
#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif

#include <framework/Log.hpp>
#include <framework/DataPath.hpp>
#include <framework/ProgramArguments.hpp>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>

// Renderer
#include <dw/renderer/Renderer.h>

// IMAGE
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>

// RCCB
#include <dw/isp/SoftISP.h>

//#define MANAGED_MEMORY

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool g_run = true;

cudaStream_t g_cudaStream  = 0;

// Program arguments
ProgramArguments g_arguments({
    ProgramArguments::Option_t("video", (DataPath::get() + "/samples/raw/rccb.raw").c_str()),
    ProgramArguments::Option_t("disableDenoising", "0"),
    ProgramArguments::Option_t("interpolationDemosaic", "0"),
    ProgramArguments::Option_t("enableCrop", "0"),
    ProgramArguments::Option_t("cropLeft", "-1"),
    ProgramArguments::Option_t("cropTop", "-1"),
    ProgramArguments::Option_t("cropWidth", "-1"),
    ProgramArguments::Option_t("cropHeight", "-1"),
    ProgramArguments::Option_t("stopFrame", "0"),
    ProgramArguments::Option_t("dumpOutput", "0"),
});

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void parseArguments(int argc, const char **argv);
void initGL(WindowBase **window);
void initRenderer(dwRendererHandle_t *renderer, dwContextHandle_t context, WindowBase *window);
void initSdk(dwContextHandle_t *context, WindowBase *window);
bool initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera, dwImageProperties *cameraImageProperties,
                 dwCameraProperties* cameraProperties, dwContextHandle_t context);

void runPipeline(WindowBase *window, dwRendererHandle_t renderer, dwSensorHandle_t camera,
                 dwSoftISPHandle_t pipeline, dwImageStreamerHandle_t cpu2cuda,
                 dwContextHandle_t sdk, float32_t framerate,
                 bool enableCropOutput, dwRect cropRegion);

void sig_int_handler(int sig);
void keyPressCallback(int key);

#include <stdio.h>

// @todo: Remove this duplication, once C++ wrappers for the dwImageXXX types are in place.
void dumpDwImageCUDA(const dwImageCUDA& image, const char* filename)
{
    const size_t pitch = image.pitch[0];
    const uint32_t width = image.prop.width;
    // @todo: Remove this WAR and make it work for general cases.
    const uint32_t height = (image.prop.pxlFormat == DW_IMAGE_RCB ?  3 * image.prop.height : image.prop.height);
    void* data = image.dptr[0];

    // @todo: Move this to a proper place? Maybe somewhere in dw/image?
    size_t bytesPerPixel = 0;
    switch(image.prop.pxlType)
    {
    case DW_TYPE_INT8:
    case DW_TYPE_UINT8:
        bytesPerPixel = 1;
        break;
    case DW_TYPE_INT16:
    case DW_TYPE_UINT16:
    case DW_TYPE_FLOAT16:
        bytesPerPixel = 2;
        break;
    case DW_TYPE_INT32:
    case DW_TYPE_UINT32:
    case DW_TYPE_FLOAT32:
        bytesPerPixel = 4;
        break;
    case DW_TYPE_INT64:
    case DW_TYPE_UINT64:
    case DW_TYPE_FLOAT64:
        bytesPerPixel = 8;
        break;
    case DW_TYPE_UNKNOWN:
    case DW_TYPE_BOOL:
    default:
        std::cerr << "Invalid or unsupported pixel type." << std::endl;
        return;
    }

    const size_t host_pitch = width * bytesPerPixel;
    char* hbuf = new char[host_pitch * height];
    if (hbuf)
    {
        cudaError_t error = cudaMemcpy2D(static_cast<void*>(hbuf), host_pitch, data, pitch, host_pitch, height, cudaMemcpyDeviceToHost);

        if (error != cudaSuccess) {
            std::cerr << "dumpOutput, memcopy failed." << std::endl;
            return;
        }

        FILE* fp;
#ifdef WINDOWS
        fopen_s(&fp, filename, "wb");
#else
        fp = fopen(filename, "wb");
#endif
        size_t numWritten = fwrite(hbuf, host_pitch * height, 1, fp);
        if (numWritten <= 0)
            std::cerr << "Writing failed." << std::endl;
        fclose(fp);

        delete[] hbuf;
    }
}


//------------------------------------------------------------------------------
// Method implementations
//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    // SDK objects
    WindowBase *window            = nullptr;
    dwContextHandle_t sdk         = DW_NULL_HANDLE;
    dwRendererHandle_t renderer   = DW_NULL_HANDLE;
    dwSALHandle_t sal             = DW_NULL_HANDLE;
    dwSensorHandle_t cameraSensor = DW_NULL_HANDLE;

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
    g_run = true;

    parseArguments(argc, argv);

    bool enableCropOutput = (std::stoi(g_arguments.get("enableCrop")) != 0);

    initGL(&window);
    initSdk(&sdk, window);
    initRenderer(&renderer, sdk, window);

    int32_t cropLeft = std::stoi(g_arguments.get("cropLeft"));
    int32_t cropTop = std::stoi(g_arguments.get("cropTop"));
    int32_t cropWidth = std::stoi(g_arguments.get("cropWidth"));
    int32_t cropHeight = std::stoi(g_arguments.get("cropHeight"));


    // create HAL and camera
    dwImageProperties rawImageProps;
    dwCameraProperties cameraProps;
    bool sensorsInitialized = initSensors(&sal, &cameraSensor, &rawImageProps, &cameraProps, sdk);

    uint32_t imageWidth = rawImageProps.width;
    uint32_t imageHeight = rawImageProps.height;
    float32_t framerate = cameraProps.framerate;

    // To simplify cropping logic, let's make sure imageWidth and imageHeight are divisible by 4.
    bool validImageSizeForCrop = !enableCropOutput || ((imageWidth % 4 == 0) && (imageHeight % 4 == 0));

    if ((cameraProps.cameraType != DW_CAMERA_GMSL_AR0231_RCCB) &&
        (cameraProps.cameraType != DW_CAMERA_GMSL_AR0231_RCCB_BAE) &&
        (cameraProps.cameraType != DW_CAMERA_GMSL_AR0231_RCCB_SSC) &&
        (cameraProps.cameraType != DW_CAMERA_GMSL_AR0231_RCCB_SS3322) &&
        (cameraProps.cameraType != DW_CAMERA_GMSL_AR0231_RCCB_SS3323) &&
        (cameraProps.cameraType != DW_CAMERA_GENERIC)) {
        std::cerr << "Raw pipeline sample: " << "This sample is supposed to demonstrate how the RCCB raw "
                                                "pipeline is setup and used, therefore videos captured with "
                                                "other raw cameras are not supported. For examples on how to"
                                                "capture from a recorded video please see sample_camera_replay"
                                                ". For an example on how to convert from generic raw to RGBA"
                                                "please see sample_camera_gmsl_raw.\n"
                                                "Currently supported are 'ar0231-rccb', 'ar0231-rccb-bae' and"
                                                "'ar0231-rccb-ssc'" << std::endl;

    } else if (sensorsInitialized && validImageSizeForCrop) {
        // Set up crop region

        dwRect cropRegion;
        bool useDefaultcropRegion = (cropLeft < 0) || (cropTop < 0);
        cropRegion.x   = useDefaultcropRegion ?  static_cast<unsigned short>(imageWidth * 0.25f + 0.5f) : static_cast<unsigned short>(cropLeft);
        cropRegion.y    = useDefaultcropRegion ?  static_cast<unsigned short>(imageHeight * 0.25f + 0.5f) : static_cast<unsigned short>(cropTop);
        cropRegion.width  = useDefaultcropRegion ?  static_cast<unsigned short>(imageWidth * 0.5f + 0.5f) : static_cast<unsigned short>(cropWidth);
        cropRegion.height = useDefaultcropRegion ?  static_cast<unsigned short>(imageHeight * 0.5f + 0.5f) : static_cast<unsigned short>(cropHeight);
        // top left coords should be even
        if(cropRegion.x&1)
            cropRegion.x += 1;
        if(cropRegion.y&1)
            cropRegion.y += 1;
        std::cout << "cropRegion (Left,Top,Width, Height) = ("
                  << cropRegion.x << ", " << cropRegion.y << ", "
                  << cropRegion.width  << ", " << cropRegion.height << ")" << std::endl;

        // Let's do the job

        dwSoftISPHandle_t rccb;
        dwSoftISPParams softISPParams;
        dwSoftISP_initParamsFromCamera(&softISPParams, cameraProps);
        CHECK_DW_ERROR(dwSoftISP_initialize(&rccb, softISPParams, sdk));
        dwSoftISP_setCUDAStream(g_cudaStream, rccb);
        if( std::stoi(g_arguments.get("interpolationDemosaic")) > 0 ) {
            dwSoftISP_setDemosaicMethod(DW_SOFT_ISP_DEMOSAIC_METHOD_INTERPOLATION, rccb);
        }
        if( std::stoi(g_arguments.get("disableDenoising")) > 0 ) {
            dwSoftISP_setDenoiseMethod(DW_SOFT_ISP_DENOISE_METHOD_NONE, rccb);
        }

        dwImageStreamerHandle_t cpu2cuda;
        dwImageStreamer_initialize(&cpu2cuda, &rawImageProps, DW_IMAGE_CUDA, sdk);

        runPipeline(window, renderer, cameraSensor, rccb, cpu2cuda, sdk, framerate,
                    enableCropOutput, cropRegion);

        dwImageStreamer_release(&cpu2cuda);
        dwSoftISP_release(&rccb);
    }

    // Clean up release used objects in correct order
    if (renderer)
        dwRenderer_release(&renderer);
    if (cameraSensor)
        dwSAL_releaseSensor(&cameraSensor);
    if (sal)
        dwSAL_release(&sal);
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
    if (!*window)
        *window = new WindowGLFW(1280, 800);

    (*window)->makeCurrent();
    (*window)->setOnKeypressCallback(keyPressCallback);
}

//------------------------------------------------------------------------------
void initSdk(dwContextHandle_t *context, WindowBase *window)
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = window->getEGLDisplay();
#else
    (void)window;
#endif

    dwInitialize(context, DW_VERSION, &sdkParams);
}

//------------------------------------------------------------------------------
void initRenderer(dwRendererHandle_t *renderer, dwContextHandle_t context, WindowBase *window)
{
    dwStatus result;

    result = dwRenderer_initialize(renderer, context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init renderer: ") + dwGetStatusName(result));

    // Set some renderer defaults
    dwRect rect;
    rect.width  = window->width();
    rect.height = window->height();
    rect.x      = 0;
    rect.y      = 0;

    dwRenderer_setRect(rect, *renderer);
}

//------------------------------------------------------------------------------
bool initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera, dwImageProperties *cameraImageProperties,
                 dwCameraProperties* cameraProperties, dwContextHandle_t context)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot initialize SAL: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    // create GMSL Camera interface
    dwSensorParams params;
    std::string parameterString = g_arguments.parameterString();

    params.parameters           = parameterString.c_str();
    params.protocol             = "camera.virtual";
    result                      = dwSAL_createSensor(camera, params, *sal);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot create driver: camera.virtual with params: " << params.parameters << std::endl
                  << "Error: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    ;
    dwSensorCamera_getImageProperties(cameraImageProperties, DW_CAMERA_RAW_IMAGE, *camera);
    dwSensorCamera_getSensorProperties(cameraProperties, *camera);

    std::cout << "Camera image with " << cameraImageProperties->width << "x"
        << cameraImageProperties->height << " at " << cameraProperties->framerate << " FPS"
              << std::endl;

    return true;
}

//------------------------------------------------------------------------------
void runPipeline(WindowBase *window, dwRendererHandle_t renderer, dwSensorHandle_t camera,
                 dwSoftISPHandle_t softISP, dwImageStreamerHandle_t cpu2cuda,
                 dwContextHandle_t sdk,  float32_t framerate,
                 bool enableCropOutput, dwRect cropRegion)
{
    (void) cpu2cuda;
    dwStatus result = DW_FAILURE;

    g_run = g_run && dwSensor_start(camera) == DW_SUCCESS;

    typedef std::chrono::high_resolution_clock myclock_t;
    typedef std::chrono::time_point<myclock_t> timepoint_t;
    auto frameDuration         = std::chrono::milliseconds((int)(1000 / framerate));
    timepoint_t lastUpdateTime = myclock_t::now();

    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_RAW_IMAGE, camera);

    // Allocate output images
    if( enableCropOutput ) {
        dwSoftISP_setDemosaicROI(cropRegion, softISP);
    }

    dwImageProperties rcbProperties;
    dwSoftISP_getDemosaicImageProperties(&rcbProperties, softISP);
    dwImageCUDA rcbImage;
    result = dwImageCUDA_create(&rcbImage, &rcbProperties, DW_IMAGE_CUDA_PITCH);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot create RCB CUDA image" << std::endl;
        g_run = false;
    }

    dwSoftISP_bindDemosaicOutput(&rcbImage, softISP);

    // RGBA image to display
    dwImageProperties rgbaImageProperties = rcbProperties;
    rgbaImageProperties.pxlFormat         = DW_IMAGE_RGBA;
    rgbaImageProperties.pxlType           = DW_TYPE_UINT8;
    rgbaImageProperties.planeCount        = 1;

    dwImageCUDA rgbaImage;
    result = dwImageCUDA_create(&rgbaImage, &rgbaImageProperties, DW_IMAGE_CUDA_PITCH);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot create RGBA CUDA image" << std::endl;
        g_run = false;
    }

    dwSoftISP_bindTonemapOutput(&rgbaImage, softISP);

    // Setup the streamer depending on display size
    dwImageStreamerHandle_t cuda2gl = 0;
    result = dwImageStreamer_initialize(&cuda2gl, &rgbaImageProperties, DW_IMAGE_GL, sdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Image streamer initialization failed: " << dwGetStatusName(result) << std::endl;
        g_run = false;
    }

    dwImageFormatConverterHandle_t convert2RGBA;
    dwImageFormatConverter_initialize(&convert2RGBA, rcbProperties.type, sdk);

    uint32_t frame = 0;
    uint32_t stopFrame = atoi(g_arguments.get("stopFrame").c_str());

    while (g_run && !window->shouldClose()) {
        std::this_thread::yield();

        // run with at most 30FPS
        std::chrono::milliseconds timeSinceUpdate =
            std::chrono::duration_cast<std::chrono::milliseconds>(myclock_t::now() - lastUpdateTime);
        if (timeSinceUpdate < frameDuration)
            continue;

        lastUpdateTime = myclock_t::now();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        dwCameraFrameHandle_t frameHandle;
        result = dwSensorCamera_readFrame(&frameHandle, 0, 4000000, camera);
        if (result == DW_END_OF_STREAM) {
            std::cout << "Camera reached end of stream" << std::endl;
            dwSensor_reset(camera);
            continue;
        }
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot read frame: " << dwGetStatusName(result) << std::endl;
            continue;
        }

        dwImageCUDA *rawImageCUDA;
        const dwImageDataLines* dataLines;
        dwImageCPU *rawImageCPU;

        result = dwSensorCamera_getImageCPU(&rawImageCPU, DW_CAMERA_RAW_IMAGE, frameHandle);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot get raw image: " << dwGetStatusName(result) << std::endl;
            continue;
        }
        dwSensorCamera_getDataLines(&dataLines, frameHandle);

        // stream to CUDA
        CHECK_DW_ERROR(dwImageStreamer_postCPU(rawImageCPU, cpu2cuda));
        CHECK_DW_ERROR(dwImageStreamer_receiveCUDA(&rawImageCUDA, 10000, cpu2cuda));

        // process the results and output in the bound outputs (rgbaImage and rcbImage)
        dwSoftISP_bindRawInput(rawImageCUDA, softISP);
        CHECK_DW_ERROR(dwSoftISP_processDeviceAsync(DW_SOFT_ISP_PROCESS_TYPE_DEMOSAIC | DW_SOFT_ISP_PROCESS_TYPE_TONEMAP,
                                                    softISP));

        CHECK_DW_ERROR(dwImageStreamer_returnReceivedCUDA(rawImageCUDA, cpu2cuda));
        dwImageCPU *rawImageCPUWait;
        CHECK_DW_ERROR(dwImageStreamer_waitPostedCPU(&rawImageCPUWait, 10000, cpu2cuda));

        if (std::stoi(g_arguments.get("dumpOutput")) != 0) {
            static unsigned int frameNumber = 0;
            std::string filename = "rcb" + std::to_string(frameNumber) + ".half";
            dumpDwImageCUDA(rcbImage, filename.c_str());
            frameNumber++;
        }

        // render received texture
        dwImageGL *frameGL;

        CHECK_DW_ERROR(dwImageStreamer_postCUDA(&rgbaImage, cuda2gl));
        result = dwImageStreamer_receiveGL(&frameGL, 10000, cuda2gl);
        if( result == DW_SUCCESS ) {
            dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);

            dwImageStreamer_returnReceivedGL(frameGL, cuda2gl);
        }

        dwImageCUDA *returnedFrame;
        dwImageStreamer_waitPostedCUDA(&returnedFrame, 10000, cuda2gl);

        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frameHandle));

        window->swapBuffers();

        ++frame;
        if(stopFrame && frame == stopFrame)
            break;
    }

    dwImageCUDA_destroy(&rgbaImage);
    dwImageCUDA_destroy(&rcbImage);

    dwImageFormatConverter_release(&convert2RGBA);
    dwImageStreamer_release(&cuda2gl);
    dwSensor_stop(camera);
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
}
