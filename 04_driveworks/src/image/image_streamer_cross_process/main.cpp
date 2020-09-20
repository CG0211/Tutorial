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
#include <unordered_map>
#include <string>
#include <unistd.h>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// window
#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#else
#endif

// Renderer
#include <dw/renderer/Renderer.h>

// IMAGE
#include <dw/image/ImageStreamer.h>
#include <dw/image/FormatConverter.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

#include <framework/ProgramArguments.hpp>
#include <framework/SampleFramework.hpp>

#define WINDOW_HEIGHT 800
#define WINDOW_WIDTH 1280

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

void processKey(int key)
{
    // stop application
    if (key == GLFW_KEY_ESCAPE) {
        std::cout<<"Quitting"<<std::endl;
        gRun = false;
    }
}

ProgramArguments g_arguments = ProgramArguments(
{
                ProgramArguments::Option_t("type", "consumer/producer"),
                ProgramArguments::Option_t("csi-port", "ab"),
                ProgramArguments::Option_t("timeout", "100000"),
                ProgramArguments::Option_t("fifo-size", "3"),
            });

//------------------------------------------------------------------------------
void initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera,
                 uint32_t *imageWidth, uint32_t *imageHeight, dwImageType *cameraImageType,
                 dwContextHandle_t context)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot initialize SAL: "
                    << dwGetStatusName(result) << std::endl;
        exit(1);
    }

    // create GMSL Camera interface
    dwSensorParams params;
    std::string parameterString = g_arguments.parameterString();
    parameterString             += ",output-format=yuv";
    parameterString             += ",camera-type=ar0231-rccb";
    params.parameters           = parameterString.c_str();
    params.protocol             = "camera.gmsl";
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
    *imageWidth = cameraImageProperties.width;
    *imageHeight = cameraImageProperties.height;
    *cameraImageType = cameraImageProperties.type;

    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, *camera);

    std::cout << "Camera image with " << *imageWidth << "x" << *imageHeight
              << " at " << cameraProperties.framerate << " FPS" << std::endl;
}

//------------------------------------------------------------------------------
void parseArguments(int argc, const char **argv)
{
    if (!g_arguments.parse(argc, argv))
        exit(-1); // Exit if not all require arguments are provided

    std::cout << "Program Arguments:\n" << g_arguments.printList() << std::endl;
}

//#######################################################################################
int main(int argc, const char **argv)
{
    bool run = true;
    gRun = true;

    parseArguments(argc, argv);

    bool isProd = false;

    if (g_arguments.get("type").compare("producer") == 0) {
        isProd = true;
    } else if (g_arguments.get("type").compare("consumer") != 0) {
        std::cerr << "Please specify a valid type.  "
                  << "Note: consumer must be created first." << std::endl;
        return -1;
    }

    dwTime_t timeout = std::stoul(g_arguments.get("timeout"));
    auto const SOCK_NAME     = "/tmp/driveworks_egl_stream";
    // Clean up file to make sure socket doesn't fail on connect
    unlink(SOCK_NAME);

    // HARDCODED ar0231
    uint32_t width = 1920;
    uint32_t height = 1208;

    dwStatus status;

    // Init SDK
    WindowBase *window;

    dwContextHandle_t sdk;
    status = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot init logger: " << dwGetStatusName(status) << std::endl;
        run = false;
    }

    status = dwLogger_setLogLevel(DW_LOG_VERBOSE);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot set log level: " << dwGetStatusName(status) << std::endl;
        run = false;
    }

    dwContextParameters sdkParams{};

    status = dwInitialize(&sdk, DW_VERSION, &sdkParams);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot init DriveWorks: " << dwGetStatusName(status) << std::endl;
        run = false;
    }

    //////////////////////
    if (isProd) {
        // PARENT producer
        std::cout << "IPC parent PID: " << getpid() << std::endl;


        // since we are relying on an EGL based image streamer (check user guide for more info), we need to
        // have a valid egl context. The context is alwyas created with the WindowGL, but since this process
        // does not render, the window itseld and the GL context are not necessary, hence we create only a
        // offscreen egl window
        window = new WindowOffscreenEGL(WINDOW_WIDTH, WINDOW_HEIGHT);

        if (window == nullptr) {
            std::cerr << "Cannot init window" << std::endl;
            run = false;
        }

        window->makeCurrent();

        // create HAL and camera
        uint32_t imageWidth;
        uint32_t imageHeight;
        dwImageType cameraImageType;
        dwSALHandle_t sal;
        dwSensorHandle_t cameraSensor;
        initSensors(&sal, &cameraSensor, &imageWidth, &imageHeight, &cameraImageType, sdk);

        dwImageProperties cameraImageProperties;
        dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensor);

        auto streamerNVM2CUDA = dwImageStreamerHandle_t{};
        status = dwImageStreamer_initializeCrossProcess(&streamerNVM2CUDA,
                                                        &cameraImageProperties, DW_IMAGE_CUDA,
                                                        {SOCK_NAME, DW_IMAGE_STREAMER_CROSS_PROCESS_PRODUCER},
                                                        sdk);

        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init gl image streamer: " << dwGetStatusName(status) << std::endl;
            run = false;
        }

        // Start Sensor and Processing
        dwCameraProperties cameraProperties;
        dwSensorCamera_getSensorProperties(&cameraProperties, cameraSensor);

        run = dwSensor_start(cameraSensor) == DW_SUCCESS;

        // Message msg;
        while (run) {
            dwCameraFrameHandle_t frameHandle;
            dwImageNvMedia *frame = nullptr;
            uint32_t camera = 0u;
            dwStatus status = dwSensorCamera_readFrame(&frameHandle, camera, timeout, cameraSensor);
            if (status != DW_SUCCESS) {
                std::cout << "\n ERROR readFrame: " << dwGetStatusName(status) << std::endl;
                continue;
            }

            status = dwSensorCamera_getImageNvMedia(&frame, DW_CAMERA_PROCESSED_IMAGE, frameHandle);

            if (status != DW_SUCCESS) {
                std::cout << "\n ERROR get nvmedia: " << dwGetStatusName(status) << std::endl;
                continue;
            }

            status = dwImageStreamer_postNvMedia(frame, streamerNVM2CUDA);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot post nvmedia image streamer: " << dwGetStatusName(status) << std::endl;
                run = false;
            }

            dwImageNvMedia *rgbaNVMBack;
            status = dwImageStreamer_waitPostedNvMedia(&rgbaNVMBack, timeout, streamerNVM2CUDA);
            if (status != DW_SUCCESS) {
                std::cout << "Cannot get back original image from image streamer: " << dwGetStatusName(status) << std::endl;
                run = false;
                std::cout << "Consumer must have disconnected, shutting down." << std::endl;
            }

            status = dwSensorCamera_returnFrame(&frameHandle);
            if (status != DW_SUCCESS) {
                std::cerr << "Cannot return frame: " << dwGetStatusName(status) << std::endl;
                run = false;
            }
        }

        // wait for child to consume
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // release everything
        dwImageStreamer_release(&streamerNVM2CUDA);
        dwSAL_releaseSensor(&cameraSensor);
        dwSAL_release(&sal);

    } else {
        // CHILD consumer
        std::cout << "IPC child PID: " << getpid() << std::endl;

        window = new WindowGLFW(WINDOW_WIDTH, WINDOW_HEIGHT);

        if (window == nullptr) {
            std::cerr << "Cannot init window" << std::endl;
            run = false;
        }

        window->makeCurrent();
        window->setOnKeypressCallback(processKey);

        dwRendererHandle_t renderer;
        status = dwRenderer_initialize(&renderer, sdk);
        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init renderer: " << dwGetStatusName(status) << std::endl;
            run = false;
        }

        dwRect screenRectangle;
        screenRectangle.height = WINDOW_HEIGHT;
        screenRectangle.width = WINDOW_WIDTH;
        screenRectangle.x = 0;
        screenRectangle.y = 0;
        status = dwRenderer_setRect(screenRectangle, renderer);
        if (status != DW_SUCCESS) {
            std::cerr << "Cannot set render rectangle: " << dwGetStatusName(status) << std::endl;
            run = false;
        }

        dwImageProperties props{};
        props.type = DW_IMAGE_NVMEDIA;
        props.width = width;
        props.height = height;
        props.planeCount = 3;
        props.pxlFormat = DW_IMAGE_YUV420;
        props.pxlType = DW_TYPE_UINT8;

        std::cout << "Initializing cross process streamer, waiting for producer..." << std::endl;

        auto streamerNVM2CUDA = dwImageStreamerHandle_t{};
        status = dwImageStreamer_initializeCrossProcess(&streamerNVM2CUDA,
                                                        &props, DW_IMAGE_CUDA,
                                                        {SOCK_NAME, DW_IMAGE_STREAMER_CROSS_PROCESS_CONSUMER},
                                                        sdk);

        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init cross proc: " << dwGetStatusName(status) << std::endl;
            run = false;
        }

        dwImageProperties propsCUDA = props;
        propsCUDA.type = DW_IMAGE_CUDA;

        dwImageProperties propsCUDAconv = propsCUDA;
        propsCUDAconv.pxlFormat = DW_IMAGE_RGBA;
        propsCUDAconv.planeCount = 1;

        dwImageStreamerHandle_t streamerCUDA2GL;
        status = dwImageStreamer_initialize(&streamerCUDA2GL, &propsCUDAconv, DW_IMAGE_GL, sdk);

        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init gl streamer: " << dwGetStatusName(status) << std::endl;
            run = false;
        }

        dwImageCUDA imgCUDAConv{};
        status = dwImageCUDA_create(&imgCUDAConv, &propsCUDAconv, DW_IMAGE_CUDA_PITCH);

        if (status != DW_SUCCESS) {
            std::cerr << "Cannot create cuda image: " << dwGetStatusName(status) << std::endl;
            run = false;
        }

        dwImageFormatConverterHandle_t converter;
        status = dwImageFormatConverter_initialize(&converter, propsCUDA.type, sdk);

        if (status != DW_SUCCESS) {
            std::cerr << "Cannot init converter: " << dwGetStatusName(status) << std::endl;
            run = false;
        }

        // receive images
        while (run && gRun) {
            dwImageCUDA *cudaImage;
            status = dwImageStreamer_receiveCUDA(&cudaImage, timeout, streamerNVM2CUDA);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot receive cuda image streamer: " << dwGetStatusName(status) << std::endl;
                run = false;
                continue;
            }

            status = dwImageFormatConverter_copyConvertCUDA(&imgCUDAConv, cudaImage, converter, 0);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot convert: " << dwGetStatusName(status) << std::endl;
                run = false;
            }

            status = dwImageStreamer_postCUDA(&imgCUDAConv, streamerCUDA2GL);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot post cuda image streamer: " << dwGetStatusName(status) << std::endl;
                run = false;
            }

            dwImageGL *glImage;
            status = dwImageStreamer_receiveGL(&glImage, 1000, streamerCUDA2GL);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot receive gl image streamer: " << dwGetStatusName(status) << std::endl;
                run = false;
            }


            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            status = dwRenderer_renderTexture(glImage->tex, glImage->target, renderer);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot render texture: " << dwGetStatusName(status) << std::endl;
                run = false;
            }

            // GL swapBuffer will draw on screen
            window->swapBuffers();
            CHECK_GL_ERROR();

            status = dwImageStreamer_returnReceivedGL(glImage, streamerCUDA2GL);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot return gl image streamer: " << dwGetStatusName(status) << std::endl;
                run = false;
            }

            dwImageCUDA *cudaBack;
            status = dwImageStreamer_waitPostedCUDA(&cudaBack, timeout, streamerCUDA2GL);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot wait post cuda image streamer: " << dwGetStatusName(status) << std::endl;
                run = false;
            }

            status = dwImageStreamer_returnReceivedCUDA(cudaImage, streamerNVM2CUDA);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot return: " << dwGetStatusName(status) << std::endl;
                run = false;
            }
        }

        dwImageCUDA_destroy(&imgCUDAConv);
        dwImageFormatConverter_release(&converter);

        // wait for parent to consume
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // release streamer and sdk
        dwImageStreamer_release(&streamerNVM2CUDA);
        dwRenderer_release(&renderer);
        dwRelease(&sdk);
        dwLogger_release();
        delete window;
        window = nullptr;
    }
    //////////////////////

    dwRelease(&sdk);
    dwLogger_release();
    delete window;
    window = nullptr;

    return 0;
}
