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

#include <dw/image/Image.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>

#include <framework/SampleFramework.hpp>

#include "ResourceManager.hpp"

ProgramArguments arguments = ProgramArguments({
        ProgramArguments::Option_t("device", "0"),
    });

int main(int argc, const char *argv[])
{
    ResourceManager resources;
    if (DW_SUCCESS != resources.initializeResources(argc, argv, &arguments, nullptr)) {
        std::cerr << "Sample cannot acquire resources, aborting" << std::endl;
        return -1;
    }

    //initialize the sensor and retrieve camera dimensions.
    dwSensorHandle_t cameraSensor = DW_NULL_HANDLE;
    {
        dwSensorParams params;
        params.protocol = "camera.pointgrey";

        std::string parameters = "device=" + resources.getArgument("device");
        params.parameters      = parameters.c_str();

        dwStatus result = dwSAL_createSensor(&cameraSensor, params, resources.getSAL());
        if (result != DW_SUCCESS)
        {
            std::cerr << "Cannot open camera.";
            return -1;
        }
    }

    dwImageProperties cameraProperties{};

    CHECK_DW_ERROR( dwSensorCamera_getImageProperties(&cameraProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensor) );

    std::cout << "Camera image with " << cameraProperties.width << "x" << cameraProperties.height << std::endl;

    // sets the window size now that we know what the camera dimensions are.
    resources.setWindowSize(cameraProperties.width, cameraProperties.height);


    // instantiation of an image streamer that can pass cpu images to OpenGL.
    dwImageStreamerHandle_t cpu2gl = DW_NULL_HANDLE;
    if (DW_SUCCESS != dwImageStreamer_initialize(&cpu2gl,
                                                 &cameraProperties, DW_IMAGE_GL,
                                                 resources.getSDK())) {
        std::cerr << "Could not initialize Image Streamer" << std::endl;
    }

    dwCameraFrameHandle_t frameHandle = DW_NULL_HANDLE;
    dwImageCPU *imageCPU = nullptr;
    dwImageGL *imageGL = nullptr;

    // start the sensor. The underlying structure will begin filling it's internal
    // buffer as images become available.
    if (DW_SUCCESS != dwSensor_start(cameraSensor))
    {
        std::cerr << "Could not start sensor" <<std::endl;
    }

    while (gRun && !gWindow->shouldClose()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // webcams can deliver as low as 20 fps so we need a timeout high enough
        const dwStatus result = dwSensorCamera_readFrame(&frameHandle, 0, 500000, cameraSensor);
        if (DW_NOT_AVAILABLE == result) {
            std::cerr << "Camera is not running or not found" << std::endl;
            break;
        }

        if(DW_SUCCESS != dwSensorCamera_getImageCPU(&imageCPU, DW_CAMERA_PROCESSED_IMAGE, frameHandle)) {
            continue;
        }

        // sends the CPU image on the stream
        if(DW_SUCCESS != dwImageStreamer_postCPU(imageCPU, cpu2gl)) {
            std::cerr << "Failed to post the image on the CPU -> OpenGL stream" << std::endl;
            continue;
        }

        // and waits for the GL image to come out.
        if (DW_SUCCESS != dwImageStreamer_receiveGL(&imageGL, 30000, cpu2gl)) {
            std::cerr << "Failed to get an OpenGL image in 30 ms" << std::endl;
            continue;
        }

        if(imageGL) {
            if (DW_SUCCESS != dwRenderer_renderTexture(imageGL->tex, imageGL->target, resources.getRenderer())) {
                std::cerr << "Error during texture rendering" << std::endl;
            }

            // returning the GL image to its stream.
            if (DW_SUCCESS != dwImageStreamer_returnReceivedGL(imageGL, cpu2gl)) {
                std::cerr << "Failure to return GL frame to streamer" << std::endl;
            }

            // and waiting for the CPU image to be returned to us.
            dwImageCPU *frame;
            if (DW_SUCCESS != dwImageStreamer_waitPostedCPU(&frame, 32000, cpu2gl)
                    || frame != imageCPU) {
                std::cerr << "Failed to get back the rgba frame from consumer" << std::endl;
            }
        }
        // the sensor-issued image is no longer useful and can be returned.
        if (DW_SUCCESS != dwSensorCamera_returnFrame(&frameHandle)) {
            std::cerr << "Failure to return frame to sensor" << std::endl;
        }

        gWindow->swapBuffers();
        CHECK_GL_ERROR();
    }

    if (DW_SUCCESS != dwSensor_stop(cameraSensor)) {
        std::cerr << "Could not stop the sensor" << std::endl;
    }

    if (DW_SUCCESS != dwImageStreamer_release(&cpu2gl)) {
        std::cerr << "Failure to release ImageStreamer" << std::endl;
    }

    dwSAL_releaseSensor(&cameraSensor);

    return 0;
}
