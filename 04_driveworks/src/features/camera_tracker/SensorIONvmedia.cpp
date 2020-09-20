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

#include "SensorIONvmedia.hpp"

#include <dw/sensors/camera/Camera.h>
#include <framework/Checks.hpp>

#include <iostream>
#include <thread>
#include <cstring>

SensorIONvmedia::SensorIONvmedia(dwContextHandle_t context, cudaStream_t stream,
                         dwSensorHandle_t cameraSensor, int cameraWidth, int cameraHeight)
    : m_cudaStream(stream)
    , m_sensor(cameraSensor)
    , m_yuv2rgba(DW_NULL_HANDLE)
    , m_nvm2gl(DW_NULL_HANDLE)
    , m_nvm2cuda(DW_NULL_HANDLE)
{
    dwStatus result = DW_FAILURE;

    dwContext_getNvMediaDevice(&m_nvmedia, context);

    // format converter
    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensor);

    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
    displayImageProperties.planeCount = 1;
    result = dwImageFormatConverter_initialize(&m_yuv2rgba, cameraImageProperties.type, context);
    if (result != DW_SUCCESS) {
        throw std::runtime_error("Cannot create pixel format converter : yuv->rgba");
    }

    // image API translator
    result = dwImageStreamer_initialize(&m_nvm2gl, &displayImageProperties, DW_IMAGE_GL, context);
    if (result != DW_SUCCESS) {
        throw std::runtime_error(std::string("Cannot init image streamer nvm-gl: ") + dwGetStatusName(result));
    }

    result = dwImageStreamer_initialize(&m_nvm2cuda, &cameraImageProperties, DW_IMAGE_CUDA, context);
    if (result != DW_SUCCESS) {
        throw std::runtime_error(std::string("Cannot init image streamer nvm-cuda: ") + dwGetStatusName(result));
    }
    dwImageStreamer_setCUDAStream(m_cudaStream, m_nvm2cuda);

    // RGBA image pool for conversion from YUV camera output
    for (int i = 0; i < POOL_SIZE; ++i) {
        dwImageNvMedia* rgbaImage = new dwImageNvMedia();

        dwImageProperties properties = cameraImageProperties;
        properties.type = DW_IMAGE_NVMEDIA;
        properties.pxlFormat = DW_IMAGE_RGBA;
        properties.pxlType = DW_TYPE_UINT8;
        properties.planeCount = 1;
        properties.width = cameraWidth;
        properties.height = cameraHeight;
        dwImageNvMedia_create(rgbaImage, &properties, context);

        m_rgbaImagePool.push_back(rgbaImage);
    }
}

SensorIONvmedia::~SensorIONvmedia()
{
    if (m_nvm2gl != DW_NULL_HANDLE)
        dwImageStreamer_release(&m_nvm2gl);
    if (m_nvm2cuda != DW_NULL_HANDLE)
        dwImageStreamer_release(&m_nvm2cuda);
    if (m_yuv2rgba != DW_NULL_HANDLE)
        dwImageFormatConverter_release(&m_yuv2rgba);

    //Release image pool
    for (auto &image : m_rgbaImagePool) {
        NvMediaImageDestroy(image->img);
        delete image;
    }
}

dwStatus SensorIONvmedia::getFrame()
{
    // try different image types
    dwStatus result = DW_FAILURE;
    result = dwSensorCamera_readFrame(&m_frameHandle, 0, 1000000, m_sensor);
    if (result != DW_SUCCESS)
        return result;

    result = dwSensorCamera_getImageNvMedia(&m_frameNvmYuv,DW_CAMERA_PROCESSED_IMAGE, m_frameHandle);
    if (result != DW_SUCCESS)
        return result;

    // conversion
    if (m_rgbaImagePool.size() > 0) {
        dwImageNvMedia *rgbaImage = m_rgbaImagePool.back();
        m_rgbaImagePool.pop_back();

        // RGBA version of the frame
        CHECK_DW_ERROR( dwImageFormatConverter_copyConvertNvMedia(rgbaImage, m_frameNvmYuv, m_yuv2rgba) );

        // Send the RGBA frame
        CHECK_DW_ERROR( dwImageStreamer_postNvMedia(rgbaImage, m_nvm2gl) );
    }

    // Send the YUV frame
    CHECK_DW_ERROR( dwImageStreamer_postNvMedia(m_frameNvmYuv, m_nvm2cuda) );

    return DW_SUCCESS;
}

dwImageCUDA *SensorIONvmedia::getCudaYuv()
{
    dwStatus result;
    result = dwImageStreamer_receiveCUDA(&m_frameCUDAyuv, 60000, m_nvm2cuda);

    if (result != DW_SUCCESS || m_frameCUDAyuv == 0) {
        std::cerr << "did not received CUDA frame within 60ms" << std::endl;
        m_frameCUDAyuv = nullptr;
    }
    return m_frameCUDAyuv;
}

void SensorIONvmedia::releaseCudaYuv()
{
    if (m_frameCUDAyuv) {
        dwImageStreamer_returnReceivedCUDA(m_frameCUDAyuv, m_nvm2cuda);
        m_frameCUDAyuv = nullptr;
    }
}

dwImageGL *SensorIONvmedia::getGlRgbaFrame()
{
    dwStatus result;
    result = dwImageStreamer_receiveGL(&m_frameGlRgba, 60000, m_nvm2gl);

    if (result != DW_SUCCESS) {
        std::cerr << "did not received GL frame within 30ms" << std::endl;
        m_frameGlRgba = nullptr;
    }
    return m_frameGlRgba;
}

void SensorIONvmedia::releaseGLRgbaFrame()
{
    if (m_frameGlRgba) {
        dwImageStreamer_returnReceivedGL(m_frameGlRgba, m_nvm2gl);
        m_frameGlRgba = nullptr;
    }
}

void SensorIONvmedia::releaseFrame()
{
    dwTime_t WAIT_TIME = 33000;
    dwStatus result;
    // any image returned back, we put back into the pool
    {
        dwImageNvMedia *retimg = nullptr;
        result = dwImageStreamer_waitPostedNvMedia(&retimg, WAIT_TIME, m_nvm2cuda);
        if (result == DW_SUCCESS && retimg) {
            if(retimg != m_frameNvmYuv) {
                std::cerr << "Error: Received wrong image from streamer\n";
            }
        } else if (result == DW_TIME_OUT) {
            std::cerr << "Warning: waitPosted (cuda) timed out\n";
        } else if (result != DW_SUCCESS) {
            std::cout << " ERROR waitPostedNvMedia: " << dwGetStatusName(result) << std::endl;
        }
        if(m_frameNvmYuv) {
            dwSensorCamera_returnFrame(&m_frameHandle);
        }
    }
    {
        dwImageNvMedia *retimg = nullptr;
        result = dwImageStreamer_waitPostedNvMedia(&retimg, WAIT_TIME, m_nvm2gl);
        if (result == DW_SUCCESS && retimg) {
            m_rgbaImagePool.push_back(retimg);
        } else if (result == DW_TIME_OUT) {
            std::cerr << "Warning: waitPosted (gl) timed out\n";
        } else if (result != DW_SUCCESS) {
            std::cout << " ERROR waitPostedNvMedia: " << dwGetStatusName(result) << std::endl;
        }
    }
}
