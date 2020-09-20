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

#include <iostream>
#include <thread>
#include <cstring>

SensorIONvmedia::SensorIONvmedia(dwContextHandle_t context, cudaStream_t stream, uint32_t sensorCount, dwSensorHandle_t cameraSensor[], int cameraWidth, int cameraHeight)
    : m_cudaStream(stream)
    , m_sensorCount(sensorCount)
    , m_sensors(sensorCount)
    , m_yuv2rgba(sensorCount, DW_NULL_HANDLE)
    , m_nvm2gl(sensorCount, DW_NULL_HANDLE)
    , m_nvm2cuda(sensorCount, DW_NULL_HANDLE)
    , m_frame(sensorCount)
    , m_frameNvmYuv(sensorCount)
    , m_frameCUDAyuv(sensorCount)
    , m_frameGlRgba(sensorCount)
{
    dwStatus result = DW_FAILURE;

    result = dwContext_getNvMediaDevice(&m_nvmedia, context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Error intializing nvmedia: ") + dwGetStatusName(result));

    for (uint32_t k = 0; k < sensorCount; k++) {
        m_sensors[k] = cameraSensor[k];

        // format converter
        dwImageProperties cameraImageProperties;
        dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, m_sensors[k]);
        dwImageProperties displayImageProperties = cameraImageProperties;
        displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
        displayImageProperties.planeCount = 1;

        result = dwImageFormatConverter_initialize(&m_yuv2rgba[k], cameraImageProperties.type, context);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot create pixel format converter : yuv->rgba: ") + dwGetStatusName(result));

        // image API translator
        result = dwImageStreamer_initialize(&m_nvm2gl[k], &displayImageProperties, DW_IMAGE_GL, context);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot init image streamer: ") + dwGetStatusName(result));

        result = dwImageStreamer_initialize(&m_nvm2cuda[k], &cameraImageProperties, DW_IMAGE_CUDA, context);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot init image streamer nvm2cuda ") + dwGetStatusName(result));
        dwImageStreamer_setCUDAStream(m_cudaStream, m_nvm2cuda[k]);
    }

    // RGBA image pool for conversion from YUV camera output
    for (uint32_t i = 0; i < sensorCount * POOL_SIZE; ++i) {
        dwImageNvMedia *rgbaImage = new dwImageNvMedia();

        dwImageProperties properties{};
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
    //Release all
    for (uint32_t k = 0; k < m_sensorCount; k++) {
        dwImageStreamer_release(&m_nvm2gl[k]);
        dwImageStreamer_release(&m_nvm2cuda[k]);
        dwImageFormatConverter_release(&m_yuv2rgba[k]);
    }

    for (auto &image : m_rgbaImagePool) {
        NvMediaImageDestroy(image->img);
        delete image;
    }
}

dwStatus SensorIONvmedia::getFrame(int siblingIdx)
{
    // try different image types
    dwStatus result = DW_FAILURE;
    result = dwSensorCamera_readFrame(&m_frame[siblingIdx], 0, 1000000, m_sensors[siblingIdx]);
    if (result != DW_SUCCESS)
        return result;

    result = dwSensorCamera_getImageNvMedia(&m_frameNvmYuv[siblingIdx], DW_CAMERA_PROCESSED_IMAGE, m_frame[siblingIdx]);
    if (result != DW_SUCCESS)
        return result;

    m_timestamp_us = m_frameNvmYuv[siblingIdx]->timestamp_us;

    // conversion
    if (m_rgbaImagePool.size() > 0) {
        dwImageNvMedia *rgbaImage = m_rgbaImagePool.back();
        m_rgbaImagePool.pop_back();

        // RGBA version of the frame
        result = dwImageFormatConverter_copyConvertNvMedia(rgbaImage, m_frameNvmYuv[siblingIdx], m_yuv2rgba[siblingIdx]);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot convert frame: " << dwGetStatusName(result) << std::endl;
            //continue;
        }

        // Send the RGBA frame
        //std::cout << "post NvMedia "
        // << frameNvMediayuv->prop.timestamp_us
        // << " : " << frameNvMediayuv << std::endl;
        result = dwImageStreamer_postNvMedia(rgbaImage, m_nvm2gl[siblingIdx]);
        if (result != DW_SUCCESS) {
            std::cerr << "cannot post NvMedia frame " << dwGetStatusName(result) << std::endl;
        }
    }

    // Send the YUV frame
    result = dwImageStreamer_postNvMedia(m_frameNvmYuv[siblingIdx], m_nvm2cuda[siblingIdx]);
    if (result != DW_SUCCESS) {
        std::cerr << "cannot post NvMedia frame " << dwGetStatusName(result) << std::endl;
    }

    return DW_SUCCESS;
}

dwImageCUDA *SensorIONvmedia::getCudaYuv(int siblingIdx)
{
    dwStatus result;
    result = dwImageStreamer_receiveCUDA(&m_frameCUDAyuv[siblingIdx], 60000, m_nvm2cuda[siblingIdx]);

    if (result != DW_SUCCESS || m_frameCUDAyuv[siblingIdx] == 0) {
        std::cerr << "did not received CUDA frame within 60ms" << std::endl;
        m_frameCUDAyuv[siblingIdx] = nullptr;
    }
    return m_frameCUDAyuv[siblingIdx];
}

void SensorIONvmedia::releaseCudaYuv(int siblingIdx)
{
    dwStatus result;
    result = dwImageStreamer_returnReceivedCUDA(m_frameCUDAyuv[siblingIdx], m_nvm2cuda[siblingIdx]);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot release cudaYuv: ") + dwGetStatusName(result));
}

dwImageGL *SensorIONvmedia::getGlRgbaFrame(int siblingIdx)
{
    dwStatus result;
    result = dwImageStreamer_receiveGL(&m_frameGlRgba[siblingIdx], 60000, m_nvm2gl[siblingIdx]);

    if (result != DW_SUCCESS) {
        std::cerr << "did not received GL frame within 30ms" << std::endl;
        m_frameGlRgba[siblingIdx] = nullptr;
    }
    return m_frameGlRgba[siblingIdx];
}

void SensorIONvmedia::releaseGlRgbaFrame(int siblingIdx)
{
    dwStatus result;

    if (m_frameGlRgba[siblingIdx]) {
        result = dwImageStreamer_returnReceivedGL(m_frameGlRgba[siblingIdx], m_nvm2gl[siblingIdx]);
        if (result != DW_SUCCESS)
            throw std::runtime_error(std::string("Cannot return received GL: ") + dwGetStatusName(result));
        m_frameGlRgba[siblingIdx] = nullptr;
    }
}

void SensorIONvmedia::releaseFrame(int siblingIdx)
{
    dwStatus result;
    // any image returned back, we put back into the pool
    {
        dwImageNvMedia *retimg = nullptr;
        result = dwImageStreamer_waitPostedNvMedia(&retimg, 33000, m_nvm2cuda[siblingIdx]);
        if (result == DW_SUCCESS && retimg) {
            if(retimg != m_frameNvmYuv[siblingIdx]) {
                //Note: the egl stream will always keep one extra frame, so
                //      retimg will always be one frame behind.
                //std::cerr << "Error: Received wrong image from streamer\n";
            }
        } else if (result == DW_TIME_OUT) {
            //Note: the egl stream will always keep one extra frame.
            //      This will always time out for the first frame.
            //      Ignore first timeout
            static uint32_t timeoutCounter=0;
            timeoutCounter++;
            if(timeoutCounter > m_sensorCount) {
                std::cerr << "Warning: waitPosted (cuda) timed out more than once\n";
            }
        } else if (result != DW_SUCCESS) {
            std::cout << " ERROR waitPostedNvMedia: " << dwGetStatusName(result) << std::endl;
        }

        if(m_frameNvmYuv[siblingIdx]) {
            dwSensorCamera_returnFrame(&m_frame[siblingIdx]);
            m_frameNvmYuv[siblingIdx] = nullptr;
        }
    }
    {
        dwImageNvMedia *retimg = nullptr;
        result = dwImageStreamer_waitPostedNvMedia(&retimg, 33000, m_nvm2gl[siblingIdx]);
        if (result == DW_SUCCESS && retimg) {
            m_rgbaImagePool.push_back(retimg);
        } else if (result == DW_TIME_OUT) {
            //Note: the egl stream will always keep one extra frame.
            //      This will always time out for the first frame.
            //      Ignore first timeout
            static uint32_t timeoutCounter=0;
            timeoutCounter++;
            if(timeoutCounter > m_sensorCount) {
                std::cerr << "Warning: waitPosted (gl) timed out more than once\n";
            }
        } else if (result != DW_SUCCESS) {
            std::cout << " ERROR waitPostedNvMedia: " << dwGetStatusName(result) << std::endl;
        }
    }
}
