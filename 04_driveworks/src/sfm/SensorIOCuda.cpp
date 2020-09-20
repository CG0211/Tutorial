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

#include "SensorIOCuda.hpp"

#include <dw/sensors/camera/Camera.h>
#include <framework/Checks.hpp>

#include <iostream>
#include <thread>

SensorIOCuda::SensorIOCuda(dwContextHandle_t context, cudaStream_t stream, uint32_t sensorCount, dwSensorHandle_t cameraSensor[], int cameraWidth, int cameraHeight)
    : m_cudaStream(stream)
    , m_sensorCount(sensorCount)
    , m_sensors(sensorCount)
    , m_yuv2rgba(sensorCount, DW_NULL_HANDLE)
    , m_cuda2gl(sensorCount, DW_NULL_HANDLE)
    , m_frame(sensorCount, DW_NULL_HANDLE)
    , m_frameCUDAyuv(sensorCount, DW_NULL_HANDLE)
    , m_frameGlRgba(sensorCount, DW_NULL_HANDLE)
{
    for (uint32_t k = 0; k < sensorCount; k++) {
        m_sensors[k] = cameraSensor[k];

        // format converter for GL display
        dwImageProperties cameraImageProperties;
        dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, m_sensors[k]);
        dwImageProperties displayImageProperties = cameraImageProperties;
        displayImageProperties.pxlFormat = DW_IMAGE_RGBA;
        displayImageProperties.planeCount = 1;

        CHECK_DW_ERROR( dwImageFormatConverter_initialize(&m_yuv2rgba[k], cameraImageProperties.type, context) );

        // image API translator
        CHECK_DW_ERROR( dwImageStreamer_initialize(&m_cuda2gl[k], &displayImageProperties, DW_IMAGE_GL, context) );
        dwImageStreamer_setCUDAStream(m_cudaStream, m_cuda2gl[k]);
    }

    // CUDA RGBA image pool
    for (uint32_t i = 0; i < sensorCount * POOL_SIZE; ++i) {
        dwImageCUDA *frameCUDArgba = new dwImageCUDA;
        {
            void *dptr   = nullptr;
            size_t pitch = 0;
            cudaError_t error = cudaMallocPitch(&dptr, &pitch, cameraWidth * 4, cameraHeight);
            if (error != cudaSuccess) {
                std::cerr << "SensorIOCUDA ERROR cannot allocate more, POOL_SIZE too large." << std::endl;
                break;
            }
            dwImageCUDA_setFromPitch(frameCUDArgba, dptr, cameraWidth,
                                     cameraHeight, pitch, DW_IMAGE_RGBA);
        }

        m_rgbaImagePool.push_back(frameCUDArgba);
    }
}

SensorIOCuda::~SensorIOCuda()
{
    for (uint32_t k = 0; k < m_sensorCount; k++) {
        dwImageStreamer_release(&m_cuda2gl[k]);
        dwImageFormatConverter_release(&m_yuv2rgba[k]);
    }

    //Release image pool
    for (auto &image : m_rgbaImagePool) {
        cudaFree(image->dptr[0]);
        delete image;
    }
}

dwStatus SensorIOCuda::getFrame(int siblingIdx)
{
    // try different image types
    dwStatus result = DW_FAILURE;
    result = dwSensorCamera_readFrame(&m_frame[siblingIdx], 0, 1000000, m_sensors[siblingIdx]);
    if (result != DW_SUCCESS) {
        return result;
    }

    result = dwSensorCamera_getImageCUDA(&m_frameCUDAyuv[siblingIdx], DW_CAMERA_PROCESSED_IMAGE,
        m_frame[siblingIdx]);
    if (result != DW_SUCCESS) {
        m_frameCUDAyuv[siblingIdx] = nullptr;
        return result;
    }

    m_timestamp_us = m_frameCUDAyuv[siblingIdx]->timestamp_us;

    // CUDA copy conversion
    if (m_rgbaImagePool.size() > 0) {
        dwImageCUDA *rgbaImage;

        rgbaImage = m_rgbaImagePool.back();
        m_rgbaImagePool.pop_back();

        // convert CUDA YUV image to RGBA
        result = dwImageFormatConverter_copyConvertCUDA(rgbaImage,
                                                        m_frameCUDAyuv[siblingIdx],
                                                        m_yuv2rgba[siblingIdx],
                                                        m_cudaStream);
        if (result != DW_SUCCESS) {
            std::cerr << "cannot convert frame YUV to RGBA" << dwGetStatusName(result) << std::endl;
        }

        //std::cout << "post CUDA " << rgbaImage->prop.timestamp_us << " : " << rgbaImage << std::endl;
        result = dwImageStreamer_postCUDA(rgbaImage, m_cuda2gl[siblingIdx]);
        if (result != DW_SUCCESS) {
            std::cerr << "cannot post CUDA RGBA image" << dwGetStatusName(result) << std::endl;
        }
    }

    return DW_SUCCESS;
}

dwImageCUDA *SensorIOCuda::getCudaYuv(int siblingIdx)
{
    (void)siblingIdx;
    return m_frameCUDAyuv[siblingIdx];
}

void SensorIOCuda::releaseCudaYuv(int siblingIdx)
{
    dwSensorCamera_returnFrame(&m_frame[siblingIdx]);
}

dwImageGL *SensorIOCuda::getGlRgbaFrame(int siblingIdx)
{
    (void)siblingIdx;

    if (dwImageStreamer_receiveGL(&m_frameGlRgba[siblingIdx], 30000, m_cuda2gl[siblingIdx]) != DW_SUCCESS) {
        std::cerr << "did not received GL frame within 30ms" << std::endl;
        m_frameGlRgba[siblingIdx] = nullptr;
    }
    return m_frameGlRgba[siblingIdx];
}

void SensorIOCuda::releaseGlRgbaFrame(int siblingIdx)
{
    (void)siblingIdx;

    if (m_frameGlRgba[siblingIdx]) {
        dwImageStreamer_returnReceivedGL(m_frameGlRgba[siblingIdx], m_cuda2gl[siblingIdx]);
        m_frameGlRgba[siblingIdx] = nullptr;
    }
}

void SensorIOCuda::releaseFrame(int siblingIdx)
{
    (void)siblingIdx;

    dwStatus result;
    dwImageCUDA *retimg = nullptr;
    result = dwImageStreamer_waitPostedCUDA(&retimg, 33000, m_cuda2gl[siblingIdx]);
    if (result == DW_SUCCESS && retimg) {
        m_rgbaImagePool.push_back(retimg);
    }
}
