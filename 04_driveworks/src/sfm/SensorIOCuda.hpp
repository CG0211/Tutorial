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

#ifndef SAMPLES_SFM_TRIANGULATION_SENSORIOCUDA_HPP__
#define SAMPLES_SFM_TRIANGULATION_SENSORIOCUDA_HPP__

#include "ISensorIO.hpp"

#include <dw/sensors/camera/Camera.h>
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>

#include <vector>

class SensorIOCuda : public ISensorIO
{
  public:
    SensorIOCuda(dwContextHandle_t context, cudaStream_t stream, uint32_t sensorCount, dwSensorHandle_t cameraSensor[], int cameraWidth, int cameraHeight);
    ~SensorIOCuda();

    dwStatus getFrame(int siblingIdx) override final;

    dwImageCUDA *getCudaYuv(int siblingIdx) override final;
    void releaseCudaYuv(int siblingIdx) override final;

    dwImageGL *getGlRgbaFrame(int siblingIdx) override final;
    void releaseGlRgbaFrame(int siblingIdx) override final;

    void releaseFrame(int siblingIdx) override final;

  protected:
    //////////////////////////
    // Configuration members
    cudaStream_t m_cudaStream;

    uint32_t m_sensorCount;
    std::vector<dwSensorHandle_t> m_sensors;

    std::vector<dwImageFormatConverterHandle_t> m_yuv2rgba;
    std::vector<dwImageStreamerHandle_t> m_cuda2gl;

    static const uint32_t POOL_SIZE = 4;
    std::vector<dwImageCUDA *> m_rgbaImagePool;

    ////////////////////////
    // Current frame
    std::vector<dwCameraFrameHandle_t> m_frame;
    std::vector<dwImageCUDA *> m_frameCUDAyuv;
    std::vector<dwImageGL *> m_frameGlRgba;
};

#endif // SAMPLES_SFM_TRIANGULATION_SENSORIOCUDA_HPP__
