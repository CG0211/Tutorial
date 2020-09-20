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

#include "utils.hpp"
#include <cuda.h>

__global__ void kernel(uint8_t* image, size_t pitch, const uint32_t width, const uint32_t height,
                       const uint32_t val)
{
    const uint32_t tidx = blockDim.x * blockIdx.x + threadIdx.x;
    const uint32_t tidy = blockDim.y * blockIdx.y + threadIdx.y;

    if (tidx >= width || tidy >= height) return;

    image[tidy*pitch + 4*tidx+0] = (tidx + val) % 256;
    image[tidy*pitch + 4*tidx+1] = (tidy + val) % 256;
    image[tidy*pitch + 4*tidx+2] = (tidx + tidy + 2*val) % 256;
    image[tidy*pitch + 4*tidx+3] = 255;
}

uint32_t iDivUp(const uint32_t a, const uint32_t b)
{
    return ((a % b) != 0U) ? ((a / b) + 1U) : (a / b);
}

void runKernel(dwImageCUDA *image, const uint32_t val)
{
    dim3 numThreads = dim3(32, 4, 1);
    kernel <<<dim3(iDivUp(image->prop.width, numThreads.x),
                   iDivUp(image->prop.height, numThreads.y)),
            numThreads >>>(static_cast<uint8_t*>(image->dptr[0]), image->pitch[0], image->prop.width, image->prop.height, val);

}
