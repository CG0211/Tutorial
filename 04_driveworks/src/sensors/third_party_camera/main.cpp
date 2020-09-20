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

#include <framework/Log.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/SampleFramework.hpp>

#include <dw/core/Status.h>
#include <dw/core/Logger.h>
#include <dw/image/Image.h>
#include <dw/image/ImageStreamer.h>
#include <dw/sensors/Sensors.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

ProgramArguments arguments;

const uint32_t width = 1024;
const uint32_t height = 768;

// this class manages the resources that are necessary for this sample
class Resources {
public:
    Resources(
            int argc,
            const char* argv[]
    ) {
        initSampleApp(argc, argv, &arguments, nullptr, width, height);
        dwStatus res = dwLogger_initialize(getConsoleLoggerCallback(true));

        throwIfNonSuccess(res, "Could not instantiate Console Logger");

        dwContextParameters params{};

#ifdef VIBRANTE
        params.eglDisplay = gWindow->getEGLDisplay();
#endif

        res = dwInitialize(&m_sdkHandle, DW_VERSION, &params);
        throwIfNonSuccess(res, "Could not initialize DriveWorks");

        res = dwRenderer_initialize(&m_renderer, m_sdkHandle);
        throwIfNonSuccess(res, "Could not initialize renderer");

        dwRect rect;
        rect.width   = width;
        rect.height  = height;
        rect.x = 0;
        rect.y = 0;

        res = dwRenderer_setRect(rect, m_renderer);
        throwIfNonSuccess(res, "Could not set renderer ROI");

        float32_t rasterTransform[9];
        rasterTransform[0] = 1.0f;
        rasterTransform[3] = 0.0f;
        rasterTransform[6] = 0.0f;

        rasterTransform[1] = 0.0f;
        rasterTransform[4] = 1.0f;
        rasterTransform[7] = 0.0f;

        rasterTransform[2] = 0.0f;
        rasterTransform[5] = 0.0f;
        rasterTransform[8] = 1.0f;

        res = dwRenderer_set2DTransform(rasterTransform, m_renderer);
        throwIfNonSuccess(res, "Could not set renderer transform");

        res = dwRenderer_setPointSize(4.0f, m_renderer);
        throwIfNonSuccess(res, "Could not set renderer point size");

        res = dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer);
        throwIfNonSuccess(res, "Could not set renderer color");
    }

    ~Resources() {
        dwRenderer_release(&m_renderer);
        dwRelease(&m_sdkHandle);
    }

    dwContextHandle_t context() const {
        return m_sdkHandle;
    }

    dwRendererHandle_t renderer() const
    {
        return m_renderer;
    }

protected:
    void throwIfNonSuccess(const dwStatus status, const std::string& message) {
        if(DW_SUCCESS != status) {
            throw std::runtime_error(message);
        }
    }

private:
    dwContextHandle_t m_sdkHandle;
    dwRendererHandle_t m_renderer;

};

class ThirdPartyCamera {
public:
    struct Frame {
        Frame(uint32_t width_, uint32_t height_):
            width(width_),
            height(height_),
            data(width * height * 3) {
        }

        const uint32_t width;
        const uint32_t height;
        std::vector<uint8_t> data;

        Frame& operator=(const Frame& rhs) = delete;
    };

    ThirdPartyCamera(uint32_t width_, uint32_t height_):
        width(width_),
        height(height_),
        framePool(numberFramesInPool, Frame(width, height)),
        idx(0)
    {
        initializeFrames();
    }

    void initializeFrames() {
        for (size_t index = 0 ; index < numberFramesInPool ; ++index) {
            for (uint32_t j = 0 ; j < height ; j++) {
                for (uint32_t i = 0 ; i < width ; ++i) {
                    for(uint32_t ch = 0 ; ch < 3 ; ++ch) {
                        framePool[index].data[j * (3*width) + 3 * i + ch] =
                                static_cast<uint8_t>(static_cast<int32_t>(index * sqrt(width*width + height*height)/numberFramesInPool + i + j + ch) % 255);
                    }
                }
            }
        }
    }


    Frame* getNextFrame() {
        Frame *frame = &(framePool[idx]);
        idx = (idx + 1) % numberFramesInPool;
        return frame;
    }

protected:
    ThirdPartyCamera& operator=(const ThirdPartyCamera& rhs) = delete;


private:
    const uint32_t width;
    const uint32_t height;
    const size_t numberFramesInPool = 50;

    std::vector<Frame> framePool;
    size_t idx;
};


int main (int argc, const char* argv[]) {
    std::unique_ptr<Resources> res;

    try{
        res.reset(new Resources(argc, argv));
    } catch (const  std::exception& e) {
        std::cerr << "An exception has occurred while acquiring resources : " << std::endl;
        std::cerr << e.what() << std::endl;
        return -1;
    }


    dwImageStreamerHandle_t streamer = DW_NULL_HANDLE;
    dwImageProperties props{};
    props.type = DW_IMAGE_CPU;
    props.pxlType = DW_TYPE_UINT8;
    props.pxlFormat = DW_IMAGE_RGB;
    props.width = width;
    props.height = height;
    props.planeCount = 1;
    dwImageStreamer_initialize(&streamer,
                               &props,
                               DW_IMAGE_GL,
                               res->context());

    // create the user-provided sensor
    ThirdPartyCamera sensor(width, height);

    //timing to limit the framerate
    std::chrono::time_point<std::chrono::high_resolution_clock> nextFrame;
    std::chrono::milliseconds frameDuration{33};

    while(gRun) {
        std::this_thread::sleep_until(nextFrame);

        ThirdPartyCamera::Frame* frame = sensor.getNextFrame();

        // Wrap the frame in a DW container
        dwImageCPU cpuFrame{};

        dwImageProperties properties{};
        properties.width = width;
        properties.height = height;
        properties.planeCount = 1;
        properties.pxlType = DW_TYPE_UINT8;
        properties.pxlFormat = DW_IMAGE_RGB;
        properties.type = DW_IMAGE_CPU;
        cpuFrame.prop = properties;

        //timestamp the dwImageCPU, using the global time base.
        dwContext_getCurrentTime(&cpuFrame.timestamp_us, res->context());

        // assign the data pointer
        cpuFrame.data[0] = &(frame->data[0]);
        cpuFrame.pitch[0] = width * 3;

        // at this point, we can pass the dwImageCPU to dw for further processing
        // in this case, streaming and displaying.

        dwStatus result = dwImageStreamer_postCPU(&cpuFrame, streamer);
        if(DW_SUCCESS != result) {
            std::cerr << "Failed to post the CPU image" << std::endl;
            break;
        }

        dwImageGL* glFrame = nullptr;
        result = dwImageStreamer_receiveGL(&glFrame, 32000, streamer);
        if(DW_SUCCESS != result) {
            std::cerr << "Failed to receive the GL image" << std::endl;
            break;
        }

        result = dwRenderer_renderTexture(glFrame->tex, glFrame->target, res->renderer());
        if(DW_SUCCESS != result) {
            std::cerr << "Failed to render the texture iamge" << std::endl;
            break;
        }

        result = dwImageStreamer_returnReceivedGL(glFrame, streamer);
        if(DW_SUCCESS != result) {
            std::cerr << "Failed to return the GL Image to the streamer" << std::endl;
        }

        // the UserSensor does not need returning of its frame, but we need to reclaim
        // the resources from the streamer anyway
        dwImageCPU* returnedFrame;
        dwImageStreamer_waitPostedCPU(&returnedFrame, 32000, streamer);


        gWindow->swapBuffers();
        CHECK_GL_ERROR();

        nextFrame = std::chrono::high_resolution_clock::now() + frameDuration;
    }

    dwImageStreamer_release(&streamer);

    return 0;
}
