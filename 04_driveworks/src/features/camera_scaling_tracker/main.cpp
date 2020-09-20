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
// Copyright (c) 2016-2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
///////////////////////////////////////////////////////////////////////////////////////


#include <framework/DriveWorksSample.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/WindowGLFW.hpp>

#include <dw/renderer/Renderer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/FormatConverter.h>
#include <dw/image/Image.h>
#include <dw/image/ImageStreamer.h>
#include <dw/features/ScalingFeatureTracker.h>

#include <atomic>

using namespace dw_samples::common;
using namespace dw::common;

//------------------------------------------------------------------------------
// Camera Scaling Tracker
// The Camera Scaling Tracker sample demonstrates the scaling feature
// tracking capabilities of the dw_features module. It loads a video stream and
// reads the images sequentially. For each frame, it tracks scaling features from the
// previous frame. It doesn't detect new features, when there's no scaling features
// in the frame, the video replay will be paused automatically, you can use
// mouse to drag the boxes to track and press space to start replay/tracking.
//------------------------------------------------------------------------------
class CameraScalingFeatureTracker : public DriveWorksSample
{
private:

    static constexpr size_t IMAGE_BUFFER_SIZE = 4;

    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t               context             = DW_NULL_HANDLE;
    dwSALHandle_t                   sal                 = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwRendererHandle_t              renderer            = DW_NULL_HANDLE;
    dwSensorHandle_t                camera              = DW_NULL_HANDLE;
    dwImageStreamerHandle_t         image2GL            = DW_NULL_HANDLE;
    dwImageFormatConverterHandle_t  formatConverter     = DW_NULL_HANDLE;
    dwCameraProperties              cameraProps         = {};
    dwImageProperties               cameraImageProps    = {};
    dwCameraFrameHandle_t           frame               = DW_NULL_HANDLE;

    dwImageGL                      *frameGL = nullptr;
    dwImageCUDA                    *frameCudaYuv = nullptr;
    std::vector<dwImageCUDA*>       cudaPool;
#ifdef VIBRANTE
    dwImageNvMedia                 *frameNvmYuv = nullptr;
    dwImageStreamerHandle_t         nvm2cuda            = DW_NULL_HANDLE;
    std::vector<dwImageNvMedia*>    nvmediaPool;
#endif

    // tracker handles
    uint32_t                        maxFeatureCount = 100;
    dwScalingFeatureListHandle_t    featureList;
    dwScalingFeatureTrackerHandle_t tracker;
    dwRenderBufferHandle_t          featureRenderBuffer;

    // Buffers used to select features for list compacting
    uint32_t                       *d_validFeatureCount;
    uint32_t                       *d_validFeatureIndexes;
    uint32_t                       *d_invalidFeatureCount;
    uint32_t                       *d_invalidFeatureIndexes;

    //These point into the buffers of featureList
    size_t                          featureDataSize;
    void                           *d_featureDataBase;
    std::unique_ptr<uint8_t[]>      featureDataBuffer;
    dwScalingFeatureListPointers    featureData;

    std::vector<dwBox2D>            newBoxToTrack;
    std::vector<dwBox2D>            trackedBoxes;
    std::atomic<bool>               updateNewBox;

public:

    CameraScalingFeatureTracker(const ProgramArguments& args) : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize Renderer, Sensors, Image Streamers and Tracker
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            dwLogger_initialize(getConsoleLoggerCallback(true));
            dwLogger_setLogLevel(DW_LOG_VERBOSE);

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};
            sdkParams.dataPath = DataPath::get_cstr();

            #ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
            #endif

            dwInitialize(&context, DW_VERSION, &sdkParams);
            dwSAL_initialize(&sal, context);
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            CHECK_DW_ERROR_MSG(dwRenderer_initialize(&renderer, context),
                               "Cannot initialize Renderer, maybe no GL context available?");
            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, renderer);

            dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, renderer);
            dwRenderer_setLineWidth(2.f, renderer);
        }

        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            std::string file = "video=" + getArgument("video");

            dwSensorParams sensorParams;
            sensorParams.protocol   = "camera.virtual";
            sensorParams.parameters = file.c_str();
            CHECK_DW_ERROR_MSG(dwSAL_createSensor(&camera, sensorParams, sal),
                               "Cannot create virtual camera sensor, maybe wrong video file?");

            dwSensorCamera_getSensorProperties(&cameraProps, camera);
            dwSensorCamera_getImageProperties(&cameraImageProps, DW_CAMERA_PROCESSED_IMAGE, camera);

            printf("Camera image with %dx%d at %f FPS\n", cameraImageProps.width,
                   cameraImageProps.height, cameraProps.framerate);


            // we would like the application run as fast as the original video
            setProcessRate(cameraProps.framerate);
        }


        // -----------------------------
        // initialize streamer pipeline
        // -----------------------------
        {
            auto displayImageProps  = cameraImageProps;
            switch (cameraImageProps.type)
            {
            case DW_IMAGE_CUDA:
                displayImageProps.pxlFormat  = DW_IMAGE_RGBA;
                displayImageProps.planeCount = 1;
                CHECK_DW_ERROR(dwImageStreamer_initialize(&image2GL,
                                                          &displayImageProps,
                                                          DW_IMAGE_GL,
                                                          context));
                CHECK_DW_ERROR(dwImageFormatConverter_initialize(&formatConverter,
                                                                 DW_IMAGE_CUDA,
                                                                 context));

                cudaPool.resize(IMAGE_BUFFER_SIZE);
                for (size_t i = 0; i < cudaPool.size(); ++i) {
                    cudaPool[i] = new dwImageCUDA;
                    CHECK_DW_ERROR_MSG(dwImageCUDA_create(cudaPool[i], &displayImageProps, DW_IMAGE_CUDA_PITCH),
                                       "Cannot allocate CUDA image, is the GPU CUDA compatible?");
                }

                break;

            #ifdef VIBRANTE
                case DW_IMAGE_NVMEDIA: {
                    displayImageProps.pxlFormat  = DW_IMAGE_RGBA;
                    displayImageProps.planeCount = 1;
                    CHECK_DW_ERROR(dwImageStreamer_initialize(&image2GL,
                                                              &displayImageProps,
                                                              DW_IMAGE_GL,
                                                              context));
                    CHECK_DW_ERROR(dwImageStreamer_initialize(&nvm2cuda,
                                                                 &cameraImageProps,
                                                                 DW_IMAGE_CUDA,
                                                                 context));

                    CHECK_DW_ERROR(dwImageFormatConverter_initialize(&formatConverter,
                                                                     DW_IMAGE_NVMEDIA,
                                                                     context));


                    nvmediaPool.resize(IMAGE_BUFFER_SIZE);
                    for (size_t i = 0; i < nvmediaPool.size(); ++i) {
                        nvmediaPool[i] = new dwImageNvMedia;
                        CHECK_DW_ERROR_MSG(dwImageNvMedia_create(nvmediaPool[i], &displayImageProps, context),
                                           "Cannot create nvmedia image, potentially this is not a supported PDK");
                    }
                } break;
            #endif

            case DW_IMAGE_CPU:
            case DW_IMAGE_GL:
            default:
                break;
            }
        }

        // -----------------------------
        // Initialize scaling feature tracker
        // -----------------------------
        {
            dwScalingFeatureTrackerParameters params = {};
            dwScalingFeatureTracker_initDefaultParams(&params);
            CHECK_DW_ERROR(dwScalingFeatureTracker_initialize(&tracker, &params, 0, context));
            CHECK_DW_ERROR(dwScalingFeatureList_initialize(&featureList, context, 0,
                                                           maxFeatureCount, cameraImageProps.pxlType));
            dwScalingFeatureList_getDataBasePointer(&d_featureDataBase, &featureDataSize, featureList);
            featureDataBuffer.reset(new uint8_t[featureDataSize]);
            dwScalingFeatureList_getDataPointers(&featureData, featureDataBuffer.get(), featureList);

            cudaMalloc(&d_validFeatureCount, sizeof(uint32_t));
            cudaMalloc(&d_validFeatureIndexes, maxFeatureCount * sizeof(uint32_t));
            cudaMalloc(&d_invalidFeatureCount, sizeof(uint32_t));
            cudaMalloc(&d_invalidFeatureIndexes, maxFeatureCount * sizeof(uint32_t));
        }

        // -----------------------------
        // Initialize render buffer
        // -----------------------------
        {
            dwRenderBufferVertexLayout layout;
            layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
            layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
            layout.colFormat   = DW_RENDER_FORMAT_NULL;
            layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
            layout.texFormat   = DW_RENDER_FORMAT_NULL;
            layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
            CHECK_DW_ERROR(dwRenderBuffer_initialize(&featureRenderBuffer, layout,
                                                     DW_RENDER_PRIM_LINELIST,
                                                     maxFeatureCount*100, context));

            dwRenderBuffer_set2DCoordNormalizationFactors((float)cameraImageProps.width,
                                                          (float)cameraImageProps.height,
                                                          featureRenderBuffer);
        }

        // -----------------------------
        // Start Sensors
        // -----------------------------
        dwSensor_start(camera);

        // -----------------------------
        // Add some features at the beginning
        // -----------------------------
        newBoxToTrack.push_back({319,188, 25,35});
        newBoxToTrack.push_back({270,227, 44,56});
        newBoxToTrack.push_back({651,275, 46,36});
        newBoxToTrack.push_back({714,279, 25,42});
        newBoxToTrack.push_back({745,231, 76,38});
        newBoxToTrack.push_back({1075,222, 60,47});
        newBoxToTrack.push_back({131,274, 36,93});
        newBoxToTrack.push_back({255,378, 26,29});
        newBoxToTrack.push_back({603,398, 50,36});
        newBoxToTrack.push_back({726,402, 74,39});
        newBoxToTrack.push_back({518,334, 27,38});
        updateNewBox = false;

        return true;
    }

    virtual void onMouseDown(int button, float x, float y) override
    {
        if (button == 0) {
            updateNewBox = true;
            newBoxToTrack.push_back(dwBox2D{});
            newBoxToTrack.back().x = x*cameraImageProps.width/getWindowWidth() + 0.5f;
            newBoxToTrack.back().y = y*cameraImageProps.height/getWindowHeight() + 0.5f;
        }
    }

    virtual void onMouseMove(float x, float y) override
    {
        if (!updateNewBox)
            return;

        int32_t ix = static_cast<int32_t>(x*cameraImageProps.width/getWindowWidth() + 0.5f);
        int32_t iy = static_cast<int32_t>(y*cameraImageProps.height/getWindowHeight() + 0.5f);
        dwBox2D& box = newBoxToTrack.back();
        box.width  = abs(ix - box.x);
        box.height = abs(iy - box.y);
        if (box.x > ix) box.x = ix;
        if (box.y > iy) box.y = iy;
    }

    virtual void onMouseUp(int button, float/* x*/, float/* y*/) override
    {
        if (button == 0) {
            if(updateNewBox) {
                updateNewBox = false;

                // Discard boxes that are too small
                if(newBoxToTrack.back().width < 3 || newBoxToTrack.back().height < 3)
                    newBoxToTrack.pop_back();

                auto &newBox = newBoxToTrack.back();
                std::cout << "New feature added: (" << newBox.x << "," << newBox.y << ") size=(" << newBox.width << "," << newBox.height << ")" << std::endl;
            }
        }
    }

    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        newBoxToTrack.clear();
        trackedBoxes.clear();
        dwSensor_reset(camera);
        dwScalingFeatureList_reset(featureList);
        dwScalingFeatureTracker_reset(tracker);
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        releaseFrame();

        // stop sensor
        dwSensor_stop(camera);
        dwSAL_releaseSensor(&camera);

        // release renderer and streamer
        dwRenderer_release(&renderer);
        dwImageStreamer_release(&image2GL);
        dwImageFormatConverter_release(&formatConverter);

        dwRenderBuffer_release(&featureRenderBuffer);
        dwScalingFeatureList_release(&featureList);
        dwScalingFeatureTracker_release(&tracker);

        cudaFree(d_validFeatureCount);
        cudaFree(d_validFeatureIndexes);
        cudaFree(d_invalidFeatureCount);
        cudaFree(d_invalidFeatureIndexes);

        featureDataBuffer.reset();

        // release buffers
    #ifdef VIBRANTE
        dwImageStreamer_release(&nvm2cuda);
        for (auto &each : nvmediaPool) {
            dwImageNvMedia_destroy(each);
            delete each;
        }
    #else
        for (auto &each : cudaPool) {
            dwImageCUDA_destroy(each);
            delete each;
        }
    #endif

        // -----------------------------------------
        // Release DriveWorks handles, context and SAL
        // -----------------------------------------
        {
            dwSAL_release(&sal);
            dwRelease(&context);
        }
    }


    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, renderer);
    }

    void onRender() override
    {
        glClearColor(0.0, 0.0, 1.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (frameGL) {
            dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);
        }

        if (m_pause) {
            dwRenderer_setColor(DW_RENDERER_COLOR_RED, renderer);
            dwRenderer_renderText(32, 32,
                                  "Drag the mouse to add boxes for tracking, press space to start tracking",
                                  renderer);
            drawBoxes(newBoxToTrack, nullptr, static_cast<float32_t>(cameraImageProps.width),
                      static_cast<float32_t>(cameraImageProps.height), featureRenderBuffer, renderer);
        }

        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
        drawBoxes(trackedBoxes, nullptr, static_cast<float32_t>(cameraImageProps.width),
                  static_cast<float32_t>(cameraImageProps.height), featureRenderBuffer, renderer);
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - grab a frame from the camera
    ///     - convert frame to RGB
    ///     - push frame through the streamer to convert it into GL
    ///     - track the features in the frame
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        ProfileCUDASection s(&m_profiler, "ProcessFrame");

        releaseFrame();

        // ---------------------------
        // grab frame from camera
        // ---------------------------
        auto status = dwSensorCamera_readFrame(&frame, 0, 60000, camera);
        if (status == DW_END_OF_STREAM) {
            m_reset = true;
            return;
        } else if (status == DW_TIME_OUT) {
            return;
        } else if (status != DW_SUCCESS) {
            CHECK_DW_ERROR(status);
        }

        // ---------------------------
        // convert frame to RGB and push to GL
        // ---------------------------
        switch (cameraImageProps.type)
        {
        case DW_IMAGE_CUDA: {
            dwSensorCamera_getImageCUDA(&frameCudaYuv, DW_CAMERA_PROCESSED_IMAGE, frame);

            auto* rgbCUDA = cudaPool.back();
            cudaPool.pop_back();
            dwImageFormatConverter_copyConvertCUDA(rgbCUDA, frameCudaYuv, formatConverter, 0);
            dwImageStreamer_postCUDA(rgbCUDA, image2GL);
            dwImageStreamer_receiveGL(&frameGL, 30000, image2GL);
        } break;

    #ifdef VIBRANTE
        case DW_IMAGE_NVMEDIA: {
            dwSensorCamera_getImageNvMedia(&frameNvmYuv, DW_CAMERA_PROCESSED_IMAGE, frame);
            dwImageStreamer_postNvMedia(frameNvmYuv, nvm2cuda);
            dwImageStreamer_receiveCUDA(&frameCudaYuv, 30000, nvm2cuda);

            auto* rgbNvMedia = nvmediaPool.back();
            nvmediaPool.pop_back();
            dwImageFormatConverter_copyConvertNvMedia(rgbNvMedia, frameNvmYuv, formatConverter);
            dwImageStreamer_postNvMedia(rgbNvMedia, image2GL);
            dwImageStreamer_receiveGL(&frameGL, 30000, image2GL);
        } break;
    #endif

        case DW_IMAGE_CPU:
        case DW_IMAGE_GL:
        default:
            break;
        }

        // ---------------------------
        // track the features in the frame
        // ---------------------------
        trackFrame();

        // ---------------------------
        // Add new features
        // ---------------------------
        if(!newBoxToTrack.empty()) {
            addNewFeaturesToTracker();
        } else if (*featureData.featureCount == 0) {
            m_pause = true;
        }

    }

    void releaseFrame()
    {
        switch (cameraImageProps.type) {
        case DW_IMAGE_CUDA:
        {
            if (frameGL) {
                dwImageCUDA *retimg = nullptr;
                dwImageStreamer_returnReceivedGL(frameGL, image2GL);
                dwImageStreamer_waitPostedCUDA(&retimg, 33000, image2GL);
                cudaPool.push_back(retimg);
            }
            if (frameCudaYuv)
                dwSensorCamera_returnFrame(&frame);
            break;
        }
#ifdef VIBRANTE
        case DW_IMAGE_NVMEDIA:
        {
            dwImageNvMedia* retimg = nullptr;
            if (frameGL) {
                dwImageStreamer_returnReceivedGL(frameGL, image2GL);
                dwImageStreamer_waitPostedNvMedia(&retimg, 33000, image2GL);
                nvmediaPool.push_back(retimg);
            }

            if (frameCudaYuv) {
                dwImageStreamer_returnReceivedCUDA(frameCudaYuv, nvm2cuda);
                dwImageStreamer_waitPostedNvMedia(&retimg, 33000, nvm2cuda);
                dwSensorCamera_returnFrame(&frame);
            }
            break;
        }
#endif
        case DW_IMAGE_CPU:
        case DW_IMAGE_GL:
        default:
            break;
        }

        frameCudaYuv = nullptr;
        frameGL = nullptr;
    }

    void trackFrame()
    {
        ProfileCUDASection s(&m_profiler, "trackFrame");

        {
            ProfileCUDASection s(&m_profiler, "trackCall");
            CHECK_DW_ERROR(dwScalingFeatureTracker_trackAsync(featureList, frameCudaYuv, tracker));
        }

        compactFeatures();

        {
            //Get tracked feature info to CPU
            ProfileCUDASection s(&m_profiler, "downloadToCPU");
            cudaMemcpy(featureDataBuffer.get(), d_featureDataBase, featureDataSize, cudaMemcpyDeviceToHost);
        }

        trackedBoxes.resize(*featureData.featureCount);
        for (uint32_t i = 0; i < *featureData.featureCount; i++) {
            dwVector2f& location = featureData.locations[i];
            dwVector2f& size = featureData.sizes[i];
            dwBox2D& box = trackedBoxes[i];
            box.x = location.x - size.x/2 + 0.5f;
            box.y = location.y - size.y/2 + 0.5f;
            box.width = size.x + 0.5f;
            box.height = size.y + 0.5f;
        }

        {
            ProfileCUDASection s(&m_profiler, "updateTemplate");
            CHECK_DW_ERROR(dwScalingFeatureTracker_updateTemplateAsync(featureList, frameCudaYuv, tracker));
        }
    }

    void addNewFeaturesToTracker()
    {
        // Avoid adding features while user is drawing
        if(updateNewBox)
            return;

        ProfileCUDASection s(&m_profiler, "AddFeature");
        if (newBoxToTrack.empty())
            return;

        uint32_t &nNew = *featureData.featureCount;
        uint32_t nOld = nNew;

        for (const dwBox2D& box : newBoxToTrack) {
            dwVector2f location = {box.x + box.width/2.f, box.y + box.height/2.f};
            dwVector2f size = {box.width*1.f, box.height*1.f};

            featureData.locations[nNew] = location;
            featureData.sizes[nNew] = size;
            featureData.statuses[nNew] = DW_FEATURE_STATUS_DETECTED;
            featureData.bNewTemplate[nNew] = true;
            nNew++;

            if (nNew >= maxFeatureCount) {
                std::cout << "Too much features, will truncate the number to " << maxFeatureCount << std::endl;
                break;
            }
        }

        {
            ProfileCUDASection s(&m_profiler, "uploadToGPU");
            cudaMemcpy(d_featureDataBase, featureDataBuffer.get(), featureDataSize, cudaMemcpyHostToDevice);
        }

        {
            ProfileCUDASection s(&m_profiler, "addEmptyEntry");
            CHECK_DW_ERROR(dwScalingFeatureList_addEmptyFeature(nNew-nOld, featureList));
        }

        compactFeatures();

        {
            ProfileCUDASection s(&m_profiler, "updateTemplate");
            CHECK_DW_ERROR(dwScalingFeatureTracker_updateTemplateAsync(featureList, frameCudaYuv, tracker));
        }

        newBoxToTrack.clear();
    }

    void compactFeatures()
    {
        ProfileCUDASection s(&m_profiler, "compactFeature");

        //Determine which features to throw away
        {
            ProfileCUDASection s(&m_profiler, "removeFeatureLargerThan");
            dwScalingFeatureList_applySizeFilter(DW_MAX_TEMPLATE_SIZE, DW_MAX_TEMPLATE_SIZE,
                                                 featureList);
        }

        {
            ProfileCUDASection s(&m_profiler, "selectValid");
            dwScalingFeatureList_selectValid(d_validFeatureCount, d_validFeatureIndexes,
                                             d_invalidFeatureCount, d_invalidFeatureIndexes,
                                             featureList);
        }

        //Compact list
        {
            ProfileCUDASection s(&m_profiler, "compactCall");
            dwScalingFeatureList_compact(featureList,
                                         d_validFeatureCount, d_validFeatureIndexes,
                                         d_invalidFeatureCount, d_invalidFeatureIndexes);
        }
    }
};



//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
    {
       ProgramArguments::Option_t("video", (DataPath::get() + "/samples/sfm/triangulation/video_0.h264").c_str()),
    },
    "Camera Scaling Tracker sample which tracks user defined scaling features and playback the results in a GL window.");


    // -------------------
    // initialize and start a window application
    CameraScalingFeatureTracker app(args);

    app.initializeWindow("Camera Scaling Tracker", 1280, 800, args.enabled("offscreen"));

    return app.run();
}
