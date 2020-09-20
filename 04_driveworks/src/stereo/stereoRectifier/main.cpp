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

#include "stereo_common/stereoCommon.hpp"

class StereoRectifierApp : public StereoApp
{
public:
    explicit StereoRectifierApp(const ProgramArguments& args);

    bool onInitialize() override final;
    void onProcess() override final;
    void onRender() override final;
    void onRelease() override final;

    bool initRenderer() override final;

private:
    // renders lines that show that rectified images have pixels that lie on the same horizontal line
    void renderHorizontalLines();
    void renderImages(dwRect screenRect, dwStereoSide side);

    // stereo rectifier
    dwStereoRectifierHandle_t m_stereoRectifier;

    // input from video
    dwImageCUDA *m_stereoImages[DW_STEREO_SIDE_COUNT];

    // output of the stereo rectifier
    dwImageCUDA m_outputRectified[DW_STEREO_SIDE_COUNT];

    // streamer for display
    std::unique_ptr<SimpleImageStreamer<dwImageCUDA, dwImageGL>> m_cuda2glInput[2];
    std::unique_ptr<SimpleImageStreamer<dwImageCUDA, dwImageGL>> m_cuda2glOutput[2];

    // container for lines
    dwRenderBufferHandle_t m_lineBuffer;
    std::vector<SimpleRenderer::dwLineSegment2Df> m_lines;

    /**
     * images that are undistorted following rectification (due to intrinsics prior distortion and
     * additional distortion after stereo rectification) are not shaped like rectangles. This roi contains
     * a rectangular are of valid pixels common to both cameras. The roi can end up being very narrow if
     * the two cameras have either heavy distortion or very large baseline
     */
    dwBox2D m_roi;

    // dwImageGL for display
    dwImageGL *m_displayInput[DW_STEREO_SIDE_COUNT], *m_displayOutput[DW_STEREO_SIDE_COUNT];
};

//#######################################################################################
StereoRectifierApp::StereoRectifierApp(const ProgramArguments& args) : StereoApp(args)
{
}

//#######################################################################################
bool StereoRectifierApp::onInitialize()
{
    if (!StereoApp::initSDK()) {
        return false;
    }

    if (!StereoApp::onInitialize()) {
        return false;
    }

    // get calibrated cameras from the rig configuration (see also sample_rigconfiguration for more details)
    if (!StereoApp::getCameraFromRig()) {
        return false;
    }

    // get the extrinsics transformation matrices (NOTE that this sample assumes the stereo cameras are the
    // first two of the enumerated camera sensors and are ordered LEFT and RIGHT)
    dwTransformation left2Rig, right2Rig;
    CHECK_DW_ERROR(dwRigConfiguration_getSensorToRigTransformation(&left2Rig, 0,
                                                                   StereoApp::m_rigConfiguration));
    CHECK_DW_ERROR(dwRigConfiguration_getSensorToRigTransformation(&right2Rig, 1,
                                                                   StereoApp::m_rigConfiguration));

    // initialize the stereo rectifier using the camera models (intrinsics) and transformations (extrinsics)
    CHECK_DW_ERROR(dwStereoRectifier_initialize(&m_stereoRectifier, StereoApp::m_cameraModel[0],
                                                StereoApp::m_cameraModel[1], left2Rig, right2Rig,
                                                StereoApp::m_context));

    //container for output of the stereo rectifier
    dwImageProperties propsInput = m_imageProperties;
    propsInput.type = DW_IMAGE_CUDA;
    propsInput.planeCount = 1;
    propsInput.pxlFormat = DW_IMAGE_RGBA;
    propsInput.pxlType = DW_TYPE_UINT8;

    for(uint32_t i=0; i<DW_STEREO_SIDE_COUNT; ++i) {
        dwImageCUDA_create(&m_outputRectified[i], &propsInput, DW_IMAGE_CUDA_PITCH);
    }

    // after rectification we use the ROI for cropping the images to the valid region, so when we stream
    // them from CUDA to GL for display we need to use the resolution after cropping
    dwImageProperties propsOutput = propsInput;
    CHECK_DW_ERROR(dwStereoRectifier_getCropROI(&m_roi, m_stereoRectifier));
    propsOutput.width = static_cast<uint32_t>(m_roi.width);
    propsOutput.height = static_cast<uint32_t>(m_roi.height);

    // initialize the streamer for display
    for (unsigned int i = 0; i < DW_STEREO_SIDE_BOTH; ++i) {
        m_cuda2glInput[i].reset(new SimpleImageStreamer<dwImageCUDA, dwImageGL>(propsInput, 10000,
                                                                                StereoApp::m_context));
        m_cuda2glOutput[i].reset(new SimpleImageStreamer<dwImageCUDA, dwImageGL>(propsOutput, 10000,
                                                                                 StereoApp::m_context));
    }

    std::cout << "Rectified image: " << m_roi.width << "x" << m_roi.height << std::endl;

    return initRenderer();
}

//#######################################################################################
bool StereoRectifierApp::initRenderer()
{
    if (!StereoApp::initRenderer()) {
        return false;
    }

    // setup line buffer for drawing horizontal lines. We draw horizontal lines to show that a rectified pair
    // has the same pixels lying on horizontal line (see also sample_renderer)
    unsigned int maxLines = 100;

    dwRenderBufferVertexLayout layout;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;

    CHECK_DW_ERROR(dwRenderBuffer_initialize(&m_lineBuffer, layout, DW_RENDER_PRIM_LINELIST, maxLines,
                                             StereoApp::m_context));
    CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)StereoApp::m_window->width(),
                                                                 (float32_t)StereoApp::m_window->height(),
                                                                 m_lineBuffer));

    for (float i = 70.0f; i < (StereoApp::m_window->height() - 70.0f); i += 40.0f) {
        m_lines.push_back(SimpleRenderer::dwLineSegment2Df{dwVector2f{0, i},
                                                           dwVector2f{
                                                               static_cast<float>(StereoApp::m_window->width())
                                                               , i}});
    }

    return true;
}

//#######################################################################################
void StereoRectifierApp::onRelease()
{
    for(uint32_t i=0; i<DW_STEREO_SIDE_COUNT; ++i) {
        dwImageCUDA_destroy(&m_outputRectified[i]);
    }

    dwRenderBuffer_release(&m_lineBuffer);
    dwStereoRectifier_release(&m_stereoRectifier);

    StereoApp::onRelease();
}

//#######################################################################################
void StereoRectifierApp::onProcess()
{
    std::this_thread::yield();
    while (!StereoApp::readStereoImages(m_stereoImages)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    CHECK_DW_ERROR(dwStereoRectifier_rectify(&m_outputRectified[DW_STEREO_SIDE_LEFT],
                                             &m_outputRectified[DW_STEREO_SIDE_RIGHT],
                                             m_stereoImages[0], m_stereoImages[1],
                                             m_stereoRectifier));

    // get display images
    for (uint32_t i = 0; i < DW_STEREO_SIDE_BOTH; ++i) {
        m_displayInput[i] = m_cuda2glInput[i]->post(m_stereoImages[i]);

        // crop the images by mapping the pointer
        dwImageCUDA croppedRectified;
        CHECK_DW_ERROR(dwImageCUDA_mapToROI(&croppedRectified, &m_outputRectified[i], m_roi));
        m_displayOutput[i] = m_cuda2glOutput[i]->post(&croppedRectified);
    }
}

//#######################################################################################
void StereoRectifierApp::onRender()
{
    dwVector2i windowSize{StereoApp::m_window->width(), StereoApp::m_window->height()};

    dwRect screenRect{0, windowSize.y/2, windowSize.x/2, windowSize.y/2};

    for (int32_t i = 0; i < DW_STEREO_SIDE_BOTH; ++i) {
        renderImages(screenRect, (dwStereoSide)i);
        screenRect.x += screenRect.width;
    }

    StereoApp::m_window->swapBuffers();
}

//#######################################################################################
void StereoRectifierApp::renderHorizontalLines()
{
    // makes lines red and slightly transparent
    const float32_t colorRedTrans[4] = {1.0f, 0.0f, 0.0f, 0.5f};
    StereoApp::m_simpleRenderer->renderLineSegments(m_lines, 1.0f, colorRedTrans);
}

//#######################################################################################
void StereoRectifierApp::renderImages(dwRect screenRect, dwStereoSide side)
{
    dwVector2ui cameraSize{m_stereoImages[0]->prop.width, m_stereoImages[0]->prop.height};

    StereoApp::m_simpleRenderer->setScreenRect(screenRect);
    StereoApp::m_simpleRenderer->renderQuad(m_displayInput[side]);
    StereoApp::m_simpleRenderer->renderText(10, 10, DW_RENDERER_COLOR_WHITE,
                                 "Input " + std::to_string(cameraSize.x) + "x" + std::to_string(cameraSize.y));
    renderHorizontalLines();

    dwVector2ui outputSize{m_displayOutput[0]->prop.width, m_displayOutput[0]->prop.height};
    screenRect.y -= screenRect.height;
    StereoApp::m_simpleRenderer->setScreenRect(screenRect);
    StereoApp::m_simpleRenderer->renderQuad(m_displayOutput[side]);
    StereoApp::m_simpleRenderer->renderText(10, 10, DW_RENDERER_COLOR_WHITE,  "Output " +
                                                         std::to_string(outputSize.x) + "x" +
                                                         std::to_string(outputSize.y));
    renderHorizontalLines();
}

//#######################################################################################
int main(int argc, const char **argv)
{
    ProgramArguments args(argc, argv, {
        ProgramArguments::Option_t{"rigconfig", (DataPath::get() + "/samples/stereo/full.xml").c_str(), "Rig configuration file."},
        ProgramArguments::Option_t{"video0", (DataPath::get() + std::string{"/samples/stereo/left_1.h264"}).c_str(), "Left input video."},
        ProgramArguments::Option_t{"video1", (DataPath::get() + std::string{"/samples/stereo/right_1.h264"}).c_str(), "Right input video."},
        ProgramArguments::Option_t{"single-input", "0", "Specifies if only one video is used as input."}
    });

    // -------------------

    StereoRectifierApp app(args);

    app.initializeWindow("StereoRectifierApp", 1280, 800);

    return app.run();
}