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

#include "stereo_common/stereoCommon.hpp"

class StereoCameraUSBApp : public StereoApp
{
public:
    explicit StereoCameraUSBApp(const ProgramArguments& args);

    bool onInitialize() override final;
    void onProcess() override final;
    void onRender() override final;
    void onRelease() override final;

    // handle to the stereo algorithm
    dwStereoHandle_t m_stereoAlgorithm;
    // handle to pyramids, input to the stereo algorithm
    dwPyramidHandle_t m_pyramids[DW_STEREO_SIDE_COUNT];
    // handle to stereo rectifier
    dwStereoRectifierHandle_t m_stereoRectifier;

    // input from camera and pointers
    dwImageCUDA *m_stereoImagesPtr[DW_STEREO_SIDE_COUNT];
    dwImageCUDA m_stereoImages[DW_STEREO_SIDE_COUNT];

    // converts RGBA to R to pass to Stereo algorithm
    dwImageFormatConverterHandle_t m_RGBA2R;
    dwImageCUDA m_outputRectified[DW_STEREO_SIDE_BOTH];

    // streamer for analglyph
    std::unique_ptr<SimpleImageStreamer<dwImageCUDA, dwImageGL>> m_cudaRGBA2gl;

    // streamer for disparity colorcoded (different one because we can change the resolution of the output)
    std::unique_ptr<SimpleImageStreamer<dwImageCUDA, dwImageGL>> m_cudaDISP2gl;

    // anaglyph (left and right overlapped)
    dwImageCUDA m_outputAnaglyph;
    // container for color coded disparity map
    dwImageCUDA m_colorDisparity;
    dwImageCUDA m_outputRectifiedR;
    uint32_t m_levelStop;
private:
    dwBox2D m_roi;
    dwImageGL *m_displayInput, *m_displayDisparity;
};

//#######################################################################################
StereoCameraUSBApp::StereoCameraUSBApp(const ProgramArguments& args) : StereoApp(args)
{
    StereoApp::m_inputTypeCamera = true;
    StereoApp::m_inputSingleImage = true;
}

//#######################################################################################
bool StereoCameraUSBApp::onInitialize()
{
    if (!StereoApp::initSDK()) {
        return false;
    }

    if (!StereoApp::initRenderer()) {
        return false;
    }

    if (!StereoApp::onInitialize()) {
        return false;
    }

    // get calibrated cameras from the rig configuration (see also sample_rigconfiguration for more details)
    if (!StereoApp::getCameraFromRig()) {
        return false;
    }

    // get the extrinsics transformation matrices
    dwTransformation left2Rig, right2Rig;
    CHECK_DW_ERROR(dwRigConfiguration_getSensorToRigTransformation(&left2Rig, 0,
                                                                   StereoApp::m_rigConfiguration));
    CHECK_DW_ERROR(dwRigConfiguration_getSensorToRigTransformation(&right2Rig, 1,
                                                                   StereoApp::m_rigConfiguration));

    // initialize the stereo rectifier using the camera models (intrinsics) and transformations (extrinsics)
    CHECK_DW_ERROR( dwStereoRectifier_initialize(&m_stereoRectifier, StereoApp::m_cameraModel[0],
                    StereoApp::m_cameraModel[1], left2Rig, right2Rig, StereoApp::m_context));

    // container for output of the stereo rectifier. The properties are set to RGBA interleaved uint8 because
    // the stereo rectifier supports it and also because we want to visualize the colored rectified images
    dwImageProperties props = m_imageProperties;
    props.type = DW_IMAGE_CUDA;
    props.planeCount = 1;
    props.pxlFormat = DW_IMAGE_RGBA;
    props.pxlType = DW_TYPE_UINT8;

    // allocate memory for the rectified image
    for(uint32_t i=0; i<DW_STEREO_SIDE_COUNT; ++i) {
        dwImageCUDA_create(&m_outputRectified[i], &props, DW_IMAGE_CUDA_PITCH);
    }

    // initialize the second streamer CUDA to GL which will stream an image of the same resolution as the
    // as the rectified image that will be used for display
    CHECK_DW_ERROR(dwStereoRectifier_getCropROI(&m_roi, m_stereoRectifier));
    props.width = m_roi.width;
    props.height = m_roi.height;

    DriveWorksSample::m_window->setWindowSize(props.width * 2, props.height);
    std::cout << "Rectified image: " << m_roi.width << "x" << m_roi.height << std::endl;

    m_cudaRGBA2gl.reset(new SimpleImageStreamer<dwImageCUDA, dwImageGL>(props, 10000,
                                                                        StereoApp::m_context));

    // allocate memory for the anaglyph (see also sample_stereo_disparity)
    dwImageCUDA_create(&m_outputAnaglyph, &props, DW_IMAGE_CUDA_PITCH);

    m_stereoImagesPtr[0] = &m_stereoImages[0];
    m_stereoImagesPtr[1] = &m_stereoImages[1];

    // setup pyramids for the stereo algorithm. The algorithm accepts grayscale images at multiple resolutions
    // this is why we prepare a DW_IMAGE_R (single channel, gray scale)...
    dwImageProperties pyrProp = props;
    pyrProp.pxlFormat = DW_IMAGE_R;
    dwImageCUDA_create(&m_outputRectifiedR, &pyrProp, DW_IMAGE_CUDA_PITCH);

    // ...and we allocate for a gaussian pyramid that will be built with the rectified images
    // the number of levels is adjusted to the base resolution of a typical USB stereo camera
    uint32_t levelCount = 3;
    dwPyramidConfig pyramidConf;
    pyramidConf.dataType = DW_TYPE_UINT8;
    pyramidConf.height = props.height;
    pyramidConf.width = props.width;
    pyramidConf.levelCount = levelCount;

    for (int32_t i = 0; i < DW_STEREO_SIDE_BOTH; ++i) {
        CHECK_DW_ERROR(dwPyramid_initialize(&m_pyramids[i], StereoApp::m_context, 0, pyramidConf));
    }

    // setup stereo algorithm
    dwStereoParams stereoParams;
    CHECK_DW_ERROR(dwStereo_initParams(&stereoParams));

    // set the same levels as the pyramid (it is not necessary that the stereo algorithm starts from the
    // lowest level of a pyramid, since this can be easily used by another module that requires more lower levels)
    stereoParams.levelCount = levelCount;
    m_levelStop = stereoParams.levelStop;

    CHECK_DW_ERROR(dwStereo_initialize(&m_stereoAlgorithm, props.width, props.height, &stereoParams,
                                       StereoApp::m_context));

    // setup a format converter that converts from the RGBA output of the stereo rectifier to R as input
    // for the pyramid
    CHECK_DW_ERROR(dwImageFormatConverter_initialize(&m_RGBA2R, props.type, StereoApp::m_context));

    // finally, based on the reolution we build the disparity map at, we setup an image streamer
    // for display with that resolution
    CHECK_DW_ERROR(dwStereo_getSize(&props.width, &props.height, stereoParams.levelStop, m_stereoAlgorithm));
    dwImageCUDA_create(&m_colorDisparity, &pyrProp, DW_IMAGE_CUDA_PITCH);
    m_cudaDISP2gl.reset(new SimpleImageStreamer<dwImageCUDA, dwImageGL>(props, 10000, StereoApp::m_context));
    return true;
}

//#######################################################################################
void StereoCameraUSBApp::onRelease()
{
    for(uint32_t i=0; i<DW_STEREO_SIDE_COUNT; ++i) {
        dwImageCUDA_destroy(&m_outputRectified[i]);
    }

    dwImageCUDA_destroy(&m_outputAnaglyph);
    dwImageCUDA_destroy(&m_colorDisparity);
    dwImageCUDA_destroy(&m_outputRectifiedR);

    dwImageFormatConverter_release(&m_RGBA2R);
    dwStereoRectifier_release(&m_stereoRectifier);

    for(int i=0; i<DW_STEREO_SIDE_COUNT; ++i)
        dwPyramid_release(&m_pyramids[i]);

    dwStereo_release(&m_stereoAlgorithm);

    StereoApp::onRelease();
}

//#######################################################################################
void StereoCameraUSBApp::onProcess()
{
    std::this_thread::yield();
    while (!StereoApp::readStereoImages(m_stereoImagesPtr)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    CHECK_DW_ERROR(dwStereoRectifier_rectify(&m_outputRectified[DW_STEREO_SIDE_LEFT],
                                             &m_outputRectified[DW_STEREO_SIDE_RIGHT],
                                             m_stereoImagesPtr[0], m_stereoImagesPtr[1], m_stereoRectifier));

    // see sample_stereo_rectifier
    // remap the rectified images to their valid ROI
    dwImageCUDA croppedRectified[2];
    for (int32_t i = 0; i < DW_STEREO_SIDE_BOTH; ++i) {
        CHECK_DW_ERROR(dwImageCUDA_mapToROI(&croppedRectified[i], &m_outputRectified[i], m_roi));
    }

    createAnaglyph(m_outputAnaglyph, croppedRectified[0], croppedRectified[1]);

    // get display GL image for anaglyph
    m_displayInput = m_cudaRGBA2gl->post(&m_outputAnaglyph);

    // compute disparity
    for (int32_t i = 0; i < DW_STEREO_SIDE_BOTH; ++i) {
        // convert RGBA to R
        CHECK_DW_ERROR(dwImageFormatConverter_copyConvertCUDA(&m_outputRectifiedR, &croppedRectified[i],
                                                              m_RGBA2R, 0));
        // feed image to build a pyramid
        CHECK_DW_ERROR(dwPyramid_build(&m_outputRectifiedR, m_pyramids[i]));
    }

    // compute disparity
    CHECK_DW_ERROR(dwStereo_computeDisparity(m_pyramids[DW_STEREO_SIDE_LEFT],
                                             m_pyramids[DW_STEREO_SIDE_RIGHT],
                                             m_stereoAlgorithm));

    // see sample_stereo_disparity
    // the grayscale disparity is colorcoded for better results
    const dwImageCUDA *disparity, *confidence;
    CHECK_DW_ERROR(dwStereo_getDisparity(&disparity, DW_STEREO_SIDE_LEFT, m_stereoAlgorithm));

    colorCode(&m_colorDisparity, *disparity, 4.0 * (1 << m_levelStop));

    CHECK_DW_ERROR(dwStereo_getConfidence(&confidence, DW_STEREO_SIDE_LEFT, m_stereoAlgorithm));

    // confidence map is used to show occlusions as black pixels
    mixDispConf(&m_colorDisparity, *confidence, StereoApp::m_invalidThreshold >= 0.0f);

    // get display GL image for disparity map
    m_displayDisparity = m_cudaDISP2gl->post(&m_colorDisparity);
}

void StereoCameraUSBApp::onRender(){
    dwVector2i windowSize{DriveWorksSample::m_window->width(), DriveWorksSample::m_window->height()};

    dwRect screenRect{};

    // render input
    screenRect.x = 0;
    screenRect.y = 0;
    screenRect.width = m_outputAnaglyph.prop.width;
    screenRect.height = m_outputAnaglyph.prop.height;
    StereoApp::m_simpleRenderer->setScreenRect(screenRect);
    StereoApp::m_simpleRenderer->renderQuad(m_displayInput);
    StereoApp::m_simpleRenderer->renderText(10, 10, DW_RENDERER_COLOR_WHITE,
                                                         std::to_string(m_colorDisparity.prop.width) + "x" +
                                                         std::to_string(m_colorDisparity.prop.height));

    // render disparity
    screenRect.x = windowSize.x/2;
    StereoApp::m_simpleRenderer->setScreenRect(screenRect);
    StereoApp::m_simpleRenderer->renderQuad(m_displayDisparity);
    StereoApp::m_simpleRenderer->renderText(10, 10, DW_RENDERER_COLOR_WHITE,
                                                         std::to_string(m_colorDisparity.prop.width) + "x" +
                                                         std::to_string(m_colorDisparity.prop.height));
    StereoApp::m_window->swapBuffers();
}

//#######################################################################################
int main(int argc, const char **argv)
{
    ProgramArguments args(argc, argv, {
        ProgramArguments::Option_t{"rigconfig", (DataPath::get() + "/samples/stereo/full.xml").c_str(), "Rig configuration file."},
        ProgramArguments::Option_t{"device", "0", "Selects whether to use iGPU (1) or dGPU(0)"},
    });

    // -------------------

    StereoCameraUSBApp app(args);

    app.initializeWindow("StereoCameraUSBApp", 1280, 800);

    return app.run();
}
