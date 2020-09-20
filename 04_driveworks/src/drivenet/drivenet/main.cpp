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


// DRIVENET COMMON
#include <drivenet/common/DriveNetApp.hpp>
#include <drivenet/common/common.hpp>

class DriveNetSimpleApp : public DriveNetApp
{
public :
    DriveNetSimpleApp(const ProgramArguments &args) : DriveNetApp(args) {}

    bool onInitialize() override final;
    void onRender() override final;
    void onProcess() override final;
    void onRelease() override final;
    void onReset() override final;

private:
    // used for display
    dwImageGL* m_imgGl;
};

bool DriveNetSimpleApp::onInitialize()
{
    // initialize the base driveworks app
    if (!DriveNetApp::initSDK()) {
        return false;
    }

    // initialize cameras and setup for receiving directly the useful frame
    if (!DriveNetApp::initSensors()) {
        return false;
    }

    // initialize the renderer
    if (!DriveNetApp::initRenderer()) {
        return false;
    }

    // initialize drivenet detector
    if (!DriveNetApp::initDetector(DriveNetApp::getImageProperties(), DriveNetApp::cudaStream)) {
        return false;
    }

    // initialize object tracker
    if (!DriveNetApp::initTracker(DriveNetApp::getImageProperties(), DriveNetApp::cudaStream)) {
        return false;
    }

    return true;
}

void DriveNetSimpleApp::onProcess()
{
    // read from camera
    dwImageCUDA* inputImage = nullptr;
    DriveNetApp::getNextFrame(&inputImage, &m_imgGl);
    std::this_thread::yield();
    while (inputImage == nullptr) {
        DriveNetApp::resetDetector();
        DriveNetApp::resetTracker();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DriveNetApp::getNextFrame(&inputImage, &m_imgGl);
    }

    // detect objects, track and get the results
    DriveNetApp::inferDetectorAsync(inputImage);
    DriveNetApp::inferTrackerAsync(inputImage);
    DriveNetApp::processResults();
}

void DriveNetSimpleApp::onRender()
{
    DriveNetApp::simpleRenderer->setScreenRect(dwRect{0, 0, m_window->width(), m_window->height()});
    DriveNetApp::simpleRenderer->renderQuad(m_imgGl);

    // set the normalization coordinates for boxes rendering
    DriveNetApp::simpleRenderer->setRenderBufferNormCoords(DriveNetApp::getImageProperties().width,
                                                                         DriveNetApp::getImageProperties().height,
                                                                         DW_RENDER_PRIM_LINELIST);

    // render boxes with labels for each detected class
    for (size_t classIdx = 0; classIdx < DriveNetApp::getNumClasses(); classIdx++) {

        // render bounding box
        DriveNetApp::simpleRenderer->setColor(DriveNetApp::boxColors[classIdx % DriveNetApp::MAX_BOX_COLORS]);
        dwImageProperties imageProperties = DriveNetApp::getImageProperties();

        simpleRenderer->renderRectanglesWithLabels(DriveNetApp::getResult(classIdx),
                                                                    static_cast<float32_t>(imageProperties.width),
                                                                    static_cast<float32_t>(imageProperties.height));
    }

    // draw ROI of the first image
    DriveNetApp::simpleRenderer->renderRectangle(DriveNetApp::detectorParams.ROIs[0], DW_RENDERER_COLOR_LIGHTBLUE);

    // draw ROI of the second image
    DriveNetApp::simpleRenderer->renderRectangle(DriveNetApp::detectorParams.ROIs[1], DW_RENDERER_COLOR_YELLOW);
}

void DriveNetSimpleApp::onRelease()
{
    // Release tracker
    CHECK_DW_ERROR(dwObjectTracker_release(&objectTracker));
    // Release clustering
    for (uint32_t clsIdx = 0U; clsIdx < numDriveNetClasses; ++clsIdx) {
        CHECK_DW_ERROR(dwObjectClustering_release(&objectClusteringHandles[clsIdx]));
    }
    // Release detector
    CHECK_DW_ERROR(dwObjectDetector_release(&driveNetDetector));
    // Release drivenet
    CHECK_DW_ERROR(dwDriveNet_release(&driveNet));
    // Release renderer
    simpleRenderer.reset();
    CHECK_DW_ERROR(dwRenderer_release(&renderer));
    // Release camera
    camera.reset();
    // Release SDK
    CHECK_DW_ERROR(dwSAL_release(&sal));
    CHECK_DW_ERROR(dwRelease(&context));
}

void DriveNetSimpleApp::onReset()
{
    DriveNetApp::resetDetector();
    DriveNetApp::resetTracker();
}

int main(int argc, const char **argv)
{
    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
    {
#ifdef VIBRANTE
       ProgramArguments::Option_t("camera-type", "ar0231-rccb-ssc", "camera gmsl type (see sample_camera_gmsl for more info)"),
       ProgramArguments::Option_t("csi-port", "ab", "input port"),
       ProgramArguments::Option_t("camera-index", "0", "camera index within the csi-port 0-3"),
       ProgramArguments::Option_t("slave", "0", "activate slave mode for Tegra B"),
       ProgramArguments::Option_t("input-type", "video", "input type either video or camera"),
#endif
       ProgramArguments::Option_t("video", (DataPath::get() + "/samples/raw/rccb.raw").c_str(), "path to video"),
       ProgramArguments::Option_t("stopFrame", "0", "frame number indicating when to stop video")
    },
    "DriveNet sample which detects and tracks objects of multiple classes.");

    DriveNetSimpleApp app(args);
    app.initializeWindow("DriveNet Simple", 1280, 800, args.enabled("offscreen"));
    return app.run();
}
