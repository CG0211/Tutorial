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

class DriveNetMultiGPUApp : public DriveNetApp
{
public :
    DriveNetMultiGPUApp(const ProgramArguments &args) : DriveNetApp(args) {}

    bool onInitialize() override final;
    void onRender() override final;
    void onProcess() override final;
    void onRelease() override final;
    void onReset() override final;

private:
    // used for display
    dwImageGL* imgGl;
    dwImageCUDA iGPUCUDAimage{};
    bool hasPeerAccess;
};

bool DriveNetMultiGPUApp::onInitialize()
{
    if (!DriveNetApp::initSDK())
        return false;

    // check gpu configuration
    int32_t gpuCount;
    CHECK_DW_ERROR(dwContext_getGPUCount(&gpuCount, DriveNetApp::context));
    if (gpuCount < 2) {
        throw std::runtime_error("Sample needs 2 GPUs to be able to run, aborting.");
    }

    int canAccessPeer = 0;
    if (cudaDeviceCanAccessPeer(&canAccessPeer, 0, 1) != cudaSuccess) {
        throw std::runtime_error("Error checking peer access");
    }
    hasPeerAccess = canAccessPeer != 0;
    if (!hasPeerAccess) {
        logWarn("GPUs don't have peer access, going through host\n");
    }

    // initialize cameras and setup for receiving directly the useful frame
    if (!DriveNetApp::initSensors()) {
        return false;
    }

    // initialize the renderer
    if (!DriveNetApp::initRenderer()) {
        return false;
    }

    // initialize drivenet detector on GPU 0
    CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
    if (!DriveNetApp::initDetector(DriveNetApp::getImageProperties(), DriveNetApp::cudaStream)) {
        return false;
    }

    // initialize object tracker on GPU 1
    CHECK_DW_ERROR(dwContext_selectGPUDevice(1, DriveNetApp::context));
    if (!DriveNetApp::initTracker(DriveNetApp::getImageProperties(), DriveNetApp::cudaStream)) {
        return false;
    }

    // the image that comes from the camera resides on GPU 0 (default) and in order to use it the iGPU
    // we need to have a copy, so we allocate for a copy
    dwImageProperties imageProperties = DriveNetApp::getImageProperties();
    CHECK_DW_ERROR(dwImageCUDA_create(&iGPUCUDAimage, &imageProperties, DW_IMAGE_CUDA_PITCH));

    // set GPU 0 back as default
    CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));

    return true;
}

void DriveNetMultiGPUApp::onProcess()
{
    // read from camera
    dwImageCUDA* inputImage = nullptr;
    DriveNetApp::getNextFrame(&inputImage, &imgGl);
    std::this_thread::yield();
    while (inputImage == nullptr) {
        CHECK_DW_ERROR(dwContext_selectGPUDevice(1, DriveNetApp::context));
        DriveNetApp::resetTracker();
        CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
        DriveNetApp::resetDetector();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DriveNetApp::getNextFrame(&inputImage, &imgGl);
    }

    // copy rgb image on the other gpu
    size_t byteSize = inputImage->pitch[0] * inputImage->prop.height *
                      dwSizeOf(inputImage->prop.pxlType);

    if(hasPeerAccess) {
        CHECK_DW_ERROR(dwContext_selectGPUDevice(1, DriveNetApp::context));
        for (uint32_t i = 0; i < inputImage->prop.planeCount; ++i) {
            if (cudaSuccess != cudaMemcpyPeerAsync(iGPUCUDAimage.dptr[i], 1,
                                                   inputImage->dptr[i], 0, byteSize,
                                                   DriveNetApp::cudaStream)) {
                throw std::runtime_error("Cannot memcpy with peer access from gpu 0 to gpu 1");
            }
        }
        CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
    } else {
        // in order to copy rgb image from gpu 0 to gpu 1 we need to copy from gpu 0 to host
        // first...
        for (uint32_t i = 0; i < inputImage->prop.planeCount; ++i) {
            std::vector<int16_t> cpuData(inputImage->pitch[i] * inputImage->prop.width);
            if (cudaSuccess != cudaMemcpy2DAsync(cpuData.data(), inputImage->pitch[i],
                                                 inputImage->dptr[i], inputImage->pitch[i],
                                                 inputImage->prop.width * dwSizeOf(inputImage->prop.pxlType),
                                                 inputImage->prop.height, cudaMemcpyDeviceToHost,
                                                 DriveNetApp::cudaStream)) {
                throw std::runtime_error("Cannot memcpy with peer access from gpu 0 to host");
            }

            // ...and then from host to gpu 1, which needs to be current when calling memcpy
            CHECK_DW_ERROR(dwContext_selectGPUDevice(1, DriveNetApp::context));
            if (cudaSuccess != cudaMemcpy2DAsync(iGPUCUDAimage.dptr[i], inputImage->pitch[i],
                                                 cpuData.data(),  inputImage->pitch[i],
                                                 inputImage->prop.width * dwSizeOf(inputImage->prop.pxlType),
                                                 inputImage->prop.height, cudaMemcpyHostToDevice,
                                                 DriveNetApp::cudaStream)) {
                throw std::runtime_error("Cannot memcpy with peer access from host to gpu 1");
            }
            CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
        }
    }
    /////////////
    // detect objects on GPU 0, track on GPU 1 and get the results on GPU 0
    CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
    DriveNetApp::inferDetectorAsync(inputImage);

    CHECK_DW_ERROR(dwContext_selectGPUDevice(1, DriveNetApp::context));
    DriveNetApp::inferTrackerAsync(&iGPUCUDAimage);

    CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
    DriveNetApp::processResults();
}

void DriveNetMultiGPUApp::onRender()
{
    DriveNetApp::simpleRenderer->renderQuad(imgGl);

    // render boxes with labels for each detected class
    for (size_t classIdx = 0; classIdx < DriveNetApp::getNumClasses(); classIdx++) {

        // render bounding box
        CHECK_DW_ERROR(dwRenderer_setColor(DriveNetApp::boxColors[classIdx % DriveNetApp::MAX_BOX_COLORS],
                       DriveNetApp::renderer));
        dwImageProperties imageProperties = DriveNetApp::getImageProperties();

        DriveNetApp::simpleRenderer->renderRectanglesWithLabels(DriveNetApp::getResult(classIdx),
                                                                static_cast<float32_t>(imageProperties.width),
                                                                static_cast<float32_t>(imageProperties.height));
    }

    // draw ROI of the first image
    DriveNetApp::simpleRenderer->renderRectangle(DriveNetApp::detectorParams.ROIs[0], DW_RENDERER_COLOR_LIGHTBLUE);

    // draw ROI of the second image
    DriveNetApp::simpleRenderer->renderRectangle(DriveNetApp::detectorParams.ROIs[1], DW_RENDERER_COLOR_YELLOW);
}

void DriveNetMultiGPUApp::onRelease()
{
    // Release tracker
    CHECK_DW_ERROR(dwContext_selectGPUDevice(1, DriveNetApp::context));
    CHECK_DW_ERROR(dwObjectTracker_release(&objectTracker));
    if (iGPUCUDAimage.dptr[0]) {
        CHECK_DW_ERROR(dwImageCUDA_destroy(&iGPUCUDAimage));
    }
    CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
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

void DriveNetMultiGPUApp::onReset()
{
    CHECK_DW_ERROR(dwContext_selectGPUDevice(1, DriveNetApp::context));
    DriveNetApp::resetTracker();
    CHECK_DW_ERROR(dwContext_selectGPUDevice(0, DriveNetApp::context));
    DriveNetApp::resetDetector();
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
    "DriveNet sample where detection and tracking are run on different GPUs.");

    DriveNetMultiGPUApp app(args);
    app.initializeWindow("DriveNet MultiGPU", 1280, 800, args.enabled("offscreen"));
    return app.run();
}
