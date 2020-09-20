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

// SAMPLES COMMON
#include <framework/Grid.hpp>

// DRIVENET COMMON
#include <drivenet/common/DriveNetApp.hpp>
#include <drivenet/common/common.hpp>


#define MAX_CAMERAS 12

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

// Driveworks Handles
dwDriveNetHandle_t gDriveNet                        = DW_NULL_HANDLE;
dwObjectDetectorHandle_t gDriveNetDetector          = DW_NULL_HANDLE;
dwObjectTrackerHandle_t gObjectTracker[MAX_CAMERAS] = {DW_NULL_HANDLE};
dwObjectClusteringHandle_t *gObjectClusterers       = nullptr;
const dwDriveNetClass *gDriveNetClasses = nullptr;
uint32_t gNumClasses;
const uint32_t gMaxProposalsPerClass = DriveNetApp::maxProposalsPerClass;
const uint32_t gMaxClustersPerClass = DriveNetApp::maxClustersPerClass;

// Number of proposals per object class
std::vector<size_t> gNumProposals[MAX_CAMERAS];
// List of proposals per object class
std::vector<std::vector<dwObject>> gObjectProposals[MAX_CAMERAS];

// Number of clusters per object class
std::vector<size_t> gNumClusters[MAX_CAMERAS];
// List of clusters per object class
std::vector<std::vector<dwObject>> gObjectClusters[MAX_CAMERAS];

// Number of merged objects per object class
std::vector<size_t> gNumMergedObjects[MAX_CAMERAS];
// List of merged objects per objecameract class
std::vector<std::vector<dwObject>> gObjectsMerged[MAX_CAMERAS];
// Number of tracked objects per object class

std::vector<size_t> gNumTrackedObjects[MAX_CAMERAS];
// List of tracked objects per object class
std::vector<std::vector<dwObject>> gObjectsTracked[MAX_CAMERAS];
// Number of object classes that the DriveNet detects

// Labels of each class
std::vector<std::string> gClassLabels;

struct Camera {
    dwSensorHandle_t sensor;
    uint32_t numSiblings;
};
std::vector<Camera> gCameras;
uint32_t gNumCameras = 0;

GridData_t gGrid;
uint32_t gImageWidth = 0;
uint32_t gImageHeight = 0;
uint32_t gFrameInference = 0;

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
bool initSensors(dwSALHandle_t *sal, std::vector<Camera> *cameras, dwImageProperties *cameraImageProperties,
                 dwCameraProperties* cameraProperties, dwContextHandle_t context);
bool initNCameras(std::vector<Camera> *cameras, dwSALHandle_t sal);
bool initNVideos(std::vector<Camera> *cameras, dwSALHandle_t sal);

bool initDriveNet(const dwImageProperties& rcbImageProps, dwContextHandle_t context);
void runPipeline(dwSoftISPHandle_t pipeline, const dwImageProperties& rawImageProps,
                 const dwImageProperties& rcbImageProp, float32_t framerate);
void detectTrack(dwImageCUDA** rcbArray, uint32_t skipInference,
                 std::vector<std::pair<dwBox2D, std::string>>* boxList);

void resizeWindowCallbackGrid(int width, int height);
void releaseGrid();

//------------------------------------------------------------------------------
// Method implementations
//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    // Program arguments
    std::string videosString = DataPath::get() + "/samples/raw/rccb.raw";
    videosString += "," + DataPath::get() + "/samples/raw/rccb.raw";
    videosString += "," + DataPath::get() + "/samples/raw/rccb.raw";
    videosString += "," + DataPath::get() + "/samples/raw/rccb.raw";
    ProgramArguments arguments({
    #ifdef VIBRANTE
        ProgramArguments::Option_t("camera-type", "ar0231-rccb-ssc"),
        ProgramArguments::Option_t("slave", "0"),
        ProgramArguments::Option_t("input-type", "video"),
        ProgramArguments::Option_t("fifo-size", "5"),
        ProgramArguments::Option_t("selector-mask", "0001"),
    #endif
        ProgramArguments::Option_t("videos", videosString.c_str()),
        ProgramArguments::Option_t("stopFrame", "0"),
        ProgramArguments::Option_t("skipFrame", "0"), // 0 skip no frames, 1 skip 1 frame every 2, and so on
        ProgramArguments::Option_t("skipInference", "0")
    });

    // init framework
    initSampleApp(argc, argv, &arguments, NULL, 1280, 800);

    // set window resize callback
    gWindow->setOnResizeWindowCallback(resizeWindowCallbackGrid);

    // init driveworks
    initSdk(&gSdk, gWindow);

#ifdef VIBRANTE
    gInputType = gArguments.get("input-type");
#else
    gInputType = "video";
#endif

    // create HAL and camera
    dwImageProperties rawImageProps;
    dwImageProperties rcbImageProps;
    dwCameraProperties cameraProps;
    bool sensorsInitialized = initSensors(&gSal, &gCameras, &rawImageProps, &cameraProps, gSdk);

    if (sensorsInitialized) {

        gImageWidth = rawImageProps.width;
        gImageHeight = rawImageProps.height;

        // Configure grid for N cameras rendering
        configureGrid(&gGrid, gWindow->width(), gWindow->height(), gImageWidth, gImageHeight, gNumCameras);

        // create 1 RCCB pipeline to handle all frames
        dwSoftISPHandle_t rawPipeline;
        dwSoftISPParams softISPParams;
        dwSoftISP_initParamsFromCamera(&softISPParams, cameraProps);
        dwSoftISP_initialize(&rawPipeline, softISPParams, gSdk);
        dwSoftISP_setCUDAStream(g_cudaStream, rawPipeline);
        dwSoftISP_getDemosaicImageProperties(&rcbImageProps, rawPipeline);

        if (initDriveNet(rcbImageProps, gSdk)) {
            initRenderer(rcbImageProps, &gRenderer, gSdk, gWindow);
            runPipeline(rawPipeline, rawImageProps, rcbImageProps, cameraProps.framerate);
        }

        dwSoftISP_release(&rawPipeline);
    }

    // release DW modules
    releaseGrid();
    release();

    // release framework
    releaseSampleApp();

    return 0;
}

//------------------------------------------------------------------------------
bool initSensors(dwSALHandle_t *sal, std::vector<Camera> *cameras, dwImageProperties *cameraImageProperties,
                 dwCameraProperties* cameraProperties, dwContextHandle_t context)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot initialize SAL: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    // create GMSL Camera interface
    bool ret = false;
    if (gInputType.compare("camera") == 0) {
        ret = initNCameras(cameras, *sal);
    }
    else{
        ret = initNVideos(cameras, *sal);
    }
    if(!ret)
        return ret;

    for(uint32_t i = 0; i < cameras->size(); ++i){
        dwSensorCamera_getImageProperties(cameraImageProperties, DW_CAMERA_RAW_IMAGE, (*cameras)[i].sensor);
        dwSensorCamera_getSensorProperties(cameraProperties, (*cameras)[i].sensor);

        if(gInputType.compare("camera") == 0){
            if(cameraProperties->rawFormat == DW_CAMERA_RAW_FORMAT_RCCB ||
               cameraProperties->rawFormat == DW_CAMERA_RAW_FORMAT_BCCR ||
               cameraProperties->rawFormat == DW_CAMERA_RAW_FORMAT_CRBC ||
               cameraProperties->rawFormat == DW_CAMERA_RAW_FORMAT_CBRC){

                std::cout << "Camera image with " << cameraProperties->resolution.x << "x"
                    << cameraProperties->resolution.y << " at " << cameraProperties->framerate << " FPS" << std::endl;
            }
            else{
                std::cerr << "Camera is not supported" << std::endl;

                return false;
            }
        }
    }

    return true;
}

//------------------------------------------------------------------------------
bool initNCameras(std::vector<Camera> *cameras, dwSALHandle_t sal)
{
    std::string selector = gArguments.get("selector-mask");

    dwStatus result;

    // identify active ports
    int idx             = 0;
    int cnt[3]          = {0, 0, 0};
    std::string port[3] = {"ab", "cd", "ef"};
    for (size_t i = 0; i < selector.length() && i < 12; i++, idx++) {
        const char s = selector[i];
        if (s == '1') {
            cnt[idx / 4]++;
        }
    }

    // how many cameras selected in a port
    for (size_t p = 0; p < 3; p++) {
        if (cnt[p] > 0) {
            std::string params;

            params += std::string("csi-port=") + port[p];
            params += ",camera-type=" + gArguments.get("camera-type");
            // when using the mask, just ask for all cameras, mask will select properly
            params += ",serialize=false,output-format=raw,camera-count=4";

            if (selector.size() >= p*4) {
                params += ",camera-mask="+ selector.substr(p*4, std::min(selector.size() - p*4, size_t{4}));
            }

            params += ",slave="  + gArguments.get("slave");

            params += ",fifo-size="  + gArguments.get("fifo-size");

            dwSensorHandle_t salSensor = DW_NULL_HANDLE;
            dwSensorParams salParams;
            salParams.parameters = params.c_str();
            salParams.protocol = "camera.gmsl";
            result = dwSAL_createSensor(&salSensor, salParams, sal);
            if (result == DW_SUCCESS) {
                Camera cam;
                cam.sensor = salSensor;

                dwImageProperties cameraImageProperties;
                dwSensorCamera_getImageProperties(&cameraImageProperties,
                                DW_CAMERA_PROCESSED_IMAGE,
                                salSensor);

                dwCameraProperties cameraProperties;
                dwSensorCamera_getSensorProperties(&cameraProperties, salSensor);
                cam.numSiblings = cameraProperties.siblings;

                cameras->push_back(cam);

                gNumCameras += cam.numSiblings;
            }
            else
            {
                std::cerr << "Cannot create driver: " << salParams.protocol
                    << " with params: " << salParams.parameters << std::endl
                    << "Error: " << dwGetStatusName(result) << std::endl;
                if (result == DW_INVALID_ARGUMENT) {
                    std::cerr << "It is possible the given camera is not supported. "
                              << "Please refer to the documentation for this sample."
                              << std::endl;
                }
                return false;
            }
        }
    }
    return true;
}

//------------------------------------------------------------------------------
bool initNVideos(std::vector<Camera> *cameras, dwSALHandle_t sal)
{
    dwStatus result;
    std::string videos = gArguments.get("videos");
    int idx = 0;
    static int count = 0;

    while(true){
        size_t found = videos.find(",", idx);

        Camera cam;
        dwSensorHandle_t salSensor = DW_NULL_HANDLE;
        dwSensorParams params;

        std::string parameterString = "video=" + videos.substr(idx, found - idx);
        params.parameters           = parameterString.c_str();
        params.protocol             = "camera.virtual";
        result                      = dwSAL_createSensor(&salSensor, params, sal);
        if (result != DW_SUCCESS) {
            std::cerr << "Cannot create driver: camera.virtual with params: " << params.parameters
                      << std::endl << "Error: " << dwGetStatusName(result) << std::endl;
            return false;
        }

        cam.sensor = salSensor;
        cam.numSiblings = 1;
        cameras->push_back(cam);
        ++gNumCameras;
        std::cout << "Initialize video " << count++ << std::endl;

        if(found == std::string::npos)
            break;
        idx = found + 1;
    }

    return true;
}

//------------------------------------------------------------------------------
bool initDriveNet(const dwImageProperties& rcbImageProps, dwContextHandle_t ctx)
{
    // in contrast to normal sample_drivenet as well as sample_drivenet_multigpu
    // we do not use 2 images per camera frame to perform DriveNet detections on
    // in this case we run each detection on 1 image only, thus treating each camera
    // image as and independent detection. Since the network supports batched input the throughput
    // should increase, however the detection quality might suffer

    dwDriveNetParams drivenetParams;
    dwDriveNet_initDefaultParams(&drivenetParams);
    drivenetParams.maxProposalsPerClass = gMaxProposalsPerClass;
    drivenetParams.maxClustersPerClass = gMaxClustersPerClass;

    // Initialize DriveNet with parameters
    dwStatus result = dwDriveNet_initialize(&gDriveNet, &gObjectClusterers, &gDriveNetClasses,
                                            &gNumClasses, ctx, &drivenetParams);
    if (result != DW_SUCCESS) {
        std::cerr << "Failed to create DriveNet" << std::endl;
        return false;
    }

    dwObjectDetectorDNNParams dnnParams{};
    dwObjectDetectorParams detectorParams{};
    dwObjectDetector_initDefaultParams(&dnnParams, &detectorParams);
    detectorParams.enableFuseObjects    = DW_FALSE;                 // no fusion from multiple images please
    detectorParams.maxNumImages         = gNumCameras;              // we feed all camera images at once
    result = dwObjectDetector_initializeFromDriveNet(&gDriveNetDetector, ctx, gDriveNet, &detectorParams);
    if (result != DW_SUCCESS) {
        std::cerr << "Failed to create Detector" << std::endl;
        return false;
    }

    result = dwObjectDetector_setCUDAStream(g_cudaStream, gDriveNetDetector);
    if (result != DW_SUCCESS) {
        std::cerr << "Failed to set CUDA stream for detector" << std::endl;
        return false;
    }



    // since our input images might have a different aspect ratio as the input to drivenet
    // we setup the ROI such that the crop happens from the top of the image
    float32_t aspectRatio = 1.0f;
    {
        dwBlobSize inputBlob;
        dwDriveNet_getInputBlobsize(&inputBlob, gDriveNet);

        aspectRatio = static_cast<float32_t>(inputBlob.height) / static_cast<float32_t>(inputBlob.width);
    }

    // Feed full image from each of the cameras
    for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx) {
        dwRect fullROI = {0, 0, static_cast<int32_t>(rcbImageProps.width),
                                static_cast<int32_t>(rcbImageProps.width * aspectRatio)};
        dwTransformation2D transformation = {{1.0f, 0.0f, 0.0f,
                                              0.0f, 1.0f, 0.0f,
                                              0.0f, 0.0f, 1.0f}};

        dwObjectDetector_setROI(camIdx, &fullROI, &transformation, gDriveNetDetector);
    }

    // setup feature tracker - we setup one feature tracker per camera
    // all feature trackers share same settings
    {
        dwObjectFeatureTrackerParams featureTrackingParams;
        dwObjectTrackerParams objectTrackingParams[DW_OBJECT_MAX_CLASSES];
        dwObjectTracker_initDefaultParams(&featureTrackingParams, objectTrackingParams, gNumClasses);
        featureTrackingParams.maxFeatureCount = 2000;
        featureTrackingParams.detectorScoreThreshold = 0.0001f;
        featureTrackingParams.iterationsLK = 10;
        featureTrackingParams.windowSizeLK = 8;

        for (uint32_t classIdx = 0U; classIdx < gNumClasses; ++classIdx) {
            objectTrackingParams[classIdx].confRateTrackMax = 0.05f;
            objectTrackingParams[classIdx].confRateDetect = 0.5f;
            objectTrackingParams[classIdx].confThreshDiscard = 0.0f;
            objectTrackingParams[classIdx].maxFeatureCountPerBox = 200;
        }

        for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx){
            result = dwObjectTracker_initialize(&(gObjectTracker[camIdx]), ctx, &rcbImageProps,
                                                &featureTrackingParams, objectTrackingParams, gNumClasses);
            if (result != DW_SUCCESS) {
                std::cerr << "Failed to create object tracker" << std::endl;
                return false;
            }
        }
    }

    // Get which label name for each class id
    // and setup default set of post clustering thresholds
    gClassLabels.resize(gNumClasses);
    for (uint32_t classIdx = 0U; classIdx < gNumClasses; ++classIdx) {
        const char *classLabel;
        dwDriveNet_getClassLabel(&classLabel, static_cast<dwDriveNetClass>(classIdx), gDriveNet);
        gClassLabels[classIdx] = classLabel;
    }

    // Initialize arrays for the pipeline
    for (uint32_t camIdx = 0U; camIdx < gNumCameras; ++camIdx){
        gObjectClusters[camIdx].resize(gNumClasses);
        gObjectsTracked[camIdx].resize(gNumClasses);
        gObjectsMerged[camIdx].resize(gNumClasses);
        gObjectProposals[camIdx].resize(gNumClasses);
        gNumClusters[camIdx].resize(gNumClasses);
        gNumProposals[camIdx].resize(gNumClasses);
        gNumMergedObjects[camIdx].resize(gNumClasses);
        gNumTrackedObjects[camIdx].resize(gNumClasses);
        for (uint32_t classIdx = 0U; classIdx < gNumClasses; ++classIdx) {
            gObjectsTracked[camIdx][classIdx].resize(gMaxClustersPerClass);
            gObjectsMerged[camIdx][classIdx].resize(gMaxClustersPerClass);
            gObjectClusters[camIdx][classIdx].resize(gMaxClustersPerClass);
            gObjectProposals[camIdx][classIdx].resize(gMaxProposalsPerClass);
        }
    }

    return true;
}

//------------------------------------------------------------------------------
void runPipeline(dwSoftISPHandle_t softISP, const dwImageProperties& rawImageProps,
                 const dwImageProperties& rcbImageProps, float32_t framerate)
{
    typedef std::chrono::high_resolution_clock myclock_t;
    typedef std::chrono::time_point<myclock_t> timepoint_t;
    auto frameDuration         = std::chrono::milliseconds((int)(1000 / framerate));
    timepoint_t lastUpdateTime = myclock_t::now();

    // crate RCB image for each camera, to be feeded as an array to drivenet
    std::vector<dwImageCUDA*> rcbArray(gNumCameras);
    {
        for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx) {
            rcbArray[camIdx] = new dwImageCUDA{};
            dwImageCUDA_create(rcbArray[camIdx], &rcbImageProps, DW_IMAGE_CUDA_PITCH);
        }
    }

    // RGBA image to display
    std::vector<dwImageCUDA*> rgbaImage(gNumCameras);
    {
        dwImageProperties rgbaImageProperties = rcbImageProps;
        rgbaImageProperties.pxlFormat         = DW_IMAGE_RGBA;
        rgbaImageProperties.pxlType           = DW_TYPE_UINT8;
        rgbaImageProperties.planeCount        = 1;
        for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx){
            rgbaImage[camIdx] = new dwImageCUDA{};
            dwImageCUDA_create(rgbaImage[camIdx], &rgbaImageProperties, DW_IMAGE_CUDA_PITCH);
        }
    }

    // Setup CUDA->GL streamer for producing images to render on screen
    dwImageStreamerHandle_t cuda2gl;
    {
        dwStatus result = dwImageStreamer_initialize(&cuda2gl, &rgbaImage[0]->prop, DW_IMAGE_GL, gSdk);
        if (result != DW_SUCCESS) {
            std::cerr << "Image streamer initialization failed: " << dwGetStatusName(result) << std::endl;
            gRun = false;
        }
    }

    // Initialize N streamers, where N is the number of ports used
    dwImageStreamerHandle_t input2cuda[gCameras.size()];
    for(uint32_t i = 0; i < gCameras.size(); ++i){
        dwImageStreamer_initialize(&input2cuda[i], &rawImageProps, DW_IMAGE_CUDA, gSdk);
    }

    uint32_t numClasses = gClassLabels.size();

    uint32_t frame = 0;
    uint32_t stopFrame = atoi(gArguments.get("stopFrame").c_str());
    uint32_t skipFrame = std::stoi(gArguments.get("skipFrame")) + 1;
    uint32_t skipInference = std::stoi(gArguments.get("skipInference")) + 1;

    for(uint32_t camNum = 0; camNum < gCameras.size(); ++camNum){
        gRun = gRun && dwSensor_start(gCameras[camNum].sensor) == DW_SUCCESS;
    }

    while (gRun && !gWindow->shouldClose()) {
        std::this_thread::yield();

        // run with at most 30FPS when the input is a video
        if (gInputType.compare("video") == 0) {
            std::chrono::milliseconds timeSinceUpdate =
                std::chrono::duration_cast<std::chrono::milliseconds>(myclock_t::now() - lastUpdateTime);

            if (timeSinceUpdate < frameDuration)
                continue;
            lastUpdateTime = myclock_t::now();
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        uint32_t countEndOfStream = 0;
        for(uint32_t camNum = 0, camIdx = 0; camNum < gCameras.size(); ++camNum){
            for(uint32_t sIdx = 0; sIdx < gCameras[camNum].numSiblings; ++sIdx, ++camIdx){

                // grab camera frame
                dwCameraFrameHandle_t frameHandle;
                {
                    dwStatus result = dwSensorCamera_readFrame(&frameHandle, sIdx, 2000000, gCameras[camNum].sensor);
                    if (result == DW_END_OF_STREAM) {
                        ++countEndOfStream;
                        continue;
                    }
                    else if (result != DW_SUCCESS) {
                        std::cerr << "Cannot read frame: " << dwGetStatusName(result) << std::endl;
                        continue;
                    }
                    else if(frame % skipFrame != 0){
                        result = dwSensorCamera_returnFrame(&frameHandle);
                        if (result != DW_SUCCESS) {
                            std::cerr << "Cannot return frame: " << dwGetStatusName(result) << std::endl;
                        }
                        continue;
                    }
                }


                // extract CUDA RAW image from the frame
                dwImageCUDA *rawImageCUDA;
                const dwImageDataLines* dataLines;
                {
                    dwStatus result = DW_SUCCESS;

                    // depending if we are grabbing from camera, we need to use nvmedia->cuda streamer
                    // or if we are grabbing from video, then we use cpu->cuda streamer

                    dwImageCPU *rawImageCPU;

                #ifdef VIBRANTE
                    dwImageNvMedia *rawImageNvMedia;

                    if (gInputType.compare("camera") == 0) {
                        result = dwSensorCamera_getImageNvMedia(&rawImageNvMedia, DW_CAMERA_RAW_IMAGE,
                                                                frameHandle);
                    }else
                #endif
                    {
                        result = dwSensorCamera_getImageCPU(&rawImageCPU, DW_CAMERA_RAW_IMAGE,
                                                            frameHandle);
                    }
                    if (result != DW_SUCCESS) {
                        std::cerr << "Cannot get raw image: " << dwGetStatusName(result) << std::endl;
                        continue;
                    }

                    result = dwSensorCamera_getDataLines(&dataLines, frameHandle);
                    if (result != DW_SUCCESS) {
                        std::cerr << "Cannot get datalines: " << dwGetStatusName(result) << std::endl;
                        continue;
                    }

                    // process
                #ifdef VIBRANTE
                    if (gInputType.compare("camera") == 0) {
                        result = dwImageStreamer_postNvMedia(rawImageNvMedia, input2cuda[camNum]);
                    }else
                #endif
                    {
                        result = dwImageStreamer_postCPU(rawImageCPU, input2cuda[camNum]);
                    }
                    if (result != DW_SUCCESS) {
                        std::cerr << "Cannot post CPU image: " << dwGetStatusName(result) << std::endl;
                        continue;
                    }

                    result = dwImageStreamer_receiveCUDA(&rawImageCUDA, 10000, input2cuda[camNum]);
                    if (result != DW_SUCCESS) {
                        std::cerr << "Cannot get receive CUDA image: " << dwGetStatusName(result) << std::endl;
                        continue;
                    }
                }

                // Raw -> RCB & RGBA
                {
                    dwSoftISP_bindRawInput(rawImageCUDA, softISP);
                    dwSoftISP_bindDemosaicOutput(rcbArray[camIdx], softISP);
                    dwSoftISP_bindTonemapOutput(rgbaImage[camIdx], softISP);
                    dwStatus result = dwSoftISP_processDeviceAsync(DW_SOFT_ISP_PROCESS_TYPE_DEMOSAIC | DW_SOFT_ISP_PROCESS_TYPE_TONEMAP,
                                                                   softISP);

                    if (result != DW_SUCCESS) {
                        std::cerr << "Cannot run rccb pipeline: " << dwGetStatusName(result) << std::endl;
                        gRun = false;
                        continue;
                    }
                }

                // return RAW CUDA image back, we habe now a copy through RawPipeline
                {
                    dwStatus result = dwImageStreamer_returnReceivedCUDA(rawImageCUDA, input2cuda[camNum]);
                    if (result != DW_SUCCESS) {
                        std::cerr << "Cannot return receive CUDA: " << dwGetStatusName(result) << std::endl;
                        continue;
                    }

                #ifdef VIBRANTE
                    if (gInputType.compare("camera") == 0) {
                        dwImageNvMedia* imageNvmedia = nullptr;
                        result = dwImageStreamer_waitPostedNvMedia(&imageNvmedia, 200000, input2cuda[camNum]);
                    }else
                #endif
                    {
                        dwImageCPU* cpuImage = nullptr;
                        result = dwImageStreamer_waitPostedCPU(&cpuImage, 200000, input2cuda[camNum]);
                    }
                    if (result != DW_SUCCESS) {
                        std::cerr << "Cannot wait posted: " << dwGetStatusName(result) << std::endl;
                        continue;
                    }
                }

                // camera frame is now free
                dwStatus result = dwSensorCamera_returnFrame(&frameHandle);
                if (result != DW_SUCCESS) {
                    std::cerr << "Cannot return frame: " << dwGetStatusName(result) << std::endl;
                    continue;
                }
            }
        }

        // If cameras reached end of stream, reset sensors and trackers
        if(countEndOfStream == gNumCameras){
            for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx){
                std::cout << "Camera " << camIdx << " reached end of stream" << std::endl;
                dwSensor_reset(gCameras[camIdx].sensor);
                dwObjectTracker_reset(gObjectTracker[camIdx]);
                for (uint32_t classIdx = 0U; classIdx < numClasses; ++classIdx) {
                    gNumClusters[camIdx][classIdx] = 0U;
                    gNumMergedObjects[camIdx][classIdx] = 0U;
                    gNumTrackedObjects[camIdx][classIdx] = 0U;
                }
            }
            frame = 0;
            gFrameInference = 0;

            continue;
        }

        // Skip frame
        if(frame % skipFrame != 0){
            ++frame;
            continue;
        }

        // Detect, track and render objects
        std::vector<std::pair<dwBox2D, std::string>> boxList[gNumCameras*numClasses];
        detectTrack(rcbArray.data(), skipInference, boxList);

        // Render received texture and bounding boxes
        for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx){

            // setup rendering rectangle for the current camera
            dwRect rect{};
            gridCellRect(&rect, gGrid, camIdx);
            dwRenderer_setRect(rect, gRenderer);

            // map CUDA RGBA frame to GL space
            dwImageGL *frameGL;
            dwImageStreamer_postCUDA(rgbaImage[camIdx], cuda2gl);
            dwStatus result = dwImageStreamer_receiveGL(&frameGL, 200000, cuda2gl);
            if( result == DW_SUCCESS ) {
                dwRenderer_renderTexture(frameGL->tex, frameGL->target, gRenderer);

                for (uint32_t classIdx = 0U; classIdx < numClasses; ++classIdx) {
                    dwRenderer_setColor(gBoxColors[classIdx % gMaxBoxColors], gRenderer);
                    drawBoxesWithLabels(boxList[camIdx*numClasses + classIdx],
                                        static_cast<float32_t>(rcbImageProps.width),
                                        static_cast<float32_t>(rcbImageProps.height),
                                        gLineBuffer, gRenderer);
                }

                dwImageStreamer_returnReceivedGL(frameGL, cuda2gl);
                dwImageCUDA *returnedFrame;
                dwImageStreamer_waitPostedCUDA(&returnedFrame, 200000, cuda2gl);
            }
        }

        gWindow->swapBuffers();

        ++frame;

        if(stopFrame && frame == stopFrame)
            break;
    }
    for(uint32_t camNum = 0; camNum < gCameras.size(); ++camNum) {
        dwSensor_stop(gCameras[camNum].sensor);
    }

    for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx){
        dwImageCUDA_destroy(rgbaImage[camIdx]);
        dwImageCUDA_destroy(rcbArray[camIdx]);
        delete rcbArray[camIdx];
        delete rgbaImage[camIdx];
    }

    // Release streamers
    for(uint32_t i = 0; i < gCameras.size(); ++i){
        dwImageStreamer_release(&input2cuda[i]);
    }
    dwImageStreamer_release(&cuda2gl);

}

//------------------------------------------------------------------------------
void detectTrack(dwImageCUDA** rcbArray, uint32_t skipInference,
                 std::vector<std::pair<dwBox2D, std::string>>* boxList)
{
    bool doInference = gFrameInference % skipInference == 0;
    uint32_t numClasses = gClassLabels.size();

    if(doInference){
        dwObjectDetector_inferDeviceAsync(rcbArray, gNumCameras, gDriveNetDetector);
        dwObjectDetector_interpretHost(gNumCameras, gDriveNetDetector);

        // Get detections and cluster them
        for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx){
            for (uint32_t classIdx = 0U; classIdx < numClasses; ++classIdx) {
                dwObjectDetector_getDetectedObjects(gObjectProposals[camIdx][classIdx].data(),
                                                    &(gNumProposals[camIdx][classIdx]),
                                                    camIdx, classIdx, gDriveNetDetector);

                dwObjectClustering_cluster(gObjectClusters[camIdx][classIdx].data(),
                                           &gNumClusters[camIdx][classIdx],
                                           gObjectProposals[camIdx][classIdx].data(),
                                           gNumProposals[camIdx][classIdx], gObjectClusterers[classIdx]);
            }
        }

    }
    ++gFrameInference;


    for(uint32_t camIdx = 0; camIdx < gNumCameras; ++camIdx){
        dwObjectTracker_featureTrackDeviceAsync(rcbArray[camIdx], gObjectTracker[camIdx]);

        for (uint32_t classIdx = 0U; classIdx < numClasses; ++classIdx) {

            std::vector<std::pair<dwBox2D, std::string>>& dnnBoxList = boxList[camIdx*numClasses + classIdx];
            dnnBoxList.clear();

            // Track previous boxes on current frame
            dwObjectTracker_boxTrackHost(gObjectsTracked[camIdx][classIdx].data(), &(gNumTrackedObjects[camIdx][classIdx]),
                                         gObjectsMerged[camIdx][classIdx].data(), gNumMergedObjects[camIdx][classIdx],
                                         classIdx, gObjectTracker[camIdx]);

            // If there is no inference do not merge, just copy the tracked objects for the next round
            const dwObject *toBeMerged[2] = {gObjectsTracked[camIdx][classIdx].data(), gObjectClusters[camIdx][classIdx].data()};
            const size_t sizes[2] = {gNumTrackedObjects[camIdx][classIdx], doInference ? gNumClusters[camIdx][classIdx] : 0};

            dwObject_merge(gObjectsMerged[camIdx][classIdx].data(), &(gNumMergedObjects[camIdx][classIdx]),
                           gMaxClustersPerClass, toBeMerged, sizes, 2U, 0.2f, 0.2f, gSdk);

            for (uint32_t objIdx = 0U; objIdx < gNumMergedObjects[camIdx][classIdx]; ++objIdx) {
                const dwObject &obj = gObjectsMerged[camIdx][classIdx][objIdx];
                dwBox2D box;
                box.x = static_cast<int32_t>(std::round(obj.box.x));
                box.y = static_cast<int32_t>(std::round(obj.box.y));
                box.width = static_cast<int32_t>(std::round(obj.box.width));
                box.height = static_cast<int32_t>(std::round(obj.box.height));

                dnnBoxList.push_back(std::make_pair(box, gClassLabels[obj.classId]));
            }
        }
    }
}

//------------------------------------------------------------------------------
void resizeWindowCallbackGrid(int width, int height) {
   configureGrid(&gGrid, width, height, gImageWidth, gImageHeight, gNumCameras);
}

//------------------------------------------------------------------------------
void releaseGrid()
{
    for(uint32_t i = 0; i < gNumCameras; ++i) {
        if (gObjectTracker[i]) dwObjectTracker_release(&(gObjectTracker[i]));
    }

    if (gObjectClusterers) {
        for (uint32_t clsIdx = 0; clsIdx < gNumClasses; ++clsIdx) {
            dwObjectClustering_release(&gObjectClusterers[clsIdx]);
        }
    }

    if (gDriveNetDetector) {
        dwObjectDetector_release(&gDriveNetDetector);
    }

    if (gDriveNet) {
        dwDriveNet_release(&gDriveNet);
    }

    if (gCameras.size()) {
        for(uint32_t i = 0; i < gCameras.size(); ++i)
            dwSAL_releaseSensor(&(gCameras[i].sensor));
    }

    if (gSal) {
        dwSAL_release(&gSal);
    }
}
