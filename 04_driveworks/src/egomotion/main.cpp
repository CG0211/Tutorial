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
// Copyright (c) 2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

// Core
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// Egomotion
#include <dw/egomotion/Egomotion.h>

// RigConfiguration
#include <dw/rigconfiguration/RigConfiguration.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>
#include <dw/sensors/imu/IMU.h>
#include <dw/sensors/gps/GPS.h>

// Renderer
#include <dw/renderer/Renderer.h>

//Sensor Manager
#include <dw/sensors/sensormanager/SensorManager.h>

// Sample Includes
#include <framework/Log.hpp>
#include <framework/DataPath.hpp>
#include <framework/MathUtils.hpp>
#include <framework/Mat4.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/Checks.hpp>

#include <condition_variable>
#include <thread>
#include <list>
#include <mutex>
#include <string>
#include <vector>

#include <stdio.h>
#include <string.h>


// driveworks variables
dwContextHandle_t gSDK                  = DW_NULL_HANDLE;
dwEgomotionHandle_t gEgomotion          = DW_NULL_HANDLE;
dwSALHandle_t gSAL                      = DW_NULL_HANDLE;
dwCANInterpreterHandle_t gCanParser     = DW_NULL_HANDLE;
dwSensorManagerHandle_t gSensorManager  = DW_NULL_HANDLE;

dwEgoMotionParameters  gEgomotionParameters{};

// current sensor state
dwIMUFrame      gCurrentIMUFrame        = {};
dwCANMessage    gCurrentCANFrame        = {};
dwGPSFrame      gCurrentGPSFrame        = {};

// render variables
dwRendererHandle_t gRenderer                     = DW_NULL_HANDLE;
dwRenderBufferHandle_t gLineBuffer               = DW_NULL_HANDLE;
dwRenderBufferHandle_t gLineBufferRigCoordinates = DW_NULL_HANDLE;
bool gEnableRendering                    = false;
const uint32_t gWindowWidth              = 1024;
const uint32_t gWindowHeight             = 768;
const uint32_t gMaxPointCount            = 100000;
const float32_t gRigCoordinateLineLength = 100;

// sample variables
const dwTime_t gPoseSampleRate      = 33333; // sample frequency of the filter in [usecs]
const dwTime_t gMotionUpdateRate    = 10000; // update frequency of the filter in [usecs]
std::vector<dwTransformation> gRig2WorldSamples;
FILE *gOutputFile = nullptr;

std::string gOutputKmlFile;

std::vector<dwVector3d> gGPSPathData;
std::vector<dwVector3d> gGPSPathEstimated;

// CAN parsing
std::string gCanSpeedName         = "Steering_Report.SPEED";
std::string gCanSteeringAngleName = "Steering_Report.ANGLE";
float32_t gCanSpeedFactor         = 0.277778f;      // kph -> m/s
float32_t gCanSteeringFactor      = 0.001179277f;   // steeringWheelAngle -> steeringAngle = deg2rad(a/14.8)

//------------------------------------------------------------------------------
// initializes
// - the rendering module
// - the render buffers
// - projection and modelview matrices
// - renderer settings
void initRendering()
{
    if (!gEnableRendering)
        return;

    CHECK_DW_ERROR( dwRenderer_initialize(&gRenderer, gSDK) );

    // render buffer to render all lines
    dwRenderBufferVertexLayout layout;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    dwRenderBuffer_initialize(&gLineBuffer, layout, DW_RENDER_PRIM_LINELIST, 2 * gMaxPointCount, gSDK);

    // render buffer to render car coordiante system
    dwRenderBuffer_initialize(&gLineBufferRigCoordinates, layout, DW_RENDER_PRIM_LINELIST, 4, gSDK);

    dwRect rect;
    rect.x      = 0;
    rect.y      = 0;
    rect.width  = gWindowWidth;
    rect.height = gWindowHeight;
    dwRenderer_setRect(rect, gRenderer);

    const float32_t white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    dwRenderer_setColor(white, gRenderer);
    dwRenderer_setLineWidth(1, gRenderer);
}

//------------------------------------------------------------------------------
// fills the render buffer with positions out of the poses array
void fillRenderBuffers(const dwTransformation *poses, uint32_t poseCount)
{
    if (!gEnableRendering || poseCount == 0)
        return;

    float32_t spanX[2] = {1e6, -1e6};
    float32_t spanY[2] = {1e6, -1e6};
    dwVector2f meanPose{0,0};

    // setup path rendering
    {
        float32_t *buffer;
        uint32_t stride      = 0;
        uint32_t maxVertices = 0;
        dwRenderBuffer_map(&buffer, &maxVertices, &stride, gLineBuffer);
        float32_t *startBuffer = buffer;

        uint32_t vertCount = 0;
        for (uint32_t i = 1; i < poseCount; ++i) {
            buffer[0] = poses[i - 1].array[0 + 3 * 4];
            buffer[1] = poses[i - 1].array[1 + 3 * 4];
            buffer[2] = 0.0f;
            buffer += stride;
            if (++vertCount >= maxVertices)
                break;

            buffer[0] = poses[i].array[0 + 3 * 4];
            buffer[1] = poses[i].array[1 + 3 * 4];
            buffer[2] = 0.0f;
            buffer += stride;
            if (++vertCount >= maxVertices)
                break;
        }

        // find dimensions of the route to accomodate renderwindow size to the path
        for (uint32_t i = 1; i < vertCount; ++i)
        {
            const dwVector2f* pt = reinterpret_cast<const dwVector2f*>(startBuffer);

            spanX[0] = std::min(spanX[0], pt->x);
            spanX[1] = std::max(spanX[1], pt->x);
            spanY[0] = std::min(spanY[0], pt->y);
            spanY[1] = std::max(spanY[1], pt->y);

            meanPose.x += pt->x;
            meanPose.y += pt->y;

            startBuffer += stride;
        }
        meanPose.x /= vertCount;
        meanPose.y /= vertCount;

        dwRenderBuffer_unmap(vertCount, gLineBuffer);
    }

    // change rendering dimension to fit whole path on the screen
    {
        // scale based on the bounding box size
        float32_t sx = spanX[1] - spanX[0];
        float32_t sy = spanY[1] - spanY[0];
        float32_t size = std::max(1500.f, std::max(sx,sy));

        // move to center
        float32_t windowAspect = static_cast<float>(gWindowWidth) / gWindowHeight;
        float32_t cx = -meanPose.x;
        float32_t cy = -meanPose.y;

        // column-major
        float32_t modelview[16] =
                             {1,0,0,0,
                              0,1,0,0,
                              0,0,1,0,
                              cx,cy,0,1};

        // orthographic projection for rendering
        float32_t projection[16];
        ortho(projection, -size, size, -size / windowAspect, size / windowAspect, 0, 1);

        dwRenderer_setModelView(modelview, gRenderer);
        dwRenderer_setProjection(projection, gRenderer);
    }

    // setup coordinate system rendering
    {
        const dwTransformation& currentRig2World = poses[poseCount-1];

        float32_t *buffer;
        uint32_t stride      = 0;
        uint32_t maxVertices = 0;
        dwRenderBuffer_map(&buffer, &maxVertices, &stride, gLineBufferRigCoordinates);

        float32_t localOrigin[3] = {0,0,0};
        float32_t localX[3] = {gRigCoordinateLineLength,0,0};
        float32_t localY[3] = {0,gRigCoordinateLineLength,0};
        float32_t worldOrigin[3];
        float32_t worldX[3];
        float32_t worldY[3];

        Mat4_Axp(worldOrigin, currentRig2World.array, localOrigin);
        Mat4_Axp(worldX, currentRig2World.array, localX);
        Mat4_Axp(worldY, currentRig2World.array, localY);

        // x-axis
        {
            buffer[0] = worldOrigin[0];
            buffer[1] = worldOrigin[1];
            buffer[2] = 0.0f;
            buffer += stride;

            buffer[0] = worldX[0];
            buffer[1] = worldX[1];
            buffer[2] = 0.0f;
            buffer += stride;
        }

        // y-axis
        {
            buffer[0] = worldOrigin[0];
            buffer[1] = worldOrigin[1];
            buffer[2] = 0.0f;
            buffer += stride;

            buffer[0] = worldY[0];
            buffer[1] = worldY[1];
            buffer[2] = 0.0f;
            buffer += stride;
        }

        dwRenderBuffer_unmap(4, gLineBufferRigCoordinates);
    }
}

//------------------------------------------------------------------------------
// renders the prepared render buffers
void render()
{
    if (!gEnableRendering)
        return;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, gRenderer);
    dwRenderer_renderBuffer(gLineBuffer, gRenderer);

    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, gRenderer);
    dwRenderer_renderBuffer(gLineBufferRigCoordinates, gRenderer);

    if (gEgomotionParameters.motionModel == DW_EGOMOTION_ODOMETRY)
    {
        dwRenderer_renderText(10, gWindowHeight - 100, "ODOMETRY", gRenderer);
    }else if (gEgomotionParameters.motionModel == DW_EGOMOTION_IMU_ODOMETRY)
    {
        dwRenderer_renderText(10, gWindowHeight - 100, "ODOMETRY+IMU", gRenderer);
    }else if (gEgomotionParameters.motionModel == DW_EGOMOTION_IMU_ODOMETRY_GPS)
    {
        dwRenderer_renderText(10, gWindowHeight - 100, "ODOMETRY+IMU+GPS", gRenderer);
    }

    // render current motion estimates
    {
        char sbuffer[128];

        dwEgomotionResult state{};
        dwEgomotion_getEstimation(&state, gEgomotion);

        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_32, gRenderer);

        if ((state.validFlags & DW_EGOMOTION_VEL_X) != 0 && (state.validFlags & DW_EGOMOTION_VEL_X) != 0)
        {
            // we want speed measured on the ground
            float32_t speed = sqrt(state.linearVelocity[0] * state.linearVelocity[0] + state.linearVelocity[1] * state.linearVelocity[1]);

            // speed
            sprintf(sbuffer, "speed: %2.2f m/s (%2.2f km/h)", speed, speed * 3.6);
        }else{
            sprintf(sbuffer, "speed: not supported");
        }
        dwRenderer_renderText(10, 100, sbuffer, gRenderer);

        float32_t roll,pitch,yaw;
        quaternionToEulerianAngle(state.rotation, roll, pitch, yaw);

        // ROLL
        if ((state.validFlags & DW_EGOMOTION_ROLL) != 0)
        {
            if ((state.validFlags & DW_EGOMOTION_GYR_X) != 0)
            {
                sprintf(sbuffer, "ROLL: %2.2f deg, dX: %2.2f deg/s", roll * 180. / M_PI, state.angularVelocity[0] * 180. / M_PI);
            }else
            {
                sprintf(sbuffer, "ROLL: %2.2f deg", roll * 180. / M_PI);
            }
            dwRenderer_renderText(10, 70, sbuffer, gRenderer);
        }

        // PITCH
        if ((state.validFlags & DW_EGOMOTION_PITCH) != 0)
        {
            if ((state.validFlags & DW_EGOMOTION_GYR_Y) != 0)
            {
                sprintf(sbuffer, "PITCH: %2.2f deg, dY: %2.2f deg/s", pitch * 180. / M_PI, state.angularVelocity[1] * 180. / M_PI);
            }else
            {
                sprintf(sbuffer, "PITCH: %2.2f deg", pitch * 180. / M_PI);
            }
            dwRenderer_renderText(10, 40, sbuffer, gRenderer);
        }

        // YAW
        if ((state.validFlags & DW_EGOMOTION_YAW) != 0)
        {
            if ((state.validFlags & DW_EGOMOTION_GYR_Z) != 0)
            {
                sprintf(sbuffer, "YAW: %2.2f deg, dZ: %2.2f deg/s", yaw * 180. / M_PI, state.angularVelocity[2] * 180. / M_PI);
            }else
            {
                sprintf(sbuffer, "YAW: %2.2f deg", yaw * 180. / M_PI);
            }
            dwRenderer_renderText(10, 10, sbuffer, gRenderer);
        }
    }

    gWindow->swapBuffers();

    //std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

//------------------------------------------------------------------------------
void writeKml(const std::string filename, const std::vector<dwVector3d>& gpsPath)
{
    if( filename.length() == 0 )
        return;

    FILE* file = fopen(filename.c_str(), "wt");

    if (!file)
    {
        printf("Cannot open KML file for write: %s\n", filename.c_str());
    }

    std::string header = R"(<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
   <name>trajectory</name>
   <Style id="myLineStyle">
    <LineStyle>
     <color>700000ff</color>
     <width>3</width>
    </LineStyle>
    <PolyStyle>
     <color>700000ff</color>
    </PolyStyle>
   </Style>
   <Placemark>
    <name>absolute</name>
    <visibility>1</visibility>
    <description>Line</description>
    <styleUrl>#myLineStyle</styleUrl>
    <LineString>
     <tessellate>1</tessellate>
     <altitudeMode>clampToGround</altitudeMode>
     <coordinates>)";

    fprintf(file, "%s\n", header.c_str());

    for (size_t i=0; i < gpsPath.size(); i++)
    {
        fprintf(file, "\t%0.15f,%0.15f,%0.15f\n", gpsPath[i].x, gpsPath[i].y, gpsPath[i].z);
    }
    fprintf(file, "     </coordinates>\n</LineString>\n</Placemark>\n");

    // mark BEGIN and END
    if (gpsPath.size() >= 1)
    {
        dwVector3d first = *gpsPath.begin();
        dwVector3d last  = *gpsPath.rbegin();

        fprintf(file, "<Placemark><name>BEGIN</name><Point>\n");
        fprintf(file, "<coordinates>%0.15f,%0.15f,%0.15f</coordinates>\n", first.x, first.y, first.z);
        fprintf(file, "</Point></Placemark>\n");

        fprintf(file, "<Placemark><name>END</name><Point>\n");
        fprintf(file, "<coordinates>%0.15f,%0.15f,%0.15f</coordinates>\n", last.x, last.y, last.z);
        fprintf(file, "</Point></Placemark>\n");
    }

    fprintf(file, "</Document></kml>\n");
    fclose(file);
}

//------------------------------------------------------------------------------
void initRig(std::string rigFile)
{
    dwRigConfigurationHandle_t rigConfig;
    dwTransformation i2r{};
    dwStatus ret;
    const dwVehicle *vehicle;
    uint32_t imuId;

    ret = dwRigConfiguration_initializeFromFile(&rigConfig, gSDK, rigFile.c_str());
    if (ret != DW_SUCCESS)
      throw std::runtime_error("Error reading rig config");

    ret = dwRigConfiguration_findSensorByName(&imuId, "imu*", rigConfig);
    if (ret != DW_SUCCESS)
        throw std::runtime_error("Could not find imu sensor in rig configuration");

    ret = dwRigConfiguration_getVehicle(&vehicle, rigConfig);
    if (ret != DW_SUCCESS)
      throw std::runtime_error("Error reading vehicle data");

    ret = dwRigConfiguration_getSensorToRigTransformation(&i2r, imuId, rigConfig);
    if (ret != DW_SUCCESS)
      throw std::runtime_error("Error reading imu to rig transform");

    gEgomotionParameters.imu2rig = i2r;
    gEgomotionParameters.wheelBase = vehicle->wheelbase;

    gCanSteeringFactor = M_PI / (180. * vehicle->steeringCoefficient);

    dwRigConfiguration_release(&rigConfig);
}

//------------------------------------------------------------------------------
void initialize(std::string canfile, std::string imufile, std::string gpsfile,
                std::string dbcfile, std::string dbcSpeedName, std::string dbcSteeringAngleName,
                std::string speedFactor, std::string steeringFactor,
                std::string output, std::string outputkml,
                std::string rig, std::string mode)
{
    // init window
    gEnableRendering = initSampleApp(0, nullptr, nullptr, nullptr, gWindowWidth, gWindowHeight);

    dwStatus status = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (status != DW_SUCCESS) {
        printf("Error dwLogger_initialize: %s\n", dwGetStatusName(status));
        exit(-1);
    }

    dwLogger_setLogLevel(DW_LOG_WARN);

    dwContextParameters contextParams{};

    // initialize Driveworks SDK
    status = dwInitialize(&gSDK, DW_VERSION, &contextParams);
    if (status != DW_SUCCESS) {
        printf("Error dwInitialize: %s\n", dwGetStatusName(status));
        exit(-1);
    }

    // init renderer
    initRendering();
    initRig(rig);

    // initialize Egomotion module
    if (mode == "0")
        gEgomotionParameters.motionModel = DW_EGOMOTION_ODOMETRY;
    else if (mode == "1")
        gEgomotionParameters.motionModel = DW_EGOMOTION_IMU_ODOMETRY;
    else if (mode == "2")
        gEgomotionParameters.motionModel = DW_EGOMOTION_IMU_ODOMETRY_GPS;
    else {
        printf("invalid mode %s\n", mode.c_str());
        exit(-1);
    }

    status            = dwEgomotion_initialize(&gEgomotion, &gEgomotionParameters, gSDK);
    if (status != DW_SUCCESS) {
        printf("Error dwEgomotion_initialize: %s\n", dwGetStatusName(status));
        exit(-1);
    }

    // create HAL module of the SDK
    dwSAL_initialize(&gSAL, gSDK);

    // initialize Driveworks SensorManager
    status = dwSensorManager_initialize(&gSensorManager, 16, gSAL);
    if (status != DW_SUCCESS) {
        printf("Error in initializing SensorManager: %s\n", dwGetStatusName(status));
        dwSensorManager_stop(gSensorManager);
        dwSensorManager_release(&gSensorManager);
        exit(-1);

    }

    // create CAN bus interface
    {
        std::string parameterString = std::string("file=") + canfile;
        dwSensorParams params;
        params.protocol   = "can.virtual";
        params.parameters = parameterString.c_str();
        if(dwSensorManager_addSensor(params, 0, gSensorManager) != DW_SUCCESS) {
            printf("Cannot create s%s with %s\n", params.protocol, params.parameters);
            dwSensorManager_release(&gSensorManager);
            exit(-1);
        }
    }

    // if interpreter is provided, create an instance of it
    dwStatus result = DW_FAILURE;
    {
        result = dwCANInterpreter_buildFromDBC(&gCanParser, dbcfile.c_str(), gSDK);
        if (result != DW_SUCCESS) {
            printf("Cannot create dbc based CAN message interpreter\n");
        }
        gCanSpeedName         = dbcSpeedName;
        gCanSteeringAngleName = dbcSteeringAngleName;
    }


    // create IMU interface - if supported by the mode
    if (gEgomotionParameters.motionModel >= DW_EGOMOTION_IMU_ODOMETRY)
    {
        std::string parameterString = std::string("file=") + imufile;
        dwSensorParams params;
        params.protocol   = "imu.virtual";
        params.parameters = parameterString.c_str();
        if (dwSensorManager_addSensor(params, 0, gSensorManager) != DW_SUCCESS) {
            printf("Cannot create sensor %s with %s\n", params.protocol, params.parameters);
            dwSensorManager_release(&gSensorManager);
            exit(-1);
        }
    }

    // create GPS interface - if supported by the mode
    if (gEgomotionParameters.motionModel >= DW_EGOMOTION_IMU_ODOMETRY_GPS)
    {
        std::string parameterString = std::string("file=") + gpsfile;
        dwSensorParams params;
        params.protocol   = "gps.virtual";
        params.parameters = parameterString.c_str();
        if (dwSensorManager_addSensor(params, 0, gSensorManager) != DW_SUCCESS) {
            printf("Cannot create sensor %s with %s\n", params.protocol, params.parameters);
            dwSensorManager_release(&gSensorManager);
            exit(-1);
        }
    }
    if (dwSensorManager_start(gSensorManager) != DW_SUCCESS) {
          printf("SensorManager start failed\n");
          dwSensorManager_release(&gSensorManager);
          exit(-1);
    }
    // output file which contains positions
    if( output.length() > 0 ) {
        gOutputFile = fopen(output.c_str(), "wt");
        printf("Cannot open file for write: %s\n", output.c_str());
    }

    gOutputKmlFile = outputkml;
    gCanSpeedFactor    = static_cast<float32_t>(std::atof(speedFactor.c_str()));
    gCanSteeringFactor = static_cast<float32_t>(std::atof(steeringFactor.c_str()));
}

//------------------------------------------------------------------------------
void release()
{
    if (gOutputKmlFile.length() && gGPSPathData.size())
    {
        writeKml(gOutputKmlFile + "_data.kml", gGPSPathData);
    }
    if (gOutputKmlFile.length() && gGPSPathEstimated.size())
    {
        writeKml(gOutputKmlFile + "_estimated.kml", gGPSPathEstimated);
    }

    if (gEnableRendering) {
        dwRenderBuffer_release(&gLineBuffer);
        dwRenderBuffer_release(&gLineBufferRigCoordinates);
        dwRenderer_release(&gRenderer);
    }

    if (gEgomotion) {
        dwEgomotion_release(&gEgomotion);
    }
    if (gSDK) {
        dwRelease(&gSDK);
    }
    dwLogger_release();

    if (gCanParser != DW_NULL_HANDLE)
        dwCANInterpreter_release(&gCanParser);

    dwSensorManager_stop(gSensorManager);
    dwSensorManager_release(&gSensorManager);

    dwSAL_release(&gSAL);
    if (gOutputFile) {
        fclose(gOutputFile);
    }

    releaseSampleApp();
}

//------------------------------------------------------------------------------
bool parseCANMessage(const dwCANMessage &msg, float32_t &speed, float32_t &steeringAngle, bool &hasSpeed, bool &hasSteering)
{
    hasSpeed = false;
    hasSteering = false;

    dwCANInterpreter_consume(&msg, gCanParser);

    // check between all available signals for consumption
    uint32_t numSignals = 0;
    if (dwCANInterpreter_getNumberSignals(&numSignals, gCanParser) == DW_SUCCESS)
    {
        for (uint32_t i=0; i < numSignals; i++) {
            const char* name = nullptr;
            float32_t value = 0;
            dwTime_t ts;

            dwCANInterpreter_getSignalName(&name, i, gCanParser);
            dwCANInterpreter_getf32(&value, &ts, i, gCanParser);

            if (0 == strcmp(name, gCanSpeedName.c_str()))
            {
                speed = value * gCanSpeedFactor;
                hasSpeed = true;
            }else if (0 == strcmp(name, gCanSteeringAngleName.c_str())) {
                steeringAngle = value * gCanSteeringFactor;
                hasSteering = true;
            }
        }
    }

    return hasSteering || hasSpeed;
}

//------------------------------------------------------------------------------
void loop()
{
    dwTransformation I = {1,0,0,0,
                          0,1,0,0,
                          0,0,1,0,
                          0,0,0,1};

    dwTime_t lastUpdateTimestamp = 0;
    dwTime_t lastSampleTimestamp = 0;

    while(gRun && !gWindow->shouldClose())
    {
        const dwSensorEvent *acquiredEvent = nullptr;
        auto status = dwSensorManager_acquireNextEvent(&acquiredEvent, 0, gSensorManager);

        if (status == DW_TIME_OUT)
            continue;

        if (status != DW_SUCCESS ) {
            if (status != DW_END_OF_STREAM)
                printf("Error reading sensor %s\n", dwGetStatusName(status));
            gRun = false;
            break;
        }

        // indicator that we had a valid update of the pose
        dwTime_t egoMotionUpdateEventTimestamp = 0;
        dwTime_t timestamp = acquiredEvent->timestamp_us;

        // parse and push the new measurment to the ego motion module
        switch (acquiredEvent->type)
        {
        case DW_SENSOR_CAN:
        {
            gCurrentCANFrame = acquiredEvent->canFrame;
            bool hasSpeed      = false;
            bool hasSteering   = false;
            float32_t speed    = 0;
            float32_t steering = 0;

            if (parseCANMessage(gCurrentCANFrame, speed, steering, hasSpeed, hasSteering))
            {
                if (hasSteering) {
                    dwStatus status = dwEgomotion_addOdometry(DW_EGOMOTION_MEASURMENT_STEERINGANGLE, steering, gCurrentCANFrame.timestamp_us, gEgomotion);
                    if (status == DW_SUCCESS) {
                        egoMotionUpdateEventTimestamp = gCurrentCANFrame.timestamp_us;
                    }
                }

                if (hasSpeed) {
                    dwStatus status = dwEgomotion_addOdometry(DW_EGOMOTION_MEASURMENT_VELOCITY, speed, gCurrentCANFrame.timestamp_us, gEgomotion);
                    if (status == DW_SUCCESS) {
                        egoMotionUpdateEventTimestamp = gCurrentCANFrame.timestamp_us;
                    }
                }

            }

            // indicate that this CAN message has been consumed
            gCurrentCANFrame.timestamp_us = 0;
            break;
        }

        case DW_SENSOR_IMU:
        {
            gCurrentIMUFrame = acquiredEvent->imuFrame;
            dwStatus status = dwEgomotion_addIMUMeasurement(&gCurrentIMUFrame, gEgomotion);
            if (status == DW_SUCCESS) {
                egoMotionUpdateEventTimestamp = gCurrentIMUFrame.timestamp_us;
                gCurrentIMUFrame.timestamp_us = 0;
            }

            // disable IMU readings if motion model does not support them
            if (status == DW_NOT_SUPPORTED) {
                gCurrentIMUFrame.timestamp_us = static_cast<dwTime_t>(-1);
            }
            break;
        }

        case DW_SENSOR_GPS:
        {
            gCurrentGPSFrame = acquiredEvent->gpsFrame;
            gGPSPathData.push_back(dwVector3d{gCurrentGPSFrame.longitude, gCurrentGPSFrame.latitude, gCurrentGPSFrame.altitude});

            dwStatus status = dwEgomotion_addGPSMeasurement(&gCurrentGPSFrame, gEgomotion);
            if (status == DW_SUCCESS) {
                egoMotionUpdateEventTimestamp = gCurrentGPSFrame.timestamp_us;
                gCurrentGPSFrame.timestamp_us = 0;
            }

            // disable GPS readings if motion model does not support them
            if (status == DW_NOT_SUPPORTED) {
                gCurrentGPSFrame.timestamp_us = static_cast<dwTime_t>(-1);
            }
            break;
        }

        default: break;
        }

        dwSensorManager_releaseAcquiredEvent(acquiredEvent, gSensorManager);

        // update the model at roughly 100Hz
        if (egoMotionUpdateEventTimestamp && lastUpdateTimestamp + gMotionUpdateRate <= timestamp)
        {
            dwEgomotion_update(timestamp, gEgomotion);
            lastUpdateTimestamp = timestamp;
        }

        // sample the actual pose at a camera rate (render it or output to the file)
        dwTime_t ts;
        if (DW_SUCCESS == dwEgomotion_getEstimationTimestamp(&ts, gEgomotion))
        {
            if (ts >= lastSampleTimestamp + gPoseSampleRate)
            {
                // extract relative pose between last and the current timestamp
                dwTransformation rigLast2rigNow;
                if (DW_SUCCESS == dwEgomotion_computeRelativeTransformation(&rigLast2rigNow, lastSampleTimestamp, ts, gEgomotion))
                {
                    // pose from last frame
                    const dwTransformation& rigLast2world = gRig2WorldSamples.empty() ? I : gRig2WorldSamples.back();

                    // compute absolute pose given the relative motion between two last estimates
                    dwTransformation rigNow2World;

                    dwEgomotion_applyRelativeTransformation(&rigNow2World, &rigLast2rigNow, &rigLast2world);

                    gRig2WorldSamples.push_back(rigNow2World);

                    if( gOutputFile )
                        fprintf(gOutputFile, "%lu,%.2f,%.2f\n", egoMotionUpdateEventTimestamp, rigNow2World.array[0 + 3 * 4], rigNow2World.array[1 + 3 * 4]);

                    fillRenderBuffers(gRig2WorldSamples.data(), static_cast<uint32_t>(gRig2WorldSamples.size()));
                    render();
                }
                lastSampleTimestamp = ts;

                // get current estimation
                dwEgomotionResult result;
                dwEgomotion_getEstimation(&result, gEgomotion);

                if (gEgomotionParameters.motionModel >= DW_EGOMOTION_IMU_ODOMETRY_GPS)
                {
                    gGPSPathEstimated.push_back(dwVector3d{result.location[0], result.location[1], result.location[2]});
                }
            }
        }
    }
}

//------------------------------------------------------------------------------
int main(int argc, const char *argv[])
{
    char defSpeedFactor[32] = {};
    char defSteeringFactor[32] = {};
    sprintf(defSpeedFactor, "%0.7f", gCanSpeedFactor);
    sprintf(defSteeringFactor, "%0.7f", gCanSteeringFactor);

    ProgramArguments arguments(
        {
            ProgramArguments::Option_t("canfile",
                                       (DataPath::get() + "/samples/egomotion/can_vehicle.bin").c_str()),
            ProgramArguments::Option_t("imufile",
                                       (DataPath::get() + "/samples/egomotion/imu_xsens.bin").c_str()),
            ProgramArguments::Option_t("gpsfile",
                                       (DataPath::get() + "/samples/egomotion/gps_xsens.bin").c_str()),
            ProgramArguments::Option_t("dbcfile",
                                       (DataPath::get() + "/samples/mapping/occupancy_grid/DataspeedByWire.dbc").c_str()),
            ProgramArguments::Option_t("dbcSpeed", gCanSpeedName.c_str()),
            ProgramArguments::Option_t("dbcSteeringAngle", gCanSteeringAngleName.c_str()),
            ProgramArguments::Option_t("speedFactor", defSpeedFactor),
            ProgramArguments::Option_t("steeringFactor", defSteeringFactor),
            ProgramArguments::Option_t("output", ""),
            ProgramArguments::Option_t("outputkml", ""),
            ProgramArguments::Option_t("rig", (DataPath::get() + "/samples/egomotion/rig.xml").c_str()),
            ProgramArguments::Option_t("mode", "1"),

                });

    if (!arguments.parse(argc, argv))
    {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << R"(
    --canfile:            The CAN file to read.
    --imufile:            IMU file recorded together with CAN data.
    --gpsfile:            GPS file recorded together with CAN data.

    --dbcfile:            Specify a DBC file to build the CAN parser.
                          If none is specified, the built in plugin
                          will be used.

    --dbcSpeed:           This is the name of the speed key in the DBC file.

    --dbcSteeringAngle:   This is the name of the steering angle key in the DBC file.

    --output:             If specified, the odometery will be output to this file.
    --outputkml:          If specified, estimated GPS location will be output into this file

    --speedFactor:        The factor is applied on the spped measurement from the CAN data after
                          being interpreted using the DBC file.

    --steeringFactor:     Same as speedFactor but applies to the steering angle.

    --rig:                Rig file ocntaining all information about vehicle sensors and calibration.

    --mode:               0=Ackerman motion, 1=IMU+Odometry, 2=IMU+Odometry+GPS

                          )";
        std::cout << std::endl;

        return -1; // Exit if not all require arguments are provided
    }

    initialize(arguments.get("canfile"), arguments.get("imufile"), arguments.get("gpsfile"),
               arguments.get("dbcfile"),
               arguments.get("dbcSpeed"), arguments.get("dbcSteeringAngle"),
               arguments.get("speedFactor"), arguments.get("steeringFactor"),
               arguments.get("output"), arguments.get("outputkml"),
               arguments.get("rig"), arguments.get("mode"));
    loop();

    // run until user exits application
    while (gEnableRendering && gRun && !gWindow->shouldClose()) {
        render();
    }

    release();

    return 0;
}
