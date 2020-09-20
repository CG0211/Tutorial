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

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <sstream>
#include <cmath>

#include <execinfo.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <thread>

#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif
#include <framework/ProgramArguments.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/Log.hpp>
#include <framework/DataPath.hpp>
#include <framework/Checks.hpp>

#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/sensors/Sensors.h>
#include <dw/renderer/Renderer.h>
#include <dw/control/vehicleio/VehicleIO.h>

#define DEG2RAD(x) (static_cast<float32_t>(x) * M_PI / 180.0f)

static dwContextHandle_t gSdk = DW_NULL_HANDLE;
static dwRendererHandle_t gRenderer = DW_NULL_HANDLE;
static dwRigConfigurationHandle_t gRig = DW_NULL_HANDLE;
static dwSALHandle_t gHal = DW_NULL_HANDLE;
static int gFrameWidth = 1280;
static int gFrameHeight = 800;
static dwVehicleIOCommand cmd{};
static dwVehicleIOState vehicleState{};

static void resetCmd()
{
    cmd.enable          = false;
    cmd.clearFaults     = false;
    cmd.steering        = 0.0f;
    cmd.steeringSpeed   = M_PI;
    cmd.throttlePercent = 0.0;
    cmd.throttleValue   = 0.0;
    cmd.brakePercent    = 0.0;
    cmd.brakeValue      = 0.0f;
    cmd.brakeTorque     = 0.0f;
    cmd.gear            = DW_VEHICLEIO_GEAR_UNKNOWN;
    cmd.turnSig         = DW_VEHICLEIO_TURNSIGNAL_UNKNOWN;
    cmd.throttleValid   = false;
    cmd.brakeValid      = false;
    cmd.steeringValid   = false;
    cmd.gearValid       = false;
    cmd.turnSigValid    = false;
}

static void vioKeyPressCallback(int key)
{
    static float32_t steeringStep = DEG2RAD(15);

    switch (key)
    {
    case GLFW_KEY_ESCAPE:
        gRun = false;
        cmd.enable = false;
        break;

    case GLFW_KEY_E:
        resetCmd();
        cmd.enable = true;
        cmd.clearFaults = true;
        break;

    case GLFW_KEY_UP:
        cmd.gearValid = 0;
        if (cmd.brakeValid) {
            cmd.brakeValue = 0;
            cmd.brakeValid = 0;
            break;
        }
        cmd.throttleValue += 0.07;
        cmd.throttleValid = 1;
        break;

    case GLFW_KEY_DOWN:
        if (cmd.throttleValid) {
            cmd.throttleValue = 0;
            cmd.throttleValid = 0;
            break;
        }
        cmd.brakeValue += 0.04;
        cmd.brakeValid = 1;
        break;

    case GLFW_KEY_LEFT:
        cmd.steering += steeringStep;
        cmd.steeringValid = 1;
        break;

    case GLFW_KEY_RIGHT:
        cmd.steering -= steeringStep;
        cmd.steeringValid = 1;
        break;

    case GLFW_KEY_F1:
        cmd.turnSig = cmd.turnSig != DW_VEHICLEIO_TURNSIGNAL_LEFT ?
            DW_VEHICLEIO_TURNSIGNAL_LEFT :
            DW_VEHICLEIO_TURNSIGNAL_OFF;
        cmd.turnSigValid = 1;
        break;

    case GLFW_KEY_F2:
        cmd.turnSig = cmd.turnSig != DW_VEHICLEIO_TURNSIGNAL_RIGHT ?
            DW_VEHICLEIO_TURNSIGNAL_RIGHT :
            DW_VEHICLEIO_TURNSIGNAL_OFF;
        cmd.turnSigValid = 1;
        break;

    case GLFW_KEY_D:
        resetCmd();
        break;

    case GLFW_KEY_P:
        cmd.gearValid = 1;
        cmd.gear = DW_VEHICLEIO_GEAR_PARK;
        break;

    default: std::cout<<"No command for this key "<<std::endl;
    }

}

static void initSdk(dwContextHandle_t *context, WindowBase *window)
{
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = window->getEGLDisplay();
#else
    (void)window;
#endif

    dwInitialize(context, DW_VERSION, &sdkParams);
}

static void initializeRendererState()
{
    dwRect rect{};
    rect.width   = gFrameWidth;
    rect.height  = gFrameHeight;
    rect.x = 0;
    rect.y = 0;

    dwRenderer_setRect(rect, gRenderer);

    float32_t rasterTransform[9];
    rasterTransform[0] = 1.0f; rasterTransform[3] = 0.0f; rasterTransform[6] = 0.0f;
    rasterTransform[1] = 0.0f; rasterTransform[4] = 1.0f; rasterTransform[7] = 0.0f;
    rasterTransform[2] = 0.0f; rasterTransform[5] = 0.0f; rasterTransform[8] = 1.0f;

    dwRenderer_set2DTransform(rasterTransform, gRenderer);
    dwRenderer_setPointSize(10.0f, gRenderer);
    dwRenderer_setColor(DW_RENDERER_COLOR_RED, gRenderer);
}

static void renderFrame(dwVehicleIOState *status)
{
    glDepthFunc(GL_LESS);

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, gRenderer);
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_12, gRenderer);
    dwRenderer_renderText(10, gFrameHeight - 20, "(ESC to quit)", gRenderer);
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_20, gRenderer);

    std::stringstream logBrakeThrottle;
    logBrakeThrottle << "Brake/Throttle Data"<< "\n";
    logBrakeThrottle << "\n";
    logBrakeThrottle << " brake: " << status->brakeValue << "\n";
    logBrakeThrottle << " brakeTorqueRequested: " << status->brakeTorqueRequested << "\n";
    logBrakeThrottle << " brakeTorqueActual: " << status->brakeTorqueActual << "\n";
    logBrakeThrottle << " throttleValue: " << status->throttleValue << "\n";

    dwRenderer_setColor(DW_RENDERER_COLOR_DARKRED, gRenderer);
    dwRenderer_renderText(10, gFrameHeight - 180, logBrakeThrottle.str().c_str(), gRenderer);

    std::stringstream logSteering;
    logSteering << "Steering Data"<< "\n";
    logSteering << "\n";
    logSteering << " steeringWheelAngle: " << status->steeringWheelAngle << "\n";
    logSteering << " steeringWheelTorque: " << status->steeringWheelTorque << "\n";

    dwRenderer_setColor(DW_RENDERER_COLOR_LIGHTGREEN, gRenderer);
    dwRenderer_renderText(10, gFrameHeight - 520, logSteering.str().c_str(), gRenderer);

    std::stringstream logMisc;
    logMisc << "Miscellaneous"<< "\n";
    logMisc << "\n";
    logMisc << " fuelLevel:  " << status->fuelLevel << "\n";
    logMisc << " gear:  " << status->gear << "\n";
    logMisc << " turnSignal:  " << status->turnSignal << "\n";

    dwRenderer_setColor(DW_RENDERER_COLOR_ORANGE, gRenderer);
    dwRenderer_renderText(500, gFrameHeight - 180, logMisc.str().c_str(), gRenderer);

    std::stringstream logSpeed;
    logSpeed << " Speed Data" << "\n";
    logSpeed << "\n";
    logSpeed << " VehicleSpeed:  " << status->speed << "\n";
    logSpeed << " wheelSpeedFrontLeft:  " << status->wheelSpeed[0]<< "\n";
    logSpeed << " wheelSpeedFrontRight:  " << status->wheelSpeed[3] << "\n";
    logSpeed << " wheelSpeedRearLeft:  " << status->wheelSpeed[1] << "\n";
    logSpeed << " wheelSpeedRearRight:  " << status->wheelSpeed[2] << "\n";

    dwRenderer_setColor(DW_RENDERER_COLOR_LIGHTBLUE, gRenderer);
    dwRenderer_renderText(500, gFrameHeight - 520, logSpeed.str().c_str(), gRenderer);

    gWindow->swapBuffers();
}

typedef enum {
    JS_A_DRIVE = 0,
    JS_B_REVERSE =1,
    JS_X_NEUTRAL = 2,
    JS_Y_PARK = 3,
    JS_Disable = 4,
    JS_Enable = 5,
    JS_Steering_multiplier_left = 6,
    JS_Steering_multiplier_right = 7,
    JS_Max_Button = 7,
} JS_buttons;

typedef enum {
    JS_brakeValue = 2,
    JS_steeringValue = 3,
    JS_throttleValue = 5,
    JS_Max_Axis = 5,
} JS_axes;

static void handleJoystick()
{
    int bcount;
    int acount;
    const unsigned char *buttons;
    const float *axes;

    buttons = glfwGetJoystickButtons(0, &bcount);
    axes = glfwGetJoystickAxes(0, &acount);

    if (bcount < JS_Max_Button || acount < JS_Max_Axis)
        return;

    if (buttons[JS_Enable]) {
        cmd.enable = true;
        cmd.clearFaults = true;
        cmd.brakeValid = true;
        cmd.throttleValid = true;
        cmd.steeringValid = true;
        cmd.gearValid = true;
        cmd.turnSigValid = true;
    }

    if (buttons[JS_Disable])
        resetCmd();

    if (axes[JS_brakeValue] + 1 >= 0.01)
        cmd.brakeValue = 0.16 + 0.17 * (axes[JS_brakeValue] + 1) / 2;
    else
        cmd.brakeValue = 0;

    cmd.throttleValue = 0.7 * (axes[JS_throttleValue] + 1) / 2;
    cmd.steering = -DEG2RAD(axes[JS_steeringValue] * 470);
}

int main(int argc, const char **argv)
{
    ProgramArguments arguments({
            ProgramArguments::Option_t("driver", "can.virtual"),
            ProgramArguments::Option_t("params", ("file=" + DataPath::get() + "/samples/vehicleio/can.bin").c_str()),
            ProgramArguments::Option_t("rig",  (DataPath::get() + "/samples/vehicleio/rig.xml").c_str()),
            ProgramArguments::Option_t("type", "dataspeed"),
        });
    initSampleApp(argc, argv, &arguments, vioKeyPressCallback, gFrameWidth, gFrameHeight);

    initSdk(&gSdk, gWindow);

    // init rig
    if (dwRigConfiguration_initializeFromFile(&gRig, gSdk, gArguments.get("rig").c_str()) != DW_SUCCESS)
    {
        std::cout << "Cannot load vehicle information from given rig file: " << gArguments.get("rig") << std::endl;
        dwRelease(&gSdk);
        dwLogger_release();

        return -1;
    }

    dwSAL_initialize(&gHal, gSdk);

    CHECK_DW_ERROR( dwRenderer_initialize(&gRenderer, gSdk) );

    initializeRendererState();

    dwSensorHandle_t canSensor = DW_NULL_HANDLE;
    {
        dwSensorParams params;

        params.parameters = gArguments.get("params").c_str();
        params.protocol   = gArguments.get("driver").c_str();

        if (dwSAL_createSensor(&canSensor, params, gHal) != DW_SUCCESS) {
            std::cout << "Cannot create sensor "
                      << params.protocol << " with " << params.parameters << std::endl;
            dwSAL_release(&gHal);
            dwRelease(&gSdk);
            dwRenderer_release(&gRenderer);
            dwLogger_release();
            return -1;
        }
    }

    dwVehicleIOType type;
    if (gArguments.get("type") == "dataspeed")
        type = DW_VEHICLEIO_DATASPEED;
    else {
        std::cout << "unknown VehicleIO type" << std::endl;
        dwSAL_releaseSensor(&canSensor);
        dwSAL_release(&gHal);
        dwRelease(&gSdk);
        dwRenderer_release(&gRenderer);
        dwLogger_release();
        return -1;
    }

    const dwVehicle *vehicle = nullptr;
    dwRigConfiguration_getVehicle(&vehicle, gRig);

    dwVehicleIOHandle_t vehicleIO = DW_NULL_HANDLE;
    if (dwVehicleIO_initialize(&vehicleIO, type, vehicle, gSdk) != DW_SUCCESS)
    {
        std::cout << "Cannot create VehicleIO controller" << std::endl;
        dwSAL_releaseSensor(&canSensor);
        dwSAL_release(&gHal);
        dwRelease(&gSdk);
        dwLogger_release();

        return -1;
    }

    if (dwVehicleIO_setDrivingMode(DW_VEHICLEIO_DRIVING_LIMITED_ND, vehicleIO) != DW_SUCCESS) {
        std::cout << "Cannot change driving mode" << std::endl;
        dwVehicleIO_release(&vehicleIO);
        dwSAL_releaseSensor(&canSensor);
        dwSAL_release(&gHal);
        dwRelease(&gSdk);
        dwLogger_release();
        return -1;
    }

    dwTime_t tn;
    dwContext_getCurrentTime(&tn, gSdk);

    resetCmd();

    dwTime_t t0serv = tn + 1000000;
    dwTime_t t0send = t0serv + 10000 + 20000;

    gRun = dwSensor_start(canSensor) == DW_SUCCESS;

    while(gRun && !gWindow->shouldClose())
    {
        dwContext_getCurrentTime(&tn, gSdk);

        handleJoystick();

        dwCANMessage msg{};
        dwStatus status;

        status = dwSensorCAN_readMessage(&msg, 100000, canSensor);
        if (status == DW_END_OF_STREAM) {
            std::cerr << "VehicleIO sample: reached CAN EOF" << std::endl;
            break;
        } else if (status != DW_SUCCESS) {
            std::cerr << "VehicleIO sample: CAN error "
                      << dwGetStatusName(status) << std::endl;
            continue;
        }

        status = dwVehicleIO_consume(&msg, vehicleIO);
        if (status != DW_SUCCESS) {
            std::cerr << "VehicleIO sample: can't consume CAN" << std::endl;
            break;
        }

        dwVehicleIO_getVehicleState(&vehicleState, vehicleIO);

        renderFrame(&vehicleState);

        if (tn >= t0send) {
            dwVehicleIO_sendVehicleCommand(&cmd, canSensor, vehicleIO);
            t0send += 20000;

            if (cmd.clearFaults)
                cmd.clearFaults = false;

            std::cout << "VehicleIO sample: Status" << std::endl
                      << "steeringWheelAngle: " << vehicleState.steeringWheelAngle << std::endl
                      << "throttleValue: " << vehicleState.throttleValue << std::endl
                      << "brake value: " << vehicleState.brakeValue << std::endl
                      << "faults: " << static_cast<uint32_t>(vehicleState.faults) << std::endl
                      << "overrides: " << static_cast<uint32_t>(vehicleState.overrides) << std::endl
                      << std::endl;
        }
    }

    dwRigConfiguration_release(&gRig);
    dwRenderer_release(&gRenderer);
    dwVehicleIO_release(&vehicleIO);
    dwSAL_releaseSensor(&canSensor);
    dwSAL_release(&gHal);
    dwRelease(&gSdk);
    dwLogger_release();

    return 0;
}
