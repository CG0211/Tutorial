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
#define _CRT_SECURE_NO_WARNINGS

#include "CameraCalibrationSample.hpp"

//#######################################################################################
int main(int argc, const char **argv)
{
    const std::string samplePath = DataPath::get() + "/samples/recordings/suburb0/";

    // -------------------
    // define all arguments used by the application
    ProgramArguments args(argc, argv,
    {
        ProgramArguments::Option_t("path", samplePath.c_str(), "Path to offline data"),
        ProgramArguments::Option_t("rig", "rig.json",
                                     "The name of the rig configuration file."),
        ProgramArguments::Option_t("sensorIndex", "0",
                                     "The index in the rig file of the sensor we are calibrating"),

        //arguments to use if you are running program on virtual data
        ProgramArguments::Option_t("video", "video_0_roof_front_120.h264","The name of video file"),
        ProgramArguments::Option_t("video-time", "video_time_0.txt","The name of video time file"),
        ProgramArguments::Option_t("can", "can_vehicle.bin","The name of CAN data file"),
        ProgramArguments::Option_t("imu", "imu_xsens.bin","The name of IMU data file"),
        ProgramArguments::Option_t("dbc", "can.dbc","The name of the CAN dbc file"),
        ProgramArguments::Option_t("dbc-speed", "Steering_Report.SPEED","The field name for speed in the CAN data"),
        ProgramArguments::Option_t("dbc-steering", "Steering_Report.ANGLE","The field name for angle in the CAN data"),
        ProgramArguments::Option_t("lanenet-threshold","0.3","The threshold for lanenet"),
    });

    dw_samples::calibration::CameraCalibrationSample app(args);

    app.initializeWindow("Camera Calibration Sample", 1280, 800, false);

    return app.run();
}

