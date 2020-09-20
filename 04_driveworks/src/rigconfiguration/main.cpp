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

#define _CRT_SECURE_NO_WARNINGS

#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/rigconfiguration/RigConfiguration.h>
#include <framework/ProgramArguments.hpp>
#include <framework/DataPath.hpp>
#include <framework/Log.hpp>

#include <vector>
#include <iostream>
#include <string>

#include <stdio.h>
#include <string.h>

#define PRINT_MEMBER(pData, member) printf(" " #member ": \t%f\n", pData->member)

dwContextParameters m_contextParams;
dwContextHandle_t m_context      = DW_NULL_HANDLE;
dwStatus m_status                = DW_SUCCESS;
dwRigConfigurationHandle_t m_rig = DW_NULL_HANDLE;

static void initialize(const char *rigconfigFilename)
{

    m_status = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (m_status != DW_SUCCESS) {
        printf("Error dwLogger_initialize: %s\n", dwGetStatusName(m_status));
        exit(-1);
    }

    // Initialize driveworks sdk
    m_status = dwInitialize(&m_context, DW_VERSION, &m_contextParams);
    if (m_status != DW_SUCCESS) {
        printf("Error dwInitialize: %s\n", dwGetStatusName(m_status));
        exit(-1);
    }

    m_status = dwRigConfiguration_initializeFromFile(&m_rig, m_context, rigconfigFilename);
    if (m_status != DW_SUCCESS) {
        printf("Error dwEgomotion_initialize: %s\n", dwGetStatusName(m_status));
        exit(-1);
    }
}

static void printVehicle()
{
    const dwVehicle *vehicle;
    dwStatus status;

    status = dwRigConfiguration_getVehicle(&vehicle, m_rig);
    if (status != DW_SUCCESS) {
        printf("No vehicle information available.\n");
        return;
    }

    printf("Vehicle Information:\n");
    PRINT_MEMBER(vehicle, width);
    PRINT_MEMBER(vehicle, height);
    PRINT_MEMBER(vehicle, length);
    PRINT_MEMBER(vehicle, wheelbase);
}

static void printSensors()
{
    uint32_t sensorCount;
    dwStatus status;

    dwRigConfiguration_getSensorCount(&sensorCount, m_rig);
    printf("Sensor Count: %d\n", sensorCount);

    for (uint32_t i = 0; i < sensorCount; ++i) {
        printf(" Sensor %d:\n", i);

        const char *name;
        status = dwRigConfiguration_getSensorName(&name, i, m_rig);
        if (status == DW_SUCCESS)
            printf("  Name:\t\t%s\n", name);

        const char *protocol;
        status = dwRigConfiguration_getSensorProtocol(&protocol, i, m_rig);
        if (status == DW_SUCCESS)
            printf("  Protocol:\t%s\n", protocol);
    }
}

static void release()
{

    if (m_rig) {
        dwRigConfiguration_release(&m_rig);
    }
    if (m_context) {
        dwRelease(&m_context);
    }
    dwLogger_release();
}

int main(int argc, const char *argv[])
{
    ProgramArguments arguments(
        {
            ProgramArguments::Option_t("rigconfig",
                                       (DataPath::get() +
                                        std::string{"/samples/sfm/triangulation/"
                                                    "rig.xml"})
                                           .c_str()),
        });

    if (!arguments.parse(argc, argv))
        return -1; // Exit if not all require arguments are provided

    std::cout << "Program Arguments:\n" << arguments.printList() << std::endl;

    initialize(arguments.get("rigconfig").c_str());
    printVehicle();
    printSensors();
    release();

    return 0;
}
