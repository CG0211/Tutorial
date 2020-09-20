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

#include <memory>

#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif
#include <framework/ProgramArguments.hpp>
#include <framework/MathUtils.hpp>

extern bool gRun;
extern bool gPause;
extern std::string gLidarImagetypeString;
extern WindowBase *gWindow;

// For 3D Display
float eye[3];
float center[3];
float up[3];

float fovRads = (float)DEG2RAD(60.0f);
float deltaSpinAngle = 0;

// MOUSE NAVIGATION VARIABLES
float deltaVerAngle;
float deltaHorAngle;
float deltaMove;

float radius;
float vertAngle;
float horAngle;

bool mouseLeft;
bool mouseRight;
int currentX;
int currentY;

//#######################################################################################
void initializeInputDefaults()
{
    // Initialize 3D view related variables.
    deltaVerAngle = 0.0f;
    deltaHorAngle = 0.0f;
    deltaMove     = 0.0f;

    radius    = 28;
    vertAngle = DEG2RAD(30.0f);
    horAngle  = DEG2RAD(180.0f);

    mouseLeft  = false;
    mouseRight = false;
    currentX   = -1;
    currentY   = -1;

    // Initialize eye, up for bowl view
    eye[0] = radius * cos(vertAngle) * cos(horAngle);
    eye[1] = radius * cos(vertAngle) * sin(horAngle);
    eye[2] = radius * sin(vertAngle);

    up[0] = 0;
    up[1] = 0;
    up[2] = 1;

    center[0] = 0;
    center[1] = 0;
    center[2] = 0;
}

//#######################################################################################
// USER INPUT
//#######################################################################################

//#######################################################################################
void keyPressCallback(int key)
{
    // stop application
    if (key == GLFW_KEY_ESCAPE)
        gRun = false;
    // pause application
    if (key == GLFW_KEY_SPACE)
        gPause = !gPause;
    // intensity Lidar Image
    if (key == GLFW_KEY_0)
        gLidarImagetypeString = std::string{"intensity"};
    // 3D depth Lidar Image
    if (key == GLFW_KEY_1)
        gLidarImagetypeString = std::string{"depth-xyz"};
    // roll 10 degree counter-clock wise
    if (key == GLFW_KEY_LEFT)
        deltaSpinAngle += 10;
    // roll 10 degree clock wise
    if (key == GLFW_KEY_RIGHT)
        deltaSpinAngle -= 10;
}

//#######################################################################################
void mouseUpCallback(int button, float x, float y)
{
    (void)x;
    (void)y;

    // only start motion if the left button is pressed
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        mouseLeft = false;
        vertAngle += deltaVerAngle;
        horAngle += deltaHorAngle;
        deltaHorAngle = deltaVerAngle = 0;
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        mouseRight = false;
        radius += deltaMove;
        deltaMove = 0;
    }

}

//#######################################################################################
void mouseDownCallback(int button, float x, float y)
{
    (void)x;
    (void)y;

    // only start motion if the left button is pressed
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        mouseLeft = true;
        currentX  = (int)floor(x);
        currentY  = (int)floor(y);

    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        mouseRight = true;
        currentX   = (int)floor(x);
        currentY   = (int)floor(y);
    }

}

//#######################################################################################
void mouseMoveCallback(float x, float y)
{
    // this will only be true when the left button is down
    if (mouseLeft) {

        // update deltaAngle
        deltaVerAngle = (y - currentY);
        deltaHorAngle = -(x - currentX);

        // scale deltaAngle
        deltaVerAngle *= 0.01f;
        deltaHorAngle *= 0.01f;

        // Limit the vertical angle (5 to 85 degrees)
        if ((vertAngle + deltaVerAngle) > DEG2RAD(85))
            deltaVerAngle = DEG2RAD(85) - vertAngle;

        if ((vertAngle + deltaVerAngle) < DEG2RAD(5))
            deltaVerAngle = DEG2RAD(5) - vertAngle;

        eye[0] = radius * cos(vertAngle + deltaVerAngle) * cos(horAngle + deltaHorAngle);
        eye[1] = radius * cos(vertAngle + deltaVerAngle) * sin(horAngle + deltaHorAngle);
        eye[2] = radius * sin(vertAngle + deltaVerAngle);
    }
}

//#######################################################################################
void mouseWheelCallback(float xOffset, float yOffset)
{
    (void)xOffset;

    float tmpRadius = radius + static_cast<float>(yOffset) * 0.5f;

    if (tmpRadius > 0.0f) {
        eye[0] = (tmpRadius)*eye[0] / radius;
        eye[1] = (tmpRadius)*eye[1] / radius;
        eye[2] = (tmpRadius)*eye[2] / radius;

        radius = tmpRadius;
    }
}
