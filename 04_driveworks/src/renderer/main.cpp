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

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <memory>
#include <math.h>

#include <framework/WindowGLFW.hpp>
#ifdef VIBRANTE
#include <framework/WindowEGL.hpp>
#endif
#include <framework/ProgramArguments.hpp>
#include <framework/Log.hpp>
#include <framework/Checks.hpp>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/Types.h>

// REndering
#include <dw/renderer/Renderer.h>

//#######################################################################################
// Globals
dwContextHandle_t  gSdk      = DW_NULL_HANDLE;

dwRendererHandle_t gRenderer = DW_NULL_HANDLE;
dwRenderBufferHandle_t gData = DW_NULL_HANDLE;

// Windows and general GL variables.
WindowBase *gWindow = nullptr;

int  gFrameWidth    = 1280;
int  gFrameHeight   = 800;
bool gRun           = false;

//#######################################################################################
// Function declarations
void initializeRendererState();
void render2Dtests();
void renderFrame();
void idle();


//#######################################################################################
void keyPressCallback(int key)
{
    // stop application
    if (key == GLFW_KEY_ESCAPE)
        gRun = false;
}

//#######################################################################################
void frameBufferResizeCallback(int width, int height)
{
    gFrameWidth  = width;
    gFrameHeight = height;

    dwRect rect;
    rect.width   = gFrameWidth;
    rect.height  = gFrameHeight;
    rect.x = 0;
    rect.y = 0;

    dwRenderer_setRect(rect, gRenderer);

    glViewport(0, 0, gFrameWidth, gFrameHeight);
}

//#######################################################################################
void initGL(WindowBase **window)
{
    if(!*window)
        *window = new WindowGLFW(gFrameWidth, gFrameHeight);

    (*window)->makeCurrent();
    (*window)->setOnKeypressCallback(keyPressCallback);
    (*window)->setOnResizeWindowCallback(frameBufferResizeCallback);
}

//#######################################################################################
void initSdk(dwContextHandle_t *context, WindowBase *window)
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = window->getEGLDisplay();
#else
    (void)window;
#endif

    dwInitialize(context, DW_VERSION, &sdkParams);
}

//#######################################################################################
void initializeRendererState()
{
    // Prepare some data for rendering
    dwRenderBufferVertexLayout layout;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;

    dwRenderBuffer_initialize(&gData, layout, DW_RENDER_PRIM_POINTLIST, 4, gSdk);

    // update the data
    float32_t* map;
    uint32_t   maxVerts, stride;

    if(dwRenderBuffer_map(&map, &maxVerts, &stride, gData) == DW_SUCCESS)
    {
        map[0] = 0.60f; map[1] = 0.25f;
        map[2] = 0.60f; map[3] = 0.75f;
        map[4] = 0.90f; map[5] = 0.25f;
        map[6] = 0.90f; map[7] = 0.75f;

        dwRenderBuffer_unmap(maxVerts, gData);
    }

    // Set some renderer defaults
    dwRect rect;
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

//#######################################################################################
void renderFrame()
{
    glDepthFunc(GL_LESS);

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    render2Dtests();

    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, gRenderer);
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, gRenderer);
    dwRenderer_renderText(10, gFrameHeight - 20, "(ESC to quit)", gRenderer);

    // Swap front and back buffers
    gWindow->swapBuffers();
}

//#######################################################################################
void idle()
{
    static float xOffset;
    static float yOffset;
    static float angle = 0.0;

    xOffset = 0.01f * cos(angle);
    yOffset = 0.01f * sin(angle);

    // update the data
    float32_t* map;
    uint32_t   maxVerts, stride;

    if(dwRenderBuffer_map(&map, &maxVerts, &stride, gData) == DW_SUCCESS)
    {
        map[0] = 0.60f + xOffset; map[1] = 0.25f + yOffset;
        map[2] = 0.60f + xOffset; map[3] = 0.75f + yOffset;
        map[4] = 0.90f + xOffset; map[5] = 0.25f + yOffset;
        map[6] = 0.90f + xOffset; map[7] = 0.75f + yOffset;

        dwRenderBuffer_unmap(maxVerts, gData);

        angle += 0.001f;
    }
}

//#######################################################################################
void render2Dtests()
{
    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, gRenderer);
    dwRenderer_renderBuffer(gData, gRenderer);

    dwRenderer_setColor(DW_RENDERER_COLOR_RED, gRenderer);

    int yCoord = 20;
    int offset = 90;

    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_8, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 8", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_12, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 12", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 16", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_20, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 20", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_24, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 24", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_32, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 32", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_48, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 48", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_64, gRenderer);
    dwRenderer_renderText(20, yCoord, "Verdana 64", gRenderer);
    yCoord +=offset;
    dwRenderer_setFont(DW_RENDER_FONT_VERDANA_20, gRenderer);
    dwRenderer_renderText(20, yCoord, "Test line break\n\tand tabs", gRenderer);
}

//#######################################################################################
//
// MAIN
//
//#######################################################################################
int main(int argc, const char **argv)
{
    (void)argc;
    (void)argv;
#ifndef WINDOWS
    struct sigaction action = {};
    action.sa_handler = [](int) { gRun = false; };

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
#endif

    gRun = true;

    initGL(&gWindow);
    initSdk(&gSdk, gWindow);


    CHECK_DW_ERROR( dwRenderer_initialize(&gRenderer, gSdk) );

    initializeRendererState();

    // MAIN LOOP
    while (gRun && !gWindow->shouldClose())
    {
        // Render here
        renderFrame();

        // Process stuff
        idle();
    }

    dwRenderer_release(&gRenderer);
    dwRenderBuffer_release(&gData);
    dwRelease(&gSdk);
    dwLogger_release();

    delete gWindow;

    return 0;
}
