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
///////////////////////////////////////////////////////////////////////////////////////


#include <framework/DriveWorksSample.hpp>

// Include all relevant DriveWorks modules
#include <dw/Driveworks.h>


using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Template of a sample. Put some description what the sample does here
//------------------------------------------------------------------------------
class MySample : public DriveWorksSample
{
private:

    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t               context             = DW_NULL_HANDLE;
    dwSALHandle_t                   sal                 = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwRendererHandle_t              renderer            = DW_NULL_HANDLE;

public:

    MySample(const ProgramArguments& args) : DriveWorksSample(args) {}

    /// -----------------------------
    /// Initialize everything of a sample here incl. SDK components
    /// -----------------------------
    bool onInitialize() override
    {
        log("Starting my sample application...\n");

        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            dwLogger_initialize(getConsoleLoggerCallback(true));
            dwLogger_setLogLevel(DW_LOG_VERBOSE);

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};
            sdkParams.dataPath = DataPath::get_cstr();

            #ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
            #endif

            dwInitialize(&context, DW_VERSION, &sdkParams);
            dwSAL_initialize(&sal, context);
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            CHECK_DW_ERROR_MSG(dwRenderer_initialize(&renderer, context),
                               "Cannot initialize Renderer, maybe no GL context available?");
            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, renderer);
        }

        return true;
    }


    ///------------------------------------------------------------------------------
    /// This method is executed when user presses `R`, it indicates that sample has to reset
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        logWarn("My sample has been reset...\n");
    }

    ///------------------------------------------------------------------------------
    /// This method is executed on release, free up used memory here
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        dwRenderer_release(&renderer);

        // -----------------------------------------
        // Release DriveWorks context and SAL
        // -----------------------------------------
        dwSAL_release(&sal);
        dwRelease(&context);
    }


    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, renderer);

        log("window resized to %dx%d\n", width, height);
    }

    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - this method is executed for window and console based applications
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        // this is called from mainloop
        // do some stuff here
    }

    ///------------------------------------------------------------------------------
    /// Render call of the sample, executed for window based applications only
    ///     - render text on screen
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        glClearColor(0.0, 0.0, 1.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT);

        // render text in the middle of the window
        dwRect window;
        dwRenderer_getRect(&window, renderer);
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_32, renderer);
        dwRenderer_renderText(window.width/2 - 100, window.height/2, "Hello World", renderer);
    }


    ///------------------------------------------------------------------------------
    /// React to user inputs
    ///------------------------------------------------------------------------------
    void onProcessKey      (int key) override { log("key: %d\n", key); }
    void onMouseDown       (int button, float x, float y) override { log("mouse down %d at %fx%f\n", button, x,y); }
    void onMouseUp         (int button, float x, float y) override { log("mouse up %d at %fx%f\n", button, x,y);  }
    void onMouseMove       (float x, float y) override { log("mouse at %fx%f\n", x,y);  }
    void onMouseWheel      (float x, float y) override { log("mouse wheel press=%f, scroll=%f\n", x,y);  }
};



//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    // -------------------
    // define all arguments used by the application
    // parse user given arguments and bail out if there is --help request or proceed
    ProgramArguments args(argc, argv,
    {
       ProgramArguments::Option_t("optionA", (DataPath::get() + "/samples/path/to/data").c_str()),
       ProgramArguments::Option_t("optionB", "default")
    }, "This is a message shown on console when sample prints help.");

    // -------------------
    // initialize and start a window application (with offscreen support if required)
    MySample app(args);

    app.initializeWindow("My best sample", 1280, 800 /*, args.enabled("offscreen")*/);

    return app.run();
}
