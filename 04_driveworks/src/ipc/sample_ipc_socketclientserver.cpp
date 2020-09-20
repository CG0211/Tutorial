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

#include <dw/core/Context.h>
#include <dw/ipc/SocketClientServer.h>

#include <framework/Log.hpp>
#include <framework/Checks.hpp>
#include <framework/ProgramArguments.hpp>

#include <signal.h>

#include <iostream>
#include <thread>

//------------------------------------------------------------------------------
// Variables of working/debugging status of the program.
//------------------------------------------------------------------------------
static volatile bool g_run = true;

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
void sig_int_handler(int)
{
    g_run = false;
}

dwStatus runServer(ProgramArguments const &arguments, dwContextHandle_t ctx)
{
    auto status = DW_FAILURE;

    auto port = static_cast<uint16_t>(std::stoul(arguments.get("port")));

    auto socketServer = dwSocketServerHandle_t{DW_NULL_HANDLE};
    CHECK_DW_ERROR(dwSocketServer_initialize(&socketServer, port, 2, ctx));

    // accept two connections (use two connections for illustration,
    // a single connection can also be used bi-directionally)
    auto socketConnectionRead  = dwSocketConnectionHandle_t{DW_NULL_HANDLE},
         socketConnectionWrite = dwSocketConnectionHandle_t{DW_NULL_HANDLE};
    do {
        status = dwSocketServer_accept(&socketConnectionRead, 10000, socketServer);
    } while (status == DW_TIME_OUT);
    do {
        status = dwSocketServer_accept(&socketConnectionWrite, 10000, socketServer);
    } while (status == DW_TIME_OUT);

    if (status != DW_FAILURE) {
        while (g_run) {
            size_t data;
            auto size = sizeof(decltype(data));

            // recieve data
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_recv(reinterpret_cast<uint8_t *>(&data), &size,
                                                  socketConnectionRead)) == DW_END_OF_STREAM)
                break;
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
                break;

            std::cout << "Socket Server received " << data << std::endl;

            // send data back
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_send(reinterpret_cast<uint8_t *>(&data), &size,
                                                  socketConnectionWrite)) == DW_END_OF_STREAM)
                break;
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
                break;

            std::cout << "Socket Server send " << data << std::endl;
        }
    }

    CHECK_DW_ERROR(dwSocketConnection_release(&socketConnectionWrite));
    CHECK_DW_ERROR(dwSocketConnection_release(&socketConnectionRead));
    CHECK_DW_ERROR(dwSocketServer_release(&socketServer));

    return DW_SUCCESS;
}

dwStatus runClient(ProgramArguments const &arguments, dwContextHandle_t ctx)
{
    auto status = DW_FAILURE;

    auto ip   = arguments.get("ip");
    auto port = static_cast<uint16_t>(std::stoul(arguments.get("port")));

    auto socketClient = dwSocketClientHandle_t{DW_NULL_HANDLE};
    CHECK_DW_ERROR(dwSocketClient_initialize(&socketClient, 2, ctx));

    // connect two connections (use two connections for illustration,
    // a single connection can also be used bi-directionally)
    auto socketConnectionWrite = dwSocketConnectionHandle_t{DW_NULL_HANDLE},
         socketConnectionRead  = dwSocketConnectionHandle_t{DW_NULL_HANDLE};
    do {
        status = dwSocketClient_connect(&socketConnectionWrite, ip.c_str(), port, 10000, socketClient);
    } while (status == DW_TIME_OUT);
    do {
        status = dwSocketClient_connect(&socketConnectionRead, ip.c_str(), port, 10000, socketClient);
    } while (status == DW_TIME_OUT);

    if (status != DW_FAILURE) {
        while (g_run) {
            // send some data
            static size_t dataRef = 0;
            ++dataRef;
            decltype(dataRef) data;
            auto size = sizeof(decltype(data));

            // send data
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_send(reinterpret_cast<uint8_t *>(&dataRef), &size,
                                                  socketConnectionWrite)) == DW_END_OF_STREAM)
                break;
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
                break;

            std::cout << "Socket Client send " << dataRef << std::endl;

            // recieve data
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 500));
            if ((status = dwSocketConnection_recv(reinterpret_cast<uint8_t *>(&data), &size,
                                                  socketConnectionRead)) == DW_END_OF_STREAM)
                break;
            CHECK_DW_ERROR(status);

            if (size != sizeof(decltype(data)))
                break;

            std::cout << "Socket Client received " << data << std::endl;
        }
    }

    CHECK_DW_ERROR(dwSocketConnection_release(&socketConnectionWrite));
    CHECK_DW_ERROR(dwSocketConnection_release(&socketConnectionRead));
    CHECK_DW_ERROR(dwSocketClient_release(&socketClient));

    return DW_SUCCESS;
}

int main(int argc, const char **argv)
{
    ProgramArguments arguments({ProgramArguments::Option_t("role", "client or server"),
                                ProgramArguments::Option_t("ip", "127.0.0.1"),
                                ProgramArguments::Option_t("port", "49252")});

    if (!arguments.parse(argc, argv))
        exit(-1); // Exit if not all require arguments are provided
    else
        std::cout << "Program Arguments:\n" << arguments.printList() << std::endl;

#if (!WINDOWS)
    struct sigaction action = {};
    action.sa_handler       = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
    sigaction(SIGSTOP, &action, NULL); // kill command
#endif
    g_run = true;

    // Initialize context
    auto ctx = dwContextHandle_t{DW_NULL_HANDLE};
    {
        CHECK_DW_ERROR(dwLogger_initialize(getConsoleLoggerCallback(true, true)));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_VERBOSE));
        CHECK_DW_ERROR(dwInitialize(&ctx, DW_VERSION, nullptr));
    }

    // Run client / server
    auto const role = arguments.get("role");

    auto status = DW_FAILURE;
    if (role == "server")
        status = runServer(arguments, ctx);
    else if (role == "client")
        status = runClient(arguments, ctx);
    else {
        std::cerr << "Invalid role parameter '" << role << "' provided (use either 'client' or 'server')"
                  << std::endl;
    }

    CHECK_DW_ERROR(dwRelease(&ctx));

    return status == DW_SUCCESS;
}
