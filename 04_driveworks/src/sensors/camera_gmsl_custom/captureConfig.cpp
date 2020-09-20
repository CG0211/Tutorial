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

#include "captureConfig.h"

#include <cassert>
#include <cstring>
#include <sstream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

namespace {
using namespace std;
static const char separator = '=';
static const char comment_marker = '#';

std::string trim(const std::string& str,
                 const std::string& whitespace = " \t")
{
    const auto begin = str.find_first_not_of(whitespace);
    if (begin == std::string::npos)
        return "";

    const auto end = str.find_last_not_of(whitespace);
    const auto range = end - begin + 1;

    return str.substr(begin, range);
}

vector<string> &split(const string& s, char delim, vector<string>& elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(trim(item));
    }
    return elems;
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

    void setDefaults(CaptureConfigParams& dataset) {
        dataset.cameraCount = 1;
        dataset.requiredFrameRate = 30;
        strcpy(dataset.cameraMask, "0000");
        strcpy(dataset.csiOut, "3210");
        strcpy(dataset.board, "P2382-a01");
        strcpy(dataset.moduleName, "ref_max9286_9271_ov10635");
        strcpy(dataset.resolution, "1280x800");
        strcpy(dataset.inputFormat, "422p");
        strcpy(dataset.interface, "csi-ab");
        dataset.slave = false;
        dataset.i2cDevice = 7;
        dataset.desAddr = 0x6a;
        dataset.brdcstSerAddr = 0x40;
        dataset.brdcstSensorAddr = 0x30;
        dataset.enableEmbLines = false;
        dataset.enableSimulator = false;
        dataset.initialized = false;
        dataset.crossCSISync = false;
    }

}

CaptureConfigParams parseConfigFile(std::string& configurationFile)
{
    CaptureConfigParams result{};

    // set some default values for the driveworks options;
    setDefaults(result);

    ifstream file(configurationFile);
    string line;

    while(getline(file, line)) {
        line = trim(line);

        if (line.size() > 0 && line.at(0) == comment_marker) {
            continue;
        }

        vector<string> halves;
        halves = split(line, separator);

        if (2 != halves.size()) {
            continue;
        }

        const std::string parameter = halves[0];
        const std::string value = halves[1];

        std::cout << "parameter " << parameter << "    value " << value << std::endl;
        if(strcmp("board", parameter.c_str()) == 0) {
            strcpy(result.board,value.c_str());
        } else if(strcmp("module-name", parameter.c_str()) == 0) {
            strcpy(result.moduleName,value.c_str());
        } else if(strcmp("resolution", parameter.c_str()) == 0) {
            strcpy(result.resolution,value.c_str());
        } else if(strcmp("input-format", parameter.c_str()) == 0) {
            strcpy(result.inputFormat,value.c_str());
        } else if(strcmp("camera-count", parameter.c_str()) == 0) {
            try {
                result.cameraCount = std::stoi(value);
            } catch (const std::exception& e) {
                std::cerr << "An invalid value was specified for camera-count" <<std::endl;
            }
        } else if(strcmp("interface", parameter.c_str()) == 0) {
            strcpy(result.interface,value.c_str());
        } else if(strcmp("i2cDevice", parameter.c_str()) == 0) {
            try {
                result.i2cDevice = std::stoi(value);
            } catch (const std::exception& e) {
                std::cerr << "An invalid value was specified for i2cDevice" <<std::endl;
            }
        } else if(strcmp("desAddr", parameter.c_str()) == 0) {
            try {
                result.desAddr = std::stoul(value, nullptr, 16);
            } catch (const std::exception& e) {
                std::cerr << "An invalid value was specified for desAddr" <<std::endl;
            }
        } else if(strcmp("brdcstSerAddr", parameter.c_str()) == 0) {
            try {
                result.brdcstSerAddr = std::stoul(value, nullptr, 16);
            } catch (const std::exception& e) {
                std::cerr << "An invalid value was specified for brdcstSerAddr" <<std::endl;
            }
        } else if(strcmp("brdcstSensorAddr", parameter.c_str()) == 0) {
            try {
                result.brdcstSensorAddr = std::stoul(value, nullptr, 16);
            } catch (const std::exception& e) {
                std::cerr << "An invalid value was specified for brdcstSensorAddr" <<std::endl;
            }
        } else if(strcmp("slave", parameter.c_str()) == 0) {
            result.slave = (strcmp("false", value.c_str()) == 0) ? false : true;
        } else if(strcmp("enableEmbLines", parameter.c_str()) == 0) {
            result.enableEmbLines = (strcmp("false", value.c_str()) == 0) ? false : true;
        } else if(strcmp("enableSimulator", parameter.c_str()) == 0) {
            result.enableSimulator = (strcmp("false", value.c_str()) == 0) ? false : true;
        } else if(strcmp("initialized", parameter.c_str()) == 0) {
            result.initialized = (strcmp("false", value.c_str()) == 0) ? false : true;
        } else if(strcmp("crossCSISync", parameter.c_str()) == 0) {
            result.crossCSISync = (strcmp("false", value.c_str()) == 0) ? false : true;
        } else if (strcmp("required-framerate", parameter.c_str()) == 0) {
            try{
                result.requiredFrameRate = std::stoi(value);
            } catch (const std::exception& e) {
                std::cerr << "An invalid value was specified for required-framerate" <<std::endl;
            }
        } else if (strcmp("camera-mask", parameter.c_str()) == 0) {
                strcpy(result.cameraMask, value.c_str());
        } else if (strcmp("csiOut",parameter.c_str()) == 0) {
                strcpy(result.csiOut, value.c_str());
        }
    }

    return result;
}


