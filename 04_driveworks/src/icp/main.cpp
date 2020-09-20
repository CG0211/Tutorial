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


// Driveworks sample includes
#include <framework/DriveWorksSample.hpp>
#include <framework/Mat4.hpp>

// Renderer
#include <dw/renderer/Renderer.h>

// ICP
#include <dw/icp/icp.h>

#include <algorithm>

#include "PlyWriters.hpp"

using namespace dw_samples::common;

using std::cout;
using std::endl;

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxB for easy usage.
inline dwTransformation& operator*=(dwTransformation& one, const dwTransformation& two)
{
    dwTransformation temp = one;
    Mat4_AxB(one.array, temp.array, two.array);
    return one;
}

//------------------------------------------------------------------------------
// Pretty printer for a dwTransformation object.
inline std::ostream& operator << (std::ostream& o, const dwTransformation& tx)
{
    o << '\n';
    for (int32_t r = 0; r < 4 ; ++r)
    {
        for (int32_t c = 0; c < 4 ; ++c)
        {
            o << '\t' << std::setw(8) << std::right
              << std::fixed << std::setprecision(6)<< tx.array[r + c*4];
        }
        o << '\n';
    }
    return o;
}



//------------------------------------------------------------------------------
// Lidar ICP samples
// The sample demonstrates how to use ICP module to align Lidar point clouds
//
//------------------------------------------------------------------------------
class LidarICP : public DriveWorksSample
{
private:

    // ------------------------------------------------
    // Types
    // ------------------------------------------------
    // A point used in DW Lidar module.
    typedef dwLidarPointXYZI dwPoint;
    // Point could data:
    typedef std::vector<dwPoint> dwPCD;

    // ------------------------------------------------
    // settings
    // ------------------------------------------------

    // Input file required for module to run
	std::string lidarFile;
    // Location of output PLY file
    std::string ICPPlyFile;
    // How many spins to skip at the beginning
    uint32_t initSpin;
    // How many spins to skip for each ICP step
    uint32_t numSkip;	
    // Maximum number of ICP iterations to do
    uint32_t maxIters;
    // Maximum number of Lidar spins to process
    uint32_t numFrames;
    // Counter for number of lidar spins to process
    uint32_t spinNum = 0;

    // ------------------------------------------------
    // Global Constants:
    // ------------------------------------------------
    // Number of points to be used for ICP, we random-subsample a spin from lidar if there are more. 32K is maximum this
    // can be under current implementation.
    const size_t ICPPtsSize = 20000;
    // Number of points to dump to PLY file, points are random-sampled.
    const size_t VizPtsSize = 8000;
    // The resolution of a depthmap arrangement in Pts/Degree, see sample function's comments.
    const size_t DepthMapHorzRes  = 2;
    // Number of Top and Bottom beams of lidar to ignore in sampling of points.
    const size_t DepthMapBeamStart = 10, DepthMapBeamEnd = 45;
    // Maximal number of points rendered
    const size_t MaxNumPointsToRender = 500000;

    // render grid size
    const float32_t WORLD_GRID_SIZE_IN_METERS = 200.0f;
    const float32_t WORLD_GRID_RES_IN_METERS = 5.0f;

    // Identity matrix for use in initialization
    static constexpr dwTransformation I = {{
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
    }};

    // ------------------------------------------------
    // Driveworks Context and SAL
    // ------------------------------------------------
    dwContextHandle_t               sdkContext          = DW_NULL_HANDLE;
    dwSALHandle_t                   sal                 = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific variables
    // ------------------------------------------------
    dwRendererHandle_t              renderer            = DW_NULL_HANDLE;
    dwICPHandle_t                   icpHandle           = DW_NULL_HANDLE;
    dwSensorHandle_t                lidarSensor         = DW_NULL_HANDLE;
    dwRenderBufferHandle_t          groundPlane         = DW_NULL_HANDLE;
    dwRenderBufferHandle_t          pointCloud          = DW_NULL_HANDLE;

    // This is the ICP Transform that starts at I, All transforms are accumulated to here.
    dwTransformation icpRigToWorld = I;

    // ICP prior pose, which is set to last optimized pose (assuming nearly constant velocity)
    dwTransformation icpPriorPose = I;

    dwVector3f centerViewDiff{0,0,5};

    // Velodyne lidar (for which this sample is written) has 64 beams that produce 64 depth measurements per vertical slice
    // We want to arrange them in the beam order to get a neighboring points in wht world be close together in the sampled
    // array. See sample() function's BeamOrder comment for more details.
    std::array<std::vector<dwPoint>, 64> depthMap;

    dwPCD curSourceFull;
    dwPCD curTargetSubsampled;
    dwPCD accumulatedPointCloud;

public:

    // initialize sample
    LidarICP(const ProgramArguments& args) : DriveWorksSample(args)
    {
        lidarFile = getArgument("lidarFile");

        initSpin = uint32_t(atoi(getArgument("init").c_str()));
        numSkip  = uint32_t(atoi(getArgument("skip").c_str()));
        numFrames = uint32_t(atoi(getArgument("numFrames").c_str()));
        maxIters = uint32_t(atoi(getArgument("maxIters").c_str()));

        // This is necessary because first frames do not contain enough points
        if(initSpin < 20)  std::cerr << "`--init` too small, set to " << (initSpin = 20) << endl;

        if(numSkip  > 5)    std::cerr << "`--skip` too large, set to " << (numSkip = 5)   << endl;

        if(maxIters > 50)   std::cerr << "`--maxIters` too large, set to " << (maxIters = 50) << endl;
        if(numFrames == 0)  numFrames = (uint32_t)-1;

        ICPPlyFile = getArgument("plyloc") +
                std::to_string(initSpin) +  "-" +
                std::to_string(numSkip + 1) +   "-" +
                std::to_string(maxIters) + "-icp.ply";

        spinNum = 0;
    }

    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers
    /// -----------------------------
    bool onInitialize() override
    {
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

            CHECK_DW_ERROR_MSG(dwInitialize(&sdkContext, DW_VERSION, &sdkParams),
                               "Cannot initialize Drive-Works SDK Context");

            CHECK_DW_ERROR_MSG(dwSAL_initialize(&sal, sdkContext),
                               "Cannot initialize SAL");
        }

        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            CHECK_DW_ERROR_MSG(dwRenderer_initialize(&renderer, sdkContext),
                               "Cannot initialize Renderer, maybe no GL context available?");
            dwRect rect;
            rect.width  = getWindowWidth();
            rect.height = getWindowHeight();
            rect.x      = 0;
            rect.y      = 0;
            dwRenderer_setRect(rect, renderer);

            constructGrid();
            preprarePointCloudRenderBuffer();
        }

        // Start the lidar sensor and check if its the right kind.
        {
            dwSensorParams sensor_params;
            std::string sensorParam = "file=" + lidarFile;
            sensor_params.parameters = sensorParam.c_str();
            sensor_params.protocol   = "lidar.virtual";

            CHECK_DW_ERROR_MSG(dwSAL_createSensor(&lidarSensor, sensor_params, sal), "Sensor create failed with ");

            CHECK_DW_ERROR_MSG(dwSensor_start(lidarSensor), "Lidar start failed");

            dwLidarProperties props;
            dwSensorLidar_getProperties(&props, lidarSensor);

            // Currently sample only works with Velodyne like Lidars
            if(std::string("VELO_HDL64E") != props.deviceString) {
                logError("Sample running with Lidar device named %s"
                          "\n This sample designed for velodyne 64 data only.\n", props.deviceString );
                return false;
            }
        }

        // -----------------------------
        // Initialize ICP module
        // -----------------------------
        {
            //We are using point cloud version forsmaplecurrently.
            dwICPParams params{};

            params.maxPoints = ICPPtsSize;
            params.icpType = dwICPType::DW_ICP_TYPE_LIDAR_POINT_CLOUD;

            CHECK_DW_ERROR_MSG(dwICP_initialize(&icpHandle, &params, sdkContext), "ICP Init failed");

            CHECK_DW_ERROR_MSG(dwICP_setMaxIterations(maxIters, icpHandle), "Failed to set ICP max iterations");
            CHECK_DW_ERROR_MSG(dwICP_setConvergenceTolerance(1e-3, 1e-3, icpHandle), "Failed to set ICP tolerances");
        }

        // Skip initial spins as per the command line args, and set the next immediate frame as target
        log("Skipping %d spins..\n", initSpin);
        {
            dwPCD tmp;
            for(uint i  = 0 ; i < initSpin; ++i) getSpin(tmp, false);
            log(" Done!\n");
        }

        // This is the first target. All points from next spins are put in the coordinates of this frame.
        {
            dwPCD targetFull;

            if (!getSpin(targetFull, true))
            {
                log("No more data available to work with. Stop\n");
                return false;
            }

            // subsample first
            sample(targetFull, curTargetSubsampled, ICPPtsSize);
        }

        accumulateCurrentPoints();

        return true;
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
        // -----------------------------------------
        // Write all currently accumulated points into a PLY file
        // -----------------------------------------
        if (getArgument("plyloc") != "")
        {
            PlyWriter ply(ICPPlyFile.c_str());
            for (const auto& t : accumulatedPointCloud)
            {
                ply.PushOnePoint(t, t.intensity);
            }
        }

        // -----------------------------------------
        // Stop sensor
        // -----------------------------------------
        dwSensor_stop(lidarSensor);
        dwSAL_releaseSensor(&lidarSensor);

        // -----------------------------------------
        // Release icp
        // -----------------------------------------
		dwICP_release(&icpHandle);

        // -----------------------------------------
        // Release renderer and streamer
        // -----------------------------------------
        {
            dwRenderer_release(&renderer);
            dwRenderBuffer_release(&pointCloud);
            dwRenderBuffer_release(&groundPlane);
        }

        // -----------------------------------------
        // Release DriveWorks handles, sdkContext and SAL
        // -----------------------------------------
        {
            dwSAL_release(&sal);
            dwRelease(&sdkContext);
        }
    }

    //------------------------------------------------------------------------------
    dwVector3f interpolateColor(float32_t value)
    {
        static std::vector<dwVector3f> HeatMap = {
                {0,0,1.0f},
                {0,1.0f,0},
                {1.0f,1.0f,0},
                {1.0f,0,0},
                {1.0f,1.0f,1.0f}
        };

        if( value <= 0 ) return HeatMap.front();
        if( value >= 1 ) return HeatMap.back();

        float32_t relative = value;
        int32_t numInts = static_cast<int32_t>(HeatMap.size() - 1);
        int32_t index = static_cast<int32_t>(relative * numInts); // multiply and round up
        relative -= static_cast<float32_t>(index) / numInts;
        relative *= numInts;
        return { (HeatMap[index].x * (1.f - relative) + HeatMap[index + 1].x * relative),
                 (HeatMap[index].y * (1.f - relative) + HeatMap[index + 1].y * relative),
                 (HeatMap[index].z * (1.f - relative) + HeatMap[index + 1].z * relative)    };
    }

    //------------------------------------------------------------------------------
    // This function dumps the points in depthMap to a PLY file via the PlyWriter object passed in.
    // Uses icpTx , the global 'current' transform or a spin. Writes ~VizPtsSize points to PLY.
    void accumulateCurrentPoints()
    {
        const float32_t vizRatio = static_cast<float32_t>(depthMap.size() * depthMap[0].size()) / VizPtsSize;
        for(auto&row: depthMap) {
            for (auto &s : row) {

                if(rand()/RAND_MAX >= vizRatio) continue;

                dwPoint t;

                Mat4_Axp(reinterpret_cast<float32_t *>(&t), icpRigToWorld.array, reinterpret_cast<const float32_t *>(&s));

                // We are coloring the points by height for clarity of the 3D view, so scaling height from 0m->4m
                t.intensity = (s.z + 2.5f)/4.f;
                accumulatedPointCloud.push_back(t);
            }
        }

        // update render buffer to keep latest points
        {
            float32_t *map;
            uint32_t maxVerts, stride;

            dwRenderBuffer_map(&map, &maxVerts, &stride, pointCloud);

            size_t size     = std::min(accumulatedPointCloud.size(), MaxNumPointsToRender);
            size_t startIdx = accumulatedPointCloud.size() < MaxNumPointsToRender ? 0 : accumulatedPointCloud.size() - MaxNumPointsToRender;

            float32_t* buffer = map;
            for (size_t idx = startIdx; idx < size + startIdx; idx++)
            {
                const dwPoint& t = accumulatedPointCloud[idx];
                buffer[0] = t.x;
                buffer[1] = t.y;
                buffer[2] = t.z;

                dwVector3f color  = interpolateColor(t.intensity);
                buffer[3] = color.x;
                buffer[4] = color.y;
                buffer[5] = color.z;

                buffer += stride;
            }

            dwRenderBuffer_unmap(maxVerts, pointCloud);
        }
    }

    //------------------------------------------------------------------------------
    // This function discards points that are too close or to far. Then random samples the remaining viable points
    // Exception being a point at origin that is pushed.
    // We do not use the intensity information in this sample, thus we over-write it to store the distance in intensity.
    void sample(dwPCD& full, dwPCD& sampled, uint32_t sampleSize)
    {
        sampled.clear();
        sampled.reserve(sampleSize);

        auto filterCriterion = [](dwLidarPointXYZI &pt) {
            static const float32_t minDist2 = 4 * 4;
            static const float32_t maxDist2 = 50 * 50;
            auto dist2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            return dist2 > minDist2 && dist2 < maxDist2;
        };

        int64_t numViable = std::count_if(full.begin(), full.end(), filterCriterion);
        float32_t ratio = float32_t(sampleSize) / numViable;

        sampled.push_back({0, 0, 0, 1});

        // Random sampling:
        for (auto &p : full)
            if (float64_t(rand()) / RAND_MAX < ratio && filterCriterion(p))
                sampled.push_back(p);

        // clamp the size.
        if (sampled.size() > sampleSize) sampled.resize(sampleSize);
    }

    //------------------------------------------------------------------------------
    // Fetch a sping from lidar. Template argument controls if points are pushed to array or discarded.
    // Returns the time of the last packet that makes up this spin.
    // It arranges the points in a map that spans 360*3 (with 3pt/degree resolution) x 64 .
    bool getSpin(dwPCD &full, bool pushPoints)
    {
        // Lidar sampling that produces thes points scrambles their order. Two adjacent points
        // in a lidar packet are not adjacent in the world that the is being sampled.
        // they're in fact related in the ordering expressed by BeamOrder. I.e. first point
        // in packet is from 27th beam, next from 26th ... last from 38th beam, etc.
        // This reordering produces depthmap that is consistent with sampled 3d space.
        //! NOTE: This beam ordering is specific to the Vewlodyne HDL 64 E type velodyne. thus this sample work
        //! only with data produyced by that lidar setup.
        static const int32_t BeamOrder[64] = {
                27, 26,  5,  4, 25, 24, 31, 30,
                23, 22, 29, 28, 15, 14, 21, 20,
                13, 12, 19, 18, 11, 10, 17, 16,
                3,  2,  9,  8,  1,  0,  7,  6,
                59, 58, 37, 36, 57, 56, 63, 62,
                55, 54, 61, 60, 47, 46, 53, 52,
                45, 44, 51, 50, 43, 42, 49, 48,
                35, 34, 41, 40, 33, 32, 39, 38
        };

        // TODO : see why this needs to be fixed like this.
        dwLidarProperties props;
        dwSensorLidar_getProperties(&props, lidarSensor);
        if(props.spinFrequency == 10) props.packetsPerSpin = 580;

        // prepare the ordering map.
        if (pushPoints) {
            for(auto& v : depthMap) {
                v.clear();
                v.resize(360 * DepthMapHorzRes); // horizontal resolution is 2pts/degree.
            }
        }

        const auto rad2Deg = [](const float64_t rad) { return float64_t(rad * 180.f / M_PI); };

        for (uint32_t i = 0; i < props.packetsPerSpin; ++i) {

            const dwLidarDecodedPacket *pckt = nullptr;
            dwStatus stat = dwSensorLidar_readPacket(&pckt, 500000, lidarSensor);
            if( stat == DW_END_OF_STREAM)
                return false;
            else if( stat != DW_SUCCESS) {
                logError("Read packet from Lidar failed with %s\n", dwGetStatusName(stat));
                return false;
            }

            auto pktPts = reinterpret_cast<const dwPoint *>(pckt->points);
            if (pushPoints) {
                for (uint32_t j = 0; j < pckt->nPoints; j++) {
                    const dwPoint& pt = pktPts[j];
                    float32_t hD  = rad2Deg(atan2(pktPts[j].y, pt.x)) ;
                    if(hD < 0) hD += 360;

                    auto& row = depthMap[BeamOrder[j % 64]];
                    size_t ptIdx = size_t(hD * DepthMapHorzRes) % row.size();

                    row[ ptIdx ] = pktPts[j];
                }
            }

            dwSensorLidar_returnPacket(pckt, lidarSensor);
        }

        if (pushPoints) {
            full.clear(); // clear to remove points from previous calls.
            full.reserve(depthMap.size() * depthMap[0].size());

            for (uint32_t i = DepthMapBeamStart; i < DepthMapBeamEnd; ++i)
                full.insert(full.end(), depthMap[i].begin(), depthMap[i].end());
        }

        return true;
    }

    //------------------------------------------------------------------------------
    // Main loop that fetchs spins, runs ICP and return the time of last spin recieved
    // The transform from ICP is fetched and accumulated into g_icpTx that is used to transform this spin.
    // We need to sample the points down to a smaller size without which ICP module will throw.
    // We swap sampled source and target because source becomes targetfor next pair.
    //
    bool runLoop(dwPCD &source, dwPCD &tgt)
    {
        float64_t loopTime = 0;

        // skip some spins, to get larger temporal distance between consecutive spins
        for (uint32_t k = 0; k < numSkip; ++k) {
            if(!getSpin(source, false))
                return false;
        }

        clock_t start = clock();

        // get next source points
        if(!getSpin(source, true))
            return false;

        // subsample src
        dwPCD src;
        {
            sample(source, src, ICPPtsSize);
        }

        // run ICP and get transformation aligning src on target
        dwTransformation icpCurrentRigToPrevRig;
        {
            dwICPIterationParams iparams{};
            iparams.sourcePts  = src.data(), iparams.targetPts = tgt.data();
            iparams.nSourcePts = src.size(), iparams.nTargetPts = tgt.size();
            iparams.initialSource2Target = &icpPriorPose;

            CHECK_DW_ERROR_MSG(dwICP_optimize(&icpCurrentRigToPrevRig, &iparams, icpHandle), "ICP Failed");
        }

        icpPriorPose = icpCurrentRigToPrevRig;

        loopTime = 1000*(float64_t(clock()) - start ) / CLOCKS_PER_SEC;

        // accumulate transformation (and fix Rotation matrix)
        icpRigToWorld *= icpCurrentRigToPrevRig;
        Mat4_RenormR(icpRigToWorld.array);

        // Get some stats about the ICP performance
        dwICPResultStats icpResultStats;
        CHECK_DW_ERROR_MSG(dwICP_getLastResultStats(&icpResultStats, icpHandle), "Couldn't get ICP result stats.");

        cout << "Loop Time: " << loopTime << "ms" << endl
             << "Number of Iterations: " << icpResultStats.actualNumIterations << endl
             << "RMS cost: " << icpResultStats.rmsCost << endl
             << "Inlier fraction: " << icpResultStats.inlierFraction << endl
             << "ICP Spin Transform: " << icpCurrentRigToPrevRig << endl
             << "Full Transform: " << icpRigToWorld << endl;

        tgt = src;

        return true;
    }


    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        if (m_pause) return;

        if (spinNum++ >= numFrames || !runLoop(curSourceFull, curTargetSubsampled))
        {
            pause();
            return;
        }

        accumulateCurrentPoints();

        cout << "----------Spin: " << spinNum << "----------\n";
    }


    ///------------------------------------------------------------------------------
    /// Initialize World Grid
    ///------------------------------------------------------------------------------
    void constructGrid()
    {
        // World grid
        int32_t gridResolution =
                static_cast<int32_t>(WORLD_GRID_SIZE_IN_METERS / WORLD_GRID_RES_IN_METERS);

        // Rendering data
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        dwRenderBuffer_initialize(&groundPlane, layout, DW_RENDER_PRIM_LINELIST,
                                  2 * (gridResolution + 1), sdkContext);

        // update the data
        float32_t *map;
        uint32_t maxVerts, stride;

        dwRenderBuffer_map(&map, &maxVerts, &stride, groundPlane);

        int32_t nVertices = 0;
        float32_t x, y;

        // Horizontal lines
        x = -0.5f * WORLD_GRID_SIZE_IN_METERS;
        for (int32_t i = 0; i <= gridResolution; ++i) {
            y = -0.5f * WORLD_GRID_SIZE_IN_METERS;

            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;
            nVertices++;

            y = 0.5f * WORLD_GRID_SIZE_IN_METERS;
            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;

            nVertices++;
            x = x + WORLD_GRID_RES_IN_METERS;
        }

        // Vertical lines
        y = -0.5f * WORLD_GRID_SIZE_IN_METERS;
        for (int32_t i = 0; i <= gridResolution; ++i) {
            x = -0.5f * WORLD_GRID_SIZE_IN_METERS;

            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;
            nVertices++;

            x = 0.5f * WORLD_GRID_SIZE_IN_METERS;
            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = -0.05f;

            nVertices++;
            y = y + WORLD_GRID_RES_IN_METERS;
        }

        dwRenderBuffer_unmap(maxVerts, groundPlane);
    }

    ///------------------------------------------------------------------------------
    /// Init point cloud rendering
    ///------------------------------------------------------------------------------
    void preprarePointCloudRenderBuffer()
    {
        // RenderBuffer
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32A32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
        layout.colFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        // Initialize Full Spin Point cloud
        dwRenderBuffer_initialize(&pointCloud, layout, DW_RENDER_PRIM_POINTLIST, MaxNumPointsToRender, sdkContext);
    }

    ///------------------------------------------------------------------------------
    /// Render Lidar Point Cloud
    ///------------------------------------------------------------------------------
    void onRender() override
    {
        glDepthFunc(GL_LESS);

        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // compute modelview by moving camera to the center of the current estimated position
        {
            float32_t center[3] = {icpRigToWorld.array[0 + 3 * 4], icpRigToWorld.array[1 + 3 * 4], icpRigToWorld.array[2 + 3 * 4]};

            if (!m_pause)
            {
                m_mouseView.setCenter(center[0] + centerViewDiff.x, center[1] + centerViewDiff.y, center[2] + centerViewDiff.z);
            }else {
                // store current difference to center, to keep it when not paused
                centerViewDiff.x = m_mouseView.getCenter()[0] - center[0];
                centerViewDiff.y = m_mouseView.getCenter()[1] - center[1];
                centerViewDiff.z = m_mouseView.getCenter()[2] - center[2];
            }
        }

        // 3D rendering
        dwRenderer_setModelView(m_mouseView.getModelView(), renderer);
        dwRenderer_setProjection(m_mouseView.getProjection(), renderer);

        dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREY, renderer);
        dwRenderer_renderBuffer(groundPlane, renderer);

        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
        dwRenderer_renderBuffer(pointCloud, renderer);
    }

};


//------------------------------------------------------------------------------
int32_t main(int32_t argc, const char **argv)
{
    // -------------------
    // define all arguments used by the application
    const std::string defLidarFile = (DataPath::get() + "/samples/icp/lidar_velodyne64.bin");
    typedef ProgramArguments::Option_t opt;

    ProgramArguments args(argc, argv,
    {
        opt("lidarFile", defLidarFile.c_str(),  "Path to lidar file, needs to be DW captured Velodyne HDL-64E file."),
        opt("plyloc",    "",                    "If specified use this directory to write ICP-fused ASCII-PLY file."),
        opt("init",      "20",                  "These many initial spins are skipped before first pair is fed to ICP."),
        opt("skip",      "0",                   "Number of frames to skip to perform ICP between a pair."),
        opt("numFrames", "0",                   "These many pairs are used to perform ICP before stopping. 0 - all frames"),
        opt("maxIters",  "12",                  "Number of ICP iterataions to run."),
    });

    // -------------------
    // initialize and start a window application
    LidarICP app(args);

    app.initializeWindow("Lidar ICP", 1280, 800);

    return app.run();
}

