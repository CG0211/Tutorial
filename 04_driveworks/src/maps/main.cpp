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

// Sample includes

#include <framework/Checks.hpp>
#include <framework/DataPath.hpp>
#include <framework/SampleFramework.hpp>
#include <framework/MathUtils.hpp>

#include <maps/GPSPathData.hpp>

// Driveworks includes
#include <dw/core/Context.h>
#include <dw/maps/Maps.h>

// stl includes
#include <cmath>
#include <limits>
#include <chrono>

//
// sample constants
//

// sample window
static const uint32_t windowWidth = 1800;
static const uint32_t windowHeight = 600;

// framerate
static const std::chrono::milliseconds targetFrameTime = static_cast<std::chrono::milliseconds>(33);

// camera settings
static const float32_t cameraAngle = 90.0f;
static const float32_t overviewHeight = 3500.0f;
static const float32_t zoomedHeight = 50.0f;
static const float32_t localHeight = 1.5f;

// map settings
static const float32_t boundsRadius = 300.0f;

// buffer sizes global
static const uint32_t maxRoadSegmentCount = 10000;
static const float32_t avgLaneDividerCountPerSegment = 3.0f;
static const float32_t avgPointCountPerLaneDivider = 10.0f;
static const uint32_t maxLaneDividerCount = static_cast<uint32_t>(avgLaneDividerCountPerSegment *
                                                                  maxRoadSegmentCount);
static const uint32_t maxPointCountTotal = static_cast<uint32_t>(avgPointCountPerLaneDivider *
                                                                 maxLaneDividerCount);

// buffer sizes local
static const uint32_t maxRoadSegmentCountLocal = 1000;
static const uint32_t maxPointCountLocal = static_cast<uint32_t>(avgLaneDividerCountPerSegment *
                                                                 avgPointCountPerLaneDivider *
                                                                 maxRoadSegmentCountLocal);

// colors
static const float32_t white[3] = {1.0f, 1.0f, 1.0f};
static const float32_t grey[3] = {0.3f, 0.3f, 0.3f};
static const float32_t green[3] = {0.1f, 0.6f, 0.4f};
static const float32_t blue[3] = {0.3f, 0.5f, 1.0f};
static const float32_t red[3] = {1.0f, 0.0f, 0.0f};


//#######################################################################################
// sample
//#######################################################################################

// compute bearing from 2 gps points
bool computeBearing(float32_t *bearing, const dwMapsGeoPoint &p, const dwMapsGeoPoint &pTarget)
{
    // first point as reference frame
    dwVector3f pTargetLocalCoords;
    dwMaps_transformPoint(&pTargetLocalCoords, &pTarget, &p, nullptr);

    // vector in the x-y plane of reference frame
    float32_t norm = sqrt(pTargetLocalCoords.x * pTargetLocalCoords.x +
                          pTargetLocalCoords.y * pTargetLocalCoords.y);

    // only update bearing if points are sufficiently far apart
    if (norm < 1.0f)
        return false;

    float32_t dx = pTargetLocalCoords.x / norm;
    float32_t dy = pTargetLocalCoords.y / norm;

    // transform angle into bearing
    *bearing = -atan2(dy, dx) + M_PI_2;

    if (*bearing < 0.0f)
        *bearing += 2.0*M_PI;

    return true;
}


// This function serves as a placeholder for localization.
// GPS positions are read from a recorded path.
void localize(dwMapsGeoPoint *p, float32_t *bearing)
{
    // cycle through gps points
    static uint32_t currentGPSPointIdx = 0;

    float64_t *currentGPSPoint = &g_gpsPath[2*currentGPSPointIdx];

    p->lon = currentGPSPoint[0];
    p->lat = currentGPSPoint[1];
    p->height = 0.0;

    // use next point to compute bearing
    float64_t *nextGPSPoint = &g_gpsPath[2*(currentGPSPointIdx+1)];
    dwMapsGeoPoint pNext;
    pNext.lon = nextGPSPoint[0];
    pNext.lat = nextGPSPoint[1];
    pNext.height = 0;
    computeBearing(bearing, *p, pNext);

    currentGPSPointIdx = (currentGPSPointIdx + 1) % (g_gpsPathPointCount-1);
}


float32_t dot(float32_t x1, float32_t y1,
              float32_t x2, float32_t y2)
{
    return x1*x2 + y1*y2;
}


bool isLineSegmentInCarDirection(const dwMapsLine &l,
                                 float32_t headingX, float32_t headingY,
                                 float32_t filterAngle)
{
    // normalized direction of line segment
    float32_t l_dx = l.p[1].x - l.p[0].x;
    float32_t l_dy = l.p[1].y - l.p[0].y;
    float32_t lineLength = sqrt(l_dx*l_dx + l_dy*l_dy);
    if (lineLength < std::numeric_limits<float32_t>::epsilon())
        return false;

    l_dx /= lineLength;
    l_dy /= lineLength;

    // compute angle
    float32_t angle = std::abs(std::acos(dot(l_dx, l_dy, headingX, headingY)));

    // ignore point order in line segment
    if (angle > M_PI_2) {
        angle = M_PI - angle;
    }

    return angle < filterAngle;
}


// Return point on 3d line segment that's closest in the projection onto the x-y plane
// (ignoring height).
dwVector3f findClosestOnLineSegment(const dwVector3f &p, const dwMapsLine &lineSegment)
{
    const dwVector3f &p0 = lineSegment.p[0];
    const dwVector3f &p1 = lineSegment.p[1];

    // segment projected onto x-y plane
    dwVector3f s;
    s.x = p1.x - p0.x;
    s.y = p1.y - p0.y;
    s.z = 0.0;

    float32_t l = sqrt(s.x * s.x +
                       s.y * s.y);

    if (l < std::numeric_limits<float32_t>::epsilon()) {
        return p0;
    }

    // normalized direction of projected line segment
    s.x /= l;
    s.y /= l;

    // vector from segment start to query point
    dwVector3f p0_p;
    p0_p.x = p.x - p0.x;
    p0_p.y = p.y - p0.y;
    p0_p.z = 0.0;

    // project p0_p onto the 2d segment s
    float32_t param = dot(s.x, s.y, p0_p.x, p0_p.y);

    dwVector3f closest;
    if (param <= 0.0f) {
        // segment start point is closest
        closest = p0;
    }
    else if (param >= l) {
        // segment end point is closest
        closest = p1;
    }
    else
    {
        // closest point on xy plane
        closest.x = p0.x + param * s.x;
        closest.y = p0.y + param * s.y;

        // also interpolate the z value accordingly
        closest.z = p0.z + param * ((p1.z - p0.z)/l);
    }

    return closest;
}


// Return point that's closest in the projection onto the x-y plane
// (ignoring height)
// If filterAngle is > 0.0f only line segments are considered that have an angle
// to the driving direciton that is smaller than filterAngle.
bool findClosest(dwVector3f *closest,
                 const dwVector3f &p, const dwMapsLine* lineSegments, uint32_t lineCount,
                 float32_t bearing, float32_t filterAngle)
{
    float32_t headingX = sin(bearing);
    float32_t headingY = cos(bearing);

    float32_t minDistanceSquared = std::numeric_limits<float32_t>::max();
    bool found = false;
    for (uint32_t i = 0; i < lineCount; ++i) {
        const dwMapsLine &lineSegment = lineSegments[i];

        if (filterAngle > 0.0f) {
            if (!isLineSegmentInCarDirection(lineSegment, headingX, headingY, filterAngle))
                continue;
        }

        dwVector3f closestOnSegment = findClosestOnLineSegment(p, lineSegment);

        // keep point that's closest in x-y plane projection
        float32_t dx = closestOnSegment.x - p.x;
        float32_t dy = closestOnSegment.y - p.y;
        float32_t distanceSquared = dx*dx + dy*dy;
        if (distanceSquared < minDistanceSquared) {
            minDistanceSquared = distanceSquared;
            *closest = closestOnSegment;
            found = true;
        }
    }

    return found;
}


// Set the height of point 'p' based on the closest map data.
void adjustHeight(dwMapsGeoPoint *p,
                  const dwMapsGeoPoint &origin,
                  float32_t bearing,
                  const dwMapsLine* lines, uint32_t lineCount)
{
    // transform point p into same render coordinate space as the lines
    dwVector3f pTransformed;
    dwMaps_transformPoint(&pTransformed, p, &origin, nullptr);

    dwVector3f closest = pTransformed;
    bool found = findClosest(&closest, pTransformed, lines, lineCount, bearing, 0.2 * M_PI);
    if (!found) {
        // if nothing is found, search again without direction filter
        found = findClosest(&closest, pTransformed, lines, lineCount, bearing, 0.0f);
    }

    // adjust the height of the point such that it matches the height of closest map data
    float32_t heightCorrection = closest.z - pTransformed.z;
    p->height += heightCorrection;
}


//
// dwMapsBounds helpers
//

void initBounds(dwMapsBounds *bounds)
{
    bounds->minLon = std::numeric_limits<float64_t>::max();
    bounds->minLat = std::numeric_limits<float64_t>::max();
    bounds->maxLon = -std::numeric_limits<float64_t>::max();
    bounds->maxLat = -std::numeric_limits<float64_t>::max();
}

void boundsInclude(dwMapsBounds *bounds, const dwMapsGeoPoint &p)
{
    if (p.lon < bounds->minLon) bounds->minLon = p.lon;
    if (p.lon > bounds->maxLon) bounds->maxLon = p.lon;
    if (p.lat < bounds->minLat) bounds->minLat = p.lat;
    if (p.lat > bounds->maxLat) bounds->maxLat = p.lat;
}

bool boundsValid(const dwMapsBounds &bounds)
{
    return bounds.minLon <= bounds.maxLon && bounds.minLat <= bounds.maxLat;
}

bool isInBounds(const dwMapsGeoPoint &position, const dwMapsBounds &bounds)
{
    return position.lon > bounds.minLon &&
           position.lon < bounds.maxLon &&
           position.lat > bounds.minLat &&
           position.lat < bounds.maxLat;
}

void computeRouteBounds(dwMapsBounds *bounds)
{
    initBounds(bounds);
    for (uint32_t i = 0; i < g_gpsPathPointCount; ++i)
    {
        float64_t *point = &g_gpsPath[2*i];

        dwMapsGeoPoint geoPoint;
        geoPoint.lon = point[0];
        geoPoint.lat = point[1];
        geoPoint.height = 0.0;
        boundsInclude(bounds, geoPoint);
    }
}


//#######################################################################################
// rendering
//#######################################################################################

struct Color
{
    Color(const float32_t color[3]) {
        rgb[0] = color[0];
        rgb[1] = color[1];
        rgb[2] = color[2];
    }

    Color() : Color(white) {}

    float32_t rgb[3];
};


struct RenderData
{
    float32_t pointSize;
    float32_t lineWidth;
    dwRenderBufferHandle_t buffer;
};


struct RenderView
{
    dwRendererHandle_t renderer;
    dwRect rectangle;
    float32_t projection[16];
    float32_t modelview[16];
    std::vector<RenderData> renderbuffers;
};

void initRendering(std::vector<RenderView> *renderViews,
                   dwRenderBufferHandle_t *lineRenderBufferTotal,
                   dwRenderBufferHandle_t *lineRenderBufferLocal,
                   dwRenderBufferHandle_t *pointRenderBuffer,
                   dwRendererHandle_t *renderer,
                   const dwContextHandle_t &dwContext)
{

    CHECK_DW_ERROR( dwRenderer_initialize(renderer, dwContext) );

    // init render buffers
    dwRenderBufferVertexLayout layout;
    layout.posFormat = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
    layout.colFormat = DW_RENDER_FORMAT_R32G32B32_FLOAT;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
    layout.texFormat = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;

    dwRenderBuffer_initialize(lineRenderBufferTotal, layout, DW_RENDER_PRIM_LINELIST,
                              maxPointCountTotal, dwContext);

    dwRenderBuffer_initialize(lineRenderBufferLocal, layout, DW_RENDER_PRIM_LINELIST,
                              maxPointCountLocal, dwContext);

    dwRenderBuffer_initialize(pointRenderBuffer, layout, DW_RENDER_PRIM_POINTLIST,
                              maxPointCountLocal, dwContext);

    renderViews->resize(3);

    RenderData renderDataTotal;
    renderDataTotal.buffer = *lineRenderBufferTotal;
    renderDataTotal.lineWidth = 1;
    renderDataTotal.pointSize = 1;

    RenderData renderDataLocalThick;
    renderDataLocalThick.buffer = *lineRenderBufferLocal;
    renderDataLocalThick.lineWidth = 3;
    renderDataLocalThick.pointSize = 1;

    RenderData renderDataLocalThin;
    renderDataLocalThin.buffer = *lineRenderBufferLocal;
    renderDataLocalThin.lineWidth = 1;
    renderDataLocalThin.pointSize = 1;

    RenderData renderDataPoints;
    renderDataPoints.buffer = *pointRenderBuffer;
    renderDataPoints.lineWidth = 1;
    renderDataPoints.pointSize = 5;

    float32_t eye[3] = {0.0f, 0.0f, 0.0f};
    float32_t lookAtPoint[3] = {0.0f, 0.0f, 0.0f};
    float32_t cameraUp[3] = {0.0f, 1.0f, 0.0f};

    float32_t windowAspect = static_cast<float32_t>(windowWidth / 3) / windowHeight;

    // high bird eye
    (*renderViews)[0].renderer = *renderer;
    (*renderViews)[0].rectangle.x = 0;
    (*renderViews)[0].rectangle.y = 0;
    (*renderViews)[0].rectangle.width = windowWidth / 3;
    (*renderViews)[0].rectangle.height = windowHeight;
    (*renderViews)[0].renderbuffers.push_back(renderDataTotal);
    (*renderViews)[0].renderbuffers.push_back(renderDataLocalThick);
    (*renderViews)[0].renderbuffers.push_back(renderDataPoints);
    perspective((*renderViews)[0].projection, cameraAngle, windowAspect, 0.01f, 10000000.0f);
    eye[2] = overviewHeight;
    lookAt((*renderViews)[0].modelview, eye, lookAtPoint, cameraUp);

    // low bird eye
    (*renderViews)[1].renderer = *renderer;
    (*renderViews)[1].rectangle.x = windowWidth / 3;
    (*renderViews)[1].rectangle.y = 0;
    (*renderViews)[1].rectangle.width = windowWidth / 3;
    (*renderViews)[1].rectangle.height = windowHeight;
    (*renderViews)[1].renderbuffers.push_back(renderDataLocalThin);
    (*renderViews)[1].renderbuffers.push_back(renderDataPoints);
    perspective((*renderViews)[1].projection, cameraAngle, windowAspect, 0.01f, 10000000.0f);
    // modelview updated at runtime

    // local view
    (*renderViews)[2].renderer = *renderer;
    (*renderViews)[2].rectangle.x = 2 * windowWidth / 3;
    (*renderViews)[2].rectangle.y = 0;
    (*renderViews)[2].rectangle.width = windowWidth / 3;
    (*renderViews)[2].rectangle.height = windowHeight;
    (*renderViews)[2].renderbuffers.push_back(renderDataLocalThin);
    perspective((*renderViews)[2].projection, cameraAngle, windowAspect, 0.01f, 10000000.0f);
    // modelview updated at runtime
}


void releaseRendering(dwRenderBufferHandle_t *lineRenderBufferTotal,
                      dwRenderBufferHandle_t *lineRenderBufferLocal,
                      dwRenderBufferHandle_t *pointRenderBuffer,
                      dwRendererHandle_t *renderer)
{
    dwRenderBuffer_release(lineRenderBufferTotal);
    dwRenderBuffer_release(lineRenderBufferLocal);
    dwRenderBuffer_release(pointRenderBuffer);
    dwRenderer_release(renderer);
}


// render buffer update
void updateLineRenderBuffer(dwRenderBufferHandle_t *renderBuffer,
                            const dwMapsLine *lines,
                            const Color *colors,
                            uint32_t lineCount)
{
    float32_t *lineBuffer = nullptr;
    uint32_t maxVertexCount = 0;
    uint32_t vertexStride = 0;
    dwRenderBuffer_map(&lineBuffer, &maxVertexCount, &vertexStride, *renderBuffer);

    uint32_t vertexCount = 0;
    for (uint32_t i = 0; i < lineCount && vertexCount < maxVertexCount; ++i)
    {
        const dwMapsLine &lineSegment = lines[i];
        for (uint32_t j = 0; j < 2; ++j) {
            lineBuffer[0] = lineSegment.p[j].x;
            lineBuffer[1] = lineSegment.p[j].y;
            lineBuffer[2] = lineSegment.p[j].z;

            lineBuffer[3] = colors[i].rgb[0];
            lineBuffer[4] = colors[i].rgb[1];
            lineBuffer[5] = colors[i].rgb[2];
            lineBuffer += vertexStride;
            ++vertexCount;
        }
    }

    dwRenderBuffer_unmap(vertexCount, *renderBuffer);
}


// render buffer update
void updatePointRenderBuffer(dwRenderBufferHandle_t *renderBuffer,
                             const dwVector3f *points, const Color *colors,
                             uint32_t pointCount)
{
    float32_t *pointBuffer = nullptr;
    uint32_t maxVertexCount = 0;
    uint32_t vertexStride = 0;
    dwRenderBuffer_map(&pointBuffer, &maxVertexCount, &vertexStride, *renderBuffer);

    uint32_t vertexCount = 0;
    for (uint32_t i = 0; i < pointCount; ++i)
    {
        const dwVector3f &p = points[i];
        pointBuffer[0] = p.x;
        pointBuffer[1] = p.y;
        pointBuffer[2] = p.z;

        pointBuffer[3] = colors[i].rgb[0];
        pointBuffer[4] = colors[i].rgb[1];
        pointBuffer[5] = colors[i].rgb[2];
        pointBuffer += vertexStride;
        ++vertexCount;
    }

    dwRenderBuffer_unmap(vertexCount, *renderBuffer);
}


//
// camera update
//

void updateBirdEyeCamera(float32_t modelview[16],
                         const dwMapsGeoPoint &currentPoint,
                         const dwMapsGeoPoint &origin)
{
    dwVector3f lookAtPoint;
    dwMaps_transformPoint(&lookAtPoint, &currentPoint, &origin, nullptr);

    dwVector3f eye = lookAtPoint;
    eye.z += zoomedHeight;

    float32_t cameraUp[3] = {0.0f, 1.0f, 0.0f};
    lookAt(modelview, &eye.x, &lookAtPoint.x, cameraUp);
}

void updateLocalViewCamera(float32_t modelview[16],
                           const dwMapsGeoPoint &currentPoint,
                           float32_t bearing,
                           const dwMapsGeoPoint &origin)
{
    dwVector3f eye;
    dwMaps_transformPoint(&eye, &currentPoint, &origin, nullptr);
    eye.z += localHeight;

    dwVector3f lookAtPoint;
    lookAtPoint.x = eye.x + sinf(bearing);
    lookAtPoint.y = eye.y + cosf(bearing);
    lookAtPoint.z = eye.z;

    float32_t cameraUp[3] = {0.0f, 0.0f, 1.0f};
    lookAt(modelview, &eye.x, &lookAtPoint.x, cameraUp);
}

void updateCameras(std::vector<RenderView> *renderViews,
                   const dwMapsGeoPoint &currentPosition,
                   const dwMapsGeoPoint &renderCoordSystemOrigin,
                   float32_t bearing)
{
    updateBirdEyeCamera((*renderViews)[1].modelview, currentPosition, renderCoordSystemOrigin);
    updateLocalViewCamera((*renderViews)[2].modelview, currentPosition, bearing, renderCoordSystemOrigin);
}


// render information text with neighbor lane information
void drawLaneInfoText(uint32_t laneCountLeft, uint32_t laneCountLeftAccessible,
                      uint32_t laneCountRight, uint32_t laneCountRightAccessible,
                      uint32_t x, uint32_t y, dwRendererHandle_t renderer)
{
    // table offsets
    uint32_t xOffsets[3] = {x, x + 90, x + 140};
    uint32_t yOffsets[4] = {y, y - 40, y - 60, y - 80};

    // table entry descrptions
    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
    dwRenderer_renderText(xOffsets[0], yOffsets[0], "Number of Neighbor Lanes:", renderer);
    dwRenderer_renderText(xOffsets[1], yOffsets[1], "Left", renderer);
    dwRenderer_renderText(xOffsets[2], yOffsets[1], "Right", renderer);
    dwRenderer_renderText(xOffsets[0], yOffsets[2], "Total", renderer);
    dwRenderer_renderText(xOffsets[0], yOffsets[3], "Accessible", renderer);


    // table entries
    std::string numberString;

    // left
    numberString = std::to_string(laneCountLeft);
    dwRenderer_renderText(xOffsets[1], yOffsets[2], numberString.c_str(), renderer);

    numberString = std::to_string(laneCountLeftAccessible);
    dwRenderer_renderText(xOffsets[1], yOffsets[3], numberString.c_str(), renderer);

    // right
    numberString = std::to_string(laneCountRight);
    dwRenderer_renderText(xOffsets[2], yOffsets[2], numberString.c_str(), renderer);

    numberString = std::to_string(laneCountRightAccessible);
    dwRenderer_renderText(xOffsets[2], yOffsets[3], numberString.c_str(), renderer);
}


// render
void render(const std::vector<RenderView> &renderViews,
            uint32_t laneCountLeft, uint32_t laneCountLeftAccessible,
            uint32_t laneCountRight, uint32_t laneCountRightAccessible)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (uint32_t i = 0; i < renderViews.size(); ++i) {
        const RenderView &view = renderViews[i];
        dwRendererHandle_t renderer = view.renderer;

        dwRenderer_setRect(view.rectangle, renderer);
        dwRenderer_setProjection(view.projection, renderer);
        dwRenderer_setModelView(view.modelview, renderer);

        for (uint32_t j = 0; j < view.renderbuffers.size(); ++j) {
            const RenderData &renderData = view.renderbuffers[j];

            dwRenderer_setPointSize(renderData.pointSize, renderer);
            dwRenderer_setLineWidth(renderData.lineWidth, renderer);
            dwRenderer_renderBuffer(renderData.buffer, renderer);
        }

        // render lane count info in 3rd view rectangle
        if (i == 2) {
            uint32_t x = view.rectangle.width - 240;
            uint32_t y = view.rectangle.height - 40;

            drawLaneInfoText(laneCountLeft, laneCountLeftAccessible,
                             laneCountRight, laneCountRightAccessible,
                             x, y, renderer);
        }

        // render courtesy indicator
        if (i == 0) {
            dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
            dwRenderer_renderText(20, 20, "HD Map data courtesy of TomTom", renderer);
        }
    }

    gWindow->swapBuffers();
}



//#######################################################################################
// driveworks
//#######################################################################################

void initDriveworks(dwContextHandle_t *dwContext)
{
    // init driveworks
    dwInitialize(dwContext, DW_VERSION, nullptr);

    // logger
    dwStatus result = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init logger: ") + dwGetStatusName(result));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);
}


void releaseDriveworks(dwContextHandle_t *dwContext)
{
    dwLogger_release();
    dwRelease(dwContext);
}


//#######################################################################################
// maps
//#######################################################################################

// load maps data
bool initMaps(dwMapHandle_t *map, const dwContextHandle_t dwContext)
{
    // paths to map files
    std::string dwMapCacheFile(DataPath::get());
    dwMapCacheFile.append("/samples/maps/dwMapCache.bin");

    // check if the cache file is there
    FILE *f = fopen(dwMapCacheFile.c_str(), "rb");
    if (f == nullptr) {
        printf("Could not find dwMapCache.bin file at \"%s\". "
               "Run the sample from the \"bin\" directory.\n", dwMapCacheFile.c_str());
        return false;
    }
    else {
        fclose(f);
    }

    // init
    dwMaps_initialize(map,
                      dwMapCacheFile.c_str(),
                      dwContext);

    return true;
}


void releaseMaps(dwMapHandle_t *map)
{
    dwMaps_release(map);
}


// Compute bounds whenever we approach the curren bounds' border.
bool updateBounds(dwMapsBounds *bounds, const dwMapsGeoPoint &position)
{
    // update if current bounds not valid
    bool doUpdate = !boundsValid(*bounds);

    if (!doUpdate) {
        // check if position is outside of inner part of bounds
        float64_t lonWidth = bounds->maxLon - bounds->minLon;
        float64_t latWidth = bounds->maxLat - bounds->minLat;

        dwMapsBounds innerBounds;
        innerBounds.minLon = bounds->minLon + 0.25 * lonWidth;
        innerBounds.maxLon = bounds->maxLon - 0.25 * lonWidth;
        innerBounds.minLat = bounds->minLat + 0.25 * latWidth;
        innerBounds.maxLat = bounds->maxLat - 0.25 * latWidth;
        doUpdate = !isInBounds(position, innerBounds);
    }

    if (doUpdate)
    {
        // compute wgs84 bounds for a given geodesic radius
        dwMaps_computeBounds(bounds, &position, boundsRadius);
    }
    return doUpdate;
}


// extract polylines from lane dividers
void getLaneDividerPolylines(std::vector<dwMapsGeoPolyline> *polylines,
                             std::vector<dwMapsLaneDividerType> *types,
                             const dwMapsLaneDividerBuffer &laneDividers)
{
    for (uint32_t i = 0; i < laneDividers.size; ++i) {
        polylines->push_back(laneDividers.buffer[i].geometry);
        types->push_back(laneDividers.buffer[i].type);
    }
}


// extract lane divider polylines from road segments
void getLaneDividerPolylines(std::vector<dwMapsGeoPolyline> *polylines,
                             std::vector<dwMapsLaneDividerType> *types,
                             const dwMapsRoadSegment* roadSegments,
                             uint32_t roadSegmentCount)
{
    for (uint32_t i = 0; i < roadSegmentCount; ++i) {
        const dwMapsRoadSegment &roadSegment = roadSegments[i];

        for (uint32_t j = 0; j < roadSegment.laneDividerGroupCount; ++j) {
            const dwMapsLaneDividerGroup &laneDividerGroup = roadSegment.laneDividerGroups[j];

            for (uint32_t k = 0; k < laneDividerGroup.laneDividerCount; ++k) {
                const dwMapsLaneDivider &laneDivider = laneDividerGroup.laneDividers[k];
                polylines->push_back(laneDivider.geometry);
                types->push_back(laneDivider.type);
            }
        }
    }
}


// extract lane polylines from road segments
void getLanePolylines(std::vector<dwMapsGeoPolyline> *polylines,
                      const dwMapsRoadSegment* roadSegments,
                      uint32_t roadSegmentCount)
{
    for (uint32_t i = 0; i < roadSegmentCount; ++i) {
        const dwMapsRoadSegment &roadSegment = roadSegments[i];

        for (uint32_t j = 0; j < roadSegment.laneCount; ++j) {
            const dwMapsLane &lane = roadSegment.lanes[j];
            polylines->push_back(lane.geometry);
        }
    }
}


// check if a lane divider is allowed to be traversed
bool isTraversable(dwMapsLaneDividerType laneDividerType)
{
    bool result = false;
    switch (laneDividerType)
    {

    // traversable
    case DW_MAPS_LANE_DIVIDER_TYPE_INVISIBLE:
    case DW_MAPS_LANE_DIVIDER_TYPE_LONG_DASHED:
    case DW_MAPS_LANE_DIVIDER_TYPE_SOLID_DASHED: // only one way
    case DW_MAPS_LANE_DIVIDER_TYPE_DASHED_SOLID: // only one way
    case DW_MAPS_LANE_DIVIDER_TYPE_SHORT_DASHED:
    case DW_MAPS_LANE_DIVIDER_TYPE_DASHED_BLOCKS:
    case DW_MAPS_LANE_DIVIDER_TYPE_DOUBLE_DASHED:
    case DW_MAPS_LANE_DIVIDER_TYPE_CENTER_TURN_LANE:
        result = true;
        break;

    // non-traversable
    case DW_MAPS_LANE_DIVIDER_TYPE_SINGLE_SOLID:
    case DW_MAPS_LANE_DIVIDER_TYPE_DOUBLE_SOLID:
    case DW_MAPS_LANE_DIVIDER_TYPE_SHADED_AREA:
    case DW_MAPS_LANE_DIVIDER_TYPE_CROSSING_ALERT:
    case DW_MAPS_LANE_DIVIDER_TYPE_PHYSICAL:
    case DW_MAPS_LANE_DIVIDER_TYPE_CURB:
    case DW_MAPS_LANE_DIVIDER_TYPE_WALL_FLAT:
    case DW_MAPS_LANE_DIVIDER_TYPE_WALL_TUNNEL:
    case DW_MAPS_LANE_DIVIDER_TYPE_BARRIER_JERSEY:
    case DW_MAPS_LANE_DIVIDER_TYPE_BARRIER_SOUND:
    case DW_MAPS_LANE_DIVIDER_TYPE_BARRIER_CABLE:
    case DW_MAPS_LANE_DIVIDER_TYPE_GUARDRAIL:
    case DW_MAPS_LANE_DIVIDER_TYPE_FENCE:
        result = false;
        break;

    // invalid cases
    case DW_MAPS_LANE_DIVIDER_TYPE_ALL:
    default:
        result = false;
        break;
    }

    return result;
}


// Transform from wgs84 polylines into line segments in a local cartesian coordinate frame.
uint32_t transformToRenderLines(std::vector<dwMapsLine> *renderLines,
                                std::vector<Color> *colors,
                                const dwMapsGeoPoint &origin,
                                const std::vector<dwMapsGeoPolyline> &polylines,
                                const std::vector<dwMapsLaneDividerType> *types)
{
    // find point count of polylines
    uint32_t pointCount = 0;
    for (uint32_t i = 0; i < polylines.size(); ++i) {
        const dwMapsGeoPolyline &polyline = polylines[i];
        pointCount += polyline.pointCount;
    }

    // transform points in polylines
    std::vector<dwVector3f> points(pointCount);
    dwMapsPointBuffer localPoints;
    localPoints.buffer = points.data();
    localPoints.maxSize = points.size();
    localPoints.size = 0;

    dwMaps_transformPolylines(&localPoints,
                              polylines.data(), static_cast<uint32_t>(polylines.size()),
                              &origin, nullptr);

    // create line segments
    const dwVector3f *localPolylinePoints = localPoints.buffer;
    for (uint32_t i = 0; i < polylines.size(); ++i) {
        const dwMapsGeoPolyline &polyline = polylines[i];

        // color based on type
        Color color = Color(white);
        if (types != nullptr && i < types->size()) {
            dwMapsLaneDividerType type = (*types)[i];

            if (type == DW_MAPS_LANE_DIVIDER_TYPE_INVISIBLE) {
                color = Color(grey);
            }
            else if (isTraversable(type)) {
                color = Color(green);
            }
        }

        // add line segments
        for (uint32_t j = 1; j < polyline.pointCount; ++j) {
            dwMapsLine line;
            line.p[0] = localPolylinePoints[j-1];
            line.p[1] = localPolylinePoints[j];

            renderLines->push_back(line);

            if (colors != nullptr) {
                colors->push_back(color);
            }
        }

        localPolylinePoints += polyline.pointCount;
    }

    return renderLines->size();
}


//#######################################################################################
// main
//#######################################################################################
int main(int argc, const char **argv)
{
    // no arguments needed
    ProgramArguments arguments;

    // init sample
    initSampleApp(argc, argv, &arguments, nullptr, windowWidth, windowHeight);

    // init driveworks
    dwContextHandle_t dwContext;
    initDriveworks(&dwContext);

    // init rendering
    std::vector<RenderView> renderViews;
    dwRenderBufferHandle_t lineRenderBufferTotal = DW_NULL_HANDLE;
    dwRenderBufferHandle_t lineRenderBufferLocal = DW_NULL_HANDLE;
    dwRenderBufferHandle_t pointRenderBuffer = DW_NULL_HANDLE;
    dwRendererHandle_t renderer = DW_NULL_HANDLE;
    initRendering(&renderViews, &lineRenderBufferTotal, &lineRenderBufferLocal, &pointRenderBuffer,
                  &renderer, dwContext);

    // init maps
    dwMapHandle_t map;
    if (!initMaps(&map, dwContext)){
        releaseRendering(&lineRenderBufferTotal, &lineRenderBufferLocal, &pointRenderBuffer, &renderer);
        releaseDriveworks(&dwContext);
        releaseSampleApp();
        return 1;
    }

    // query all lane dividers in map data
    std::vector<dwMapsLaneDivider> laneDividersTotal(maxLaneDividerCount);
    dwMapsLaneDividerBuffer laneDividers;
    laneDividers.buffer = laneDividersTotal.data();
    laneDividers.maxSize = laneDividersTotal.size();
    laneDividers.size = 0;
    dwMaps_getLaneDividers(&laneDividers, DW_MAPS_LANE_DIVIDER_TYPE_ALL, nullptr, map);

    // find center of sample data
    dwMapsBounds boundsTotal;
    computeRouteBounds(&boundsTotal);
    dwMapsGeoPoint origin;
    origin.lon = 0.5*(boundsTotal.minLon + boundsTotal.maxLon);
    origin.lat = 0.5*(boundsTotal.minLat + boundsTotal.maxLat);
    origin.height = 0.0f;

    // transform map data into cartesian render coordinate syste
    // with origin at center of map data
    std::vector<dwMapsGeoPolyline> lanePolylines;
    std::vector<dwMapsGeoPolyline> laneDividerPolylines;
    std::vector<dwMapsLaneDividerType> laneDividerTypes;
    getLaneDividerPolylines(&laneDividerPolylines, &laneDividerTypes, laneDividers);

    std::vector<dwMapsLine> laneDividerLinesTotal;
    transformToRenderLines(&laneDividerLinesTotal,
                           nullptr,
                           origin,
                           laneDividerPolylines,
                           &laneDividerTypes);

    // update render buffer with map data
    std::vector<Color> laneDividerLineColorsTotal(laneDividerLinesTotal.size(), Color(blue));
    updateLineRenderBuffer(&lineRenderBufferTotal,
                           laneDividerLinesTotal.data(),
                           laneDividerLineColorsTotal.data(),
                           laneDividerLinesTotal.size());

    // arrays for point rendering
    std::vector<dwVector3f> renderPoints;
    std::vector<Color> renderPointColors;

    //
    // main loop
    //
    std::vector<dwMapsLine> laneDividerLinesLocal;
    std::vector<Color> laneDividerLineColorsLocal;

    std::vector<dwMapsLine> laneLinesLocal;

    std::vector<dwMapsRoadSegment> roadSegmentsLocal(maxRoadSegmentCountLocal);
    dwMapsRoadSegmentBuffer roadSegments;
    roadSegments.buffer = roadSegmentsLocal.data();
    roadSegments.maxSize = roadSegmentsLocal.size();

    dwMapsBounds bounds;
    initBounds(&bounds);

    auto tNow = std::chrono::high_resolution_clock::now();
    auto tStart = tNow;
    std::chrono::milliseconds tDiff(0);

    float32_t bearing = 0.0f;
    while(gRun && !gWindow->shouldClose()) {
        // clear this frame's buffers
        renderPoints.clear();

        // get current gps position
        dwMapsGeoPoint position;
        localize(&position, &bearing);

        // transform into render coordinates
        dwVector3f positionRenderCoord;
        dwMaps_transformPoint(&positionRenderCoord, &position, &origin, nullptr);
        renderPoints.push_back(positionRenderCoord);
        renderPointColors.push_back(red);

        // query map data around current position if necessary
        if (updateBounds(&bounds, position))
        {
            // get the local road segments
            roadSegments.size = 0;
            dwMaps_getRoadSegments(&roadSegments, &bounds, map);

            // exctract polylines from road segments
            laneDividerPolylines.clear();
            laneDividerTypes.clear();
            getLaneDividerPolylines(&laneDividerPolylines,
                                    &laneDividerTypes,
                                    roadSegments.buffer,
                                    roadSegments.size);

            lanePolylines.clear();
            getLanePolylines(&lanePolylines,
                             roadSegments.buffer,
                             roadSegments.size);

            // transform into cartesian render coordinates
            laneDividerLinesLocal.clear();
            laneDividerLineColorsLocal.clear();
            transformToRenderLines(&laneDividerLinesLocal,
                                   &laneDividerLineColorsLocal,
                                   origin,
                                   laneDividerPolylines,
                                   &laneDividerTypes);

            laneLinesLocal.clear();
            transformToRenderLines(&laneLinesLocal,
                                   nullptr,
                                   origin,
                                   lanePolylines,
                                   nullptr);

            // update render buffer for local lane divider lines
            updateLineRenderBuffer(&lineRenderBufferLocal,
                                   laneDividerLinesLocal.data(),
                                   laneDividerLineColorsLocal.data(),
                                   laneDividerLinesLocal.size());
        }

        // if localize only provides latitude and longitude, the current height
        // needs to set according to closest map data
        adjustHeight(&position, origin, bearing,
                     laneLinesLocal.data(), laneLinesLocal.size());


        // find closest lane
        const dwMapsLane *currentLane = nullptr;
        dwMaps_getClosestLane(&currentLane, nullptr, &position, DW_TRUE, DW_FALSE, map);

        // number of lanes
        uint32_t leftLaneCount = 0;
        uint32_t leftLaneCountAccessible = 0;
        uint32_t rightLaneCount = 0;
        uint32_t rightLaneCountAccessible = 0;
        if (currentLane != nullptr) {
            dwMaps_getNeighborLaneCount(&leftLaneCount, &leftLaneCountAccessible, currentLane,
                                        DW_MAPS_SIDE_LEFT, DW_TRUE);

            dwMaps_getNeighborLaneCount(&rightLaneCount, &rightLaneCountAccessible, currentLane,
                                        DW_MAPS_SIDE_RIGHT, DW_TRUE);
        }

        // fill point render buffer
        updatePointRenderBuffer(&pointRenderBuffer, renderPoints.data(), renderPointColors.data(), renderPoints.size());

        // render
        do {
            updateCameras(&renderViews, position, origin, bearing);
            render(renderViews,
                   leftLaneCount, leftLaneCountAccessible, rightLaneCount, rightLaneCountAccessible);

            tNow = std::chrono::high_resolution_clock::now();
            tDiff = std::chrono::duration_cast<std::chrono::milliseconds>(tNow - tStart);
        }
        while(gRun && (tDiff < targetFrameTime));
        tStart = tNow;
    }

    // release resources
    releaseMaps(&map);
    releaseRendering(&lineRenderBufferTotal, &lineRenderBufferLocal, &pointRenderBuffer, &renderer);
    releaseDriveworks(&dwContext);
    releaseSampleApp();

    return 0;
}
