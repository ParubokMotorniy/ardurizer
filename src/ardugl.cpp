#include "ardugl.h"

#include "glm.hpp"

#include <Arduino.h>

#include <cstring>
#include <malloc.h>
#include <vector>

extern char __HeapBase;
extern char __StackTop;

static void printRamInfo()
{
    constexpr uintptr_t RAM_START = 0x20000000;
    constexpr size_t RAM_SIZE = 32 * 1024;

    char stackMarker;
    auto mi = mallinfo();

    size_t staticUsed = (uintptr_t)&__HeapBase - RAM_START;
    size_t heapUsed = mi.uordblks;
    size_t stackUsed = (uintptr_t)&__StackTop - (uintptr_t)&stackMarker;

    Serial.println("RAM:");
    Serial.print("  total SRAM:   ");
    Serial.println(RAM_SIZE);

    Serial.print("  static used:  ");
    Serial.println(staticUsed);

    Serial.print("  heap used:    ");
    Serial.println(heapUsed);

    Serial.print("  stack used:   ");
    Serial.println(stackUsed);

    Serial.print("  rough free:   ");
    Serial.println(RAM_SIZE - staticUsed - heapUsed - stackUsed);
}

static void printTriangleInfo(const glm::vec3 &t)
{
    Serial.println("Triangle info:");
    Serial.println(t.x);
    Serial.println(t.y);
    Serial.println(t.z);
}

struct Buffer
{
    char *buffPtr = nullptr;
    int buffSize = 0;
    int itemSize = 0;
};

struct AABB
{
    // bl = botom left
    float blX = 0;
    float blY = 0;
    float width = 0;
    float height = 0;
};

// buffer management

Buffer vertexBuffer;
Buffer indexBuffer;
Buffer colorBuffer;
Buffer depthBuffer;
AABB renderTargetDimensions;

// TODO: ideally, replace with vector. But that breaks everything.
constexpr int maxRasterizedFragments = (240 / 8) * (135 / 8);

struct RasterizedTriangle
{
    RasterizedTriangle() { fragments = new glm::vec2[maxRasterizedFragments]; }
    ~RasterizedTriangle() { delete[] fragments; }

    glm::vec2 *fragments = nullptr;
    int count = 0;
};

ArduGL::ReturnInfo ArduGL::clearBuffer(BufferType buffType, char clearValue)
{
    switch (buffType)
    {
    case BufferType::BT_VertexAttribute:
    case BufferType::BT_VertexIndex:
        return ReturnInfo{ false, EC_InvalidOperation };
    case BufferType::BT_Depth:
        std::memset(depthBuffer.buffPtr, clearValue, depthBuffer.buffSize);
        break;
    case BufferType::BT_Color:
        std::memset(colorBuffer.buffPtr, clearValue, colorBuffer.buffSize);
        break;
    default:
        return ReturnInfo{ false, EC_UnsupportedBufferType };
    }
    return ReturnInfo{ true, EC_OK };
}

ArduGL::ReturnInfo ArduGL::bindBuffer(BufferType buffType, char *buffPtr, int buffSize,
                                      int itemSize)
{
    switch (buffType)
    {
    case BufferType::BT_VertexAttribute:
        vertexBuffer = Buffer{ .buffPtr = buffPtr, .buffSize = buffSize, .itemSize = itemSize };
        break;
    case BufferType::BT_VertexIndex:
        indexBuffer = Buffer{ .buffPtr = buffPtr, .buffSize = buffSize, .itemSize = itemSize };
        break;
    case BufferType::BT_Depth:
        depthBuffer = Buffer{ .buffPtr = buffPtr, .buffSize = buffSize, .itemSize = itemSize };
        break;
    case BufferType::BT_Color:
        colorBuffer = Buffer{ .buffPtr = buffPtr, .buffSize = buffSize, .itemSize = itemSize };
        break;
    default:
        return ReturnInfo{ false, EC_UnsupportedBufferType };
    }
    return ReturnInfo{ true, EC_OK };
}

ArduGL::ReturnInfo ArduGL::unbindBuffer(BufferType buffType)
{
    switch (buffType)
    {
    case BufferType::BT_VertexAttribute:
        vertexBuffer.buffPtr = nullptr;
        break;
    case BufferType::BT_VertexIndex:
        indexBuffer.buffPtr = nullptr;
        break;
    case BufferType::BT_Depth:
        depthBuffer.buffPtr = nullptr;
        break;
    case BufferType::BT_Color:
        colorBuffer.buffPtr = nullptr;
        break;
    default:
        return ReturnInfo{ false, EC_UnsupportedBufferType };
    }
    return ReturnInfo{ true, EC_OK };
}

ArduGL::ReturnInfo ArduGL::setRenderTargetDimensions(int width, int height)
{
    renderTargetDimensions = AABB{ .blX = 0.0,
                                   .blY = 0.0,
                                   .width = static_cast<float>(width),
                                   .height = static_cast<float>(height) };
    return ReturnInfo{ .success = true, .code = ErrorCode::EC_OK };
}

// shader management

using VertexShaderOutput
    = std::pair<glm::vec4 /*view space vertex*/,
                std::vector<float> /*attributes to be passed down the pipeline*/>;
using VertexShader = VertexShaderOutput (*)(const char *vertex /*vertex data from buffer*/);
VertexShader vertexShaderPtr = nullptr;

using FragmentShader = glm::vec3 /*color*/ (*)(const std::vector<float> &interpolatedAttributes);
FragmentShader fragmentShaderPtr = nullptr;

ArduGL::ReturnInfo ArduGL::bindShader(ShaderType shType, void *shaderFuncPtr)
{
    switch (shType)
    {
    case ShaderType::ST_Vertex:
        vertexShaderPtr = reinterpret_cast<VertexShader>(shaderFuncPtr);
        break;
    case ShaderType::ST_Fragment:
        fragmentShaderPtr = reinterpret_cast<FragmentShader>(shaderFuncPtr);
        break;
    default:
        return ReturnInfo{ false, EC_UnsupportedShaderType };
    }
    return ReturnInfo{ true, EC_OK };
}

ArduGL::ReturnInfo ArduGL::unbindShader(ShaderType shType)
{
    switch (shType)
    {
    case ShaderType::ST_Vertex:
        vertexShaderPtr = nullptr;
        break;
    case ShaderType::ST_Fragment:
        fragmentShaderPtr = nullptr;
        break;
    default:
        return ReturnInfo{ false, EC_UnsupportedShaderType };
    }
    return ReturnInfo{ true, EC_OK };
}

// pipeline execution

void perspectiveDivide(glm::vec4 &clipPos)
{
    clipPos.x /= clipPos.w;
    clipPos.y /= clipPos.w;
    clipPos.z /= clipPos.w;
}

// shifts z to [0,1]
void rescaleZ(glm::vec4 &ndcPos)
{
    ndcPos.z += 1.0;
    ndcPos.z /= 2.0;
}

void mapToScreen(glm::vec4 &ndcPos)
{
    // shifts the viewport (not configurable)
    ndcPos.x += 1.0;
    ndcPos.x /= 2.0;

    ndcPos.y += 1.0;
    ndcPos.y /= 2.0;

    // stretches the vector according to render target dimensions.
    ndcPos.x *= static_cast<float>(renderTargetDimensions.width);
    ndcPos.y *= static_cast<float>(renderTargetDimensions.height);
}

// result interpretation: <bottom-left-x, bottom-left-y, width, height>
// origin: bottom left. Y is up. X is right.
AABB computeTriangleAABB(const glm::vec4 &v1, const glm::vec4 &v2, const glm::vec4 &v3)
{
    const float minX = glm::min(glm::min(v1.x, v2.x), v3.x);
    const float maxX = glm::max(glm::max(v1.x, v2.x), v3.x);

    const float minY = glm::min(glm::min(v1.y, v2.y), v3.y);
    const float maxY = glm::max(glm::max(v1.y, v2.y), v3.y);

    return AABB{ .blX = minX, .blY = minY, .width = maxX - minX, .height = maxY - minY };
}

bool checkAABBIntersect(const AABB &b1, const AABB &b2)
{
    const bool noIntersection = (b1.blX + b1.width) < b2.blX || (b2.blX + b2.width) < b1.blX
                                || (b1.blY + b1.height) < b2.blY || (b2.blY + b2.height) < b1.blY;
    return !noIntersection;
}

glm::vec3 computeTriCrossProduct(const glm::vec4 &v1, const glm::vec4 &v2, const glm::vec4 &v3)
{
    const glm::vec3 v1Pos{ v1.x, v1.y, 0 };
    const glm::vec3 v2Pos{ v2.x, v2.y, 0 };
    const glm::vec3 v3Pos{ v3.x, v3.y, 0 };

    return glm::cross(v2Pos - v1Pos, v3Pos - v1Pos);
}

void rasterizeTriangle(const AABB &triangleAABB, const glm::vec4 &v1, const glm::vec4 &v2,
                       const glm::vec4 &v3, RasterizedTriangle &coveredFragments)
{
    const glm::vec3 v1Pos{ v1.x, v1.y, 0 };
    const glm::vec3 v2Pos{ v2.x, v2.y, 0 };
    const glm::vec3 v3Pos{ v3.x, v3.y, 0 };
    coveredFragments.count = 0;

    const int xMin = glm::max(static_cast<int>(glm::ceil(triangleAABB.blX)), 0);
    const int xMax = glm::min(static_cast<int>(glm::ceil(triangleAABB.blX + triangleAABB.width)),
                              static_cast<int>(renderTargetDimensions.width));
    const int yMin = glm::max(static_cast<int>(glm::ceil(triangleAABB.blY)), 0);
    const int yMax = glm::min(static_cast<int>(glm::ceil(triangleAABB.blY + triangleAABB.height)),
                              static_cast<int>(renderTargetDimensions.height));

    if (xMin >= xMax || yMin >= yMax)
        return;

    for (int x = xMin; x < xMax; ++x)
    {
        for (int y = yMin; y < yMax; ++y)
        {
            const glm::vec3 fragmentCoordinates{ x, y, 0.0 };

            const bool pixelCenterIsCovered
                = glm::cross(fragmentCoordinates - v1Pos, v2Pos - v1Pos).z >= 0.0
                  && glm::cross(fragmentCoordinates - v2Pos, v3Pos - v2Pos).z >= 0.0
                  && glm::cross(fragmentCoordinates - v3Pos, v1Pos - v3Pos).z >= 0.0;
            if (pixelCenterIsCovered)
            {
                if (coveredFragments.count >= maxRasterizedFragments)
                {
                    assert(false); // TODO get rid of this and instead use dynamically sized buffers
                    return;
                }
                coveredFragments.fragments[coveredFragments.count++] = glm::vec2{ x, y };
            }
        }
    }
}

glm::vec3 computeBarycentricCoordinates(const glm::vec2 &point, const glm::vec4 &v1,
                                        const glm::vec4 &v2, const glm::vec4 &v3)
{
    const glm::vec3 v1Pos{ v1.x, v1.y, 0.0 };
    const glm::vec3 v2Pos{ v2.x, v2.y, 0.0 };
    const glm::vec3 v3Pos{ v3.x, v3.y, 0.0 };
    const glm::vec3 pointPos{ point.x, point.y, 0.0 };

    // TODO: add renormalization of coords so that they ad up to 1
    const float triangleArea = glm::length(glm::cross(v3Pos - v1Pos, v2Pos - v1Pos));
    const float area1 = glm::length(glm::cross(pointPos - v2Pos, pointPos - v3Pos));
    const float area2 = glm::length(glm::cross(pointPos - v1Pos, pointPos - v3Pos));
    const float area3 = glm::length(glm::cross(pointPos - v1Pos, pointPos - v2Pos));

    // actually, compares ratios of paralelogram areas
    return { area1 / triangleArea, area2 / triangleArea, area3 / triangleArea };
}

ArduGL::ReturnInfo ArduGL::renderPrimitives()
{
    RasterizedTriangle coveredFragments;

    for (int v = 0, totalTriangles = vertexBuffer.buffSize / (3 * vertexBuffer.itemSize);
         v < totalTriangles; ++v)
    {
        // Serial.print("- Processing triangle: ");
        // Serial.println(v);

        // primitive assembly + vertex shader
        const char *triangleAttributesStart = vertexBuffer.buffPtr + v * 3 * vertexBuffer.itemSize;

        VertexShaderOutput tv1 = vertexShaderPtr(triangleAttributesStart
                                                 + vertexBuffer.itemSize * 0);
        VertexShaderOutput tv2 = vertexShaderPtr(triangleAttributesStart
                                                 + vertexBuffer.itemSize * 1);
        VertexShaderOutput tv3 = vertexShaderPtr(triangleAttributesStart
                                                 + vertexBuffer.itemSize * 2);

        assert(static_cast<int>(renderTargetDimensions.width * renderTargetDimensions.height)
               == (colorBuffer.buffSize / colorBuffer.itemSize));

        perspectiveDivide(tv1.first);
        perspectiveDivide(tv2.first);
        perspectiveDivide(tv3.first);

        // shift z
        rescaleZ(tv1.first);
        rescaleZ(tv2.first);
        rescaleZ(tv3.first);

        // screen mapping
        mapToScreen(tv1.first);
        mapToScreen(tv2.first);
        mapToScreen(tv3.first);

        // face culling
        if (computeTriCrossProduct(tv1.first, tv2.first, tv3.first).z <= 0.0)
            continue;

        // Serial.println("--- Transformed triangle.");
        // printTriangleInfo(tv1.first);
        // printTriangleInfo(tv2.first);
        // printTriangleInfo(tv3.first);

        const AABB triangleAABB = computeTriangleAABB(tv1.first, tv2.first, tv3.first);

        // if the traingle is out of NDC -> skip it
        if (!checkAABBIntersect(renderTargetDimensions, triangleAABB))
            continue;

        // rasterization. No need to clip against NDC frustrum sincle I clip triangle AABB against
        // it here.
        rasterizeTriangle(triangleAABB, tv1.first, tv2.first, tv3.first, coveredFragments);

        // Serial.println("--- Rasterized triangle.");
        // Serial.print("--- Total fragments covered: ");
        // Serial.println(coveredFragments.count);

        for (int fragmentIndex = 0; fragmentIndex < coveredFragments.count; ++fragmentIndex)
        {
            const auto &fragment = coveredFragments.fragments[fragmentIndex];
            const int linearFragmentCoordinates = fragment.y * renderTargetDimensions.width
                                                  + fragment.x;
            // Serial.println("----- Processed fragment");
            // Serial.println(fragment.x);
            // Serial.println(fragment.y);

            const auto fragmentBarycentricCoordinates = computeBarycentricCoordinates(fragment,
                                                                                      tv1.first,
                                                                                      tv2.first,
                                                                                      tv3.first);

            const glm::vec3 oneOverWs = glm::vec3(1.0 / tv1.first.w, 1.0 / tv2.first.w,
                                                  1.0 / tv3.first.w);
            const float oneOverW = glm::dot(fragmentBarycentricCoordinates, oneOverWs);

            // Serial.println("----- Computed barycentrics");
            // Serial.println(fragmentBarycentricCoordinates.x);
            // Serial.println(fragmentBarycentricCoordinates.y);
            // Serial.println(fragmentBarycentricCoordinates.z);

            // depth processing
            {
                // TODO: formalize this convention
                float *depthBufPtr = reinterpret_cast<float *>(depthBuffer.buffPtr)
                                     + linearFragmentCoordinates;
                const auto storedDepth = *depthBufPtr;
                const auto currentNonLinearDepth = glm::dot(fragmentBarycentricCoordinates,
                                                            glm::vec3(tv1.first.z, tv2.first.z,
                                                                      tv3.first.z));

                // Serial.println("----- Computed depth");
                // Serial.println(currentNonLinearDepth);

                if (currentNonLinearDepth <= storedDepth)
                    continue;

                *depthBufPtr = currentNonLinearDepth;
                // Serial.println("------ Wrote depth");
            }

            // color processing
            {
                std::vector<float> interpolatedAttributes;
                const int numTotalAttributes = tv1.second.size();
                interpolatedAttributes.reserve(numTotalAttributes);
                for (int a = 0; a < numTotalAttributes; ++a)
                {
                    interpolatedAttributes.emplace_back(
                        glm::dot(fragmentBarycentricCoordinates,
                                 glm::vec3(tv1.second[a], tv2.second[a], tv3.second[a]) * oneOverWs)
                        / oneOverW);
                }
                // Serial.println("----- Interpolated attributes");

                const glm::vec3 fragmentOutput = fragmentShaderPtr(interpolatedAttributes);
                // Serial.println("----- Computed color");
                // printTriangleInfo(fragmentOutput);

                // TODO: formalize this convention and absence of alpha support
                glm::vec3 *colorBufPtr = reinterpret_cast<glm::vec3 *>(colorBuffer.buffPtr)
                                         + linearFragmentCoordinates;

                *colorBufPtr = fragmentOutput;
                // Serial.println("------ Wrote color");
            }
        }
    }
}

ArduGL::ReturnInfo ArduGL::renderIndexedPrimitives() {}
