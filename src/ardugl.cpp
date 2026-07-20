#include "ardugl.h"

#include "glm.hpp"

#include <Arduino.h>

#include <cstdint>
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

    //Serial.println("RAM:");
    //Serial.print("  total SRAM:   ");
    //Serial.println(RAM_SIZE);

    //Serial.print("  static used:  ");
    //Serial.println(staticUsed);

    //Serial.print("  heap used:    ");
    //Serial.println(heapUsed);

    //Serial.print("  stack used:   ");
    //Serial.println(stackUsed);

    //Serial.print("  rough free:   ");
    //Serial.println(RAM_SIZE - staticUsed - heapUsed - stackUsed);
}

static void printTriangleInfo(const glm::vec3 &t)
{
    //Serial.println("Triangle info:");
    //Serial.println(t.x);
    //Serial.println(t.y);
    //Serial.println(t.z);
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

uint16_t quantizeChannel(float value, uint16_t maxValue)
{
    if (!(value >= 0.0f))
        return 0;
    if (value > 1.0f)
        value = 1.0f;

    return static_cast<uint16_t>(value * static_cast<float>(maxValue) + 0.5f);
}

uint16_t packRGB565(const glm::vec3 &color)
{
    const uint16_t red = quantizeChannel(color.r, 31);
    const uint16_t green = quantizeChannel(color.g, 63);
    const uint16_t blue = quantizeChannel(color.b, 31);

    return static_cast<uint16_t>((red << 11) | (green << 5) | blue);
}

ArduGL::ReturnInfo ArduGL::clearBuffer(BufferType buffType, float clearValue)
{
    switch (buffType)
    {
    case BufferType::BT_VertexAttribute:
    case BufferType::BT_VertexIndex:
        return ReturnInfo{ false, EC_InvalidOperation };
    case BufferType::BT_Depth:
    {
        float *depthValues = reinterpret_cast<float *>(depthBuffer.buffPtr);
        const int depthValueCount = depthBuffer.buffSize / depthBuffer.itemSize;
        for (int i = 0; i < depthValueCount; ++i)
            depthValues[i] = clearValue;
        break;
    }
    case BufferType::BT_Color:
    {
        uint16_t *colorValues = reinterpret_cast<uint16_t *>(colorBuffer.buffPtr);
        const int colorValueCount = colorBuffer.buffSize / colorBuffer.itemSize;
        const uint16_t packedColor = packRGB565(glm::vec3(clearValue));
        for (int i = 0; i < colorValueCount; ++i)
            colorValues[i] = packedColor;
        break;
    }
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

void mapToScreen(glm::vec4 &ndcPos)
{
    // shifts z to [0,1]
    ndcPos.z += 1.0;
    ndcPos.z /= 2.0;

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
                       const glm::vec4 &v3, std::vector<glm::vec2> &coveredFragments)
{
    const glm::vec3 v1Pos{ v1.x, v1.y, 0 };
    const glm::vec3 v2Pos{ v2.x, v2.y, 0 };
    const glm::vec3 v3Pos{ v3.x, v3.y, 0 };
    coveredFragments.clear();

    //TODO: implement top-left rule rigorously
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
                coveredFragments.emplace_back(x, y);
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
    std::vector<glm::vec2> coveredFragments;

    for (int v = 0, totalTriangles = vertexBuffer.buffSize / (3 * vertexBuffer.itemSize);
         v < totalTriangles; ++v)
    {
        //Serial.print("- Processing triangle: ");
        //Serial.println(v);

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

        //TODO: Cohen–Sutherland?
        // screen mapping + z shift
        mapToScreen(tv1.first);
        mapToScreen(tv2.first);
        mapToScreen(tv3.first);

        // face culling
        if (computeTriCrossProduct(tv1.first, tv2.first, tv3.first).z >= 0.0)
        {
            //Serial.println("Face-culled triangle!");
            continue;
        }

        //Serial.println("--- Transformed triangle.");
        printTriangleInfo(tv1.first);
        printTriangleInfo(tv2.first);
        printTriangleInfo(tv3.first);

        const AABB triangleAABB = computeTriangleAABB(tv1.first, tv2.first, tv3.first);

        // if the triangle is out of screen -> skip it
        if (!checkAABBIntersect(renderTargetDimensions, triangleAABB))
        {
            //Serial.println("Triangle is out of frustrum!");
            continue;
        }

        // rasterization. No need to clip against NDC frustrum sincle I clip triangle AABB against
        // it here.
        rasterizeTriangle(triangleAABB, tv1.first, tv2.first, tv3.first, coveredFragments);

        //Serial.println("--- Rasterized triangle.");
        //Serial.print("--- Total fragments covered: ");
        //Serial.println(coveredFragments.size());

        for (const glm::vec2 &fragment : coveredFragments)
        {
            const int linearFragmentCoordinates = fragment.y * renderTargetDimensions.width
                                                  + fragment.x;
            //Serial.println("----- Processed fragment");
            //Serial.println(fragment.x);
            //Serial.println(fragment.y);

            const auto fragmentBarycentricCoordinates = computeBarycentricCoordinates(fragment,
                                                                                      tv1.first,
                                                                                      tv2.first,
                                                                                      tv3.first);

            const glm::vec3 oneOverWs = glm::vec3(1.0 / tv1.first.w, 1.0 / tv2.first.w,
                                                  1.0 / tv3.first.w);
            const float oneOverW = glm::dot(fragmentBarycentricCoordinates, oneOverWs);

            //Serial.println("----- Computed barycentrics");
            //Serial.println(fragmentBarycentricCoordinates.x);
            //Serial.println(fragmentBarycentricCoordinates.y);
            //Serial.println(fragmentBarycentricCoordinates.z);

            // depth processing
            {
                // TODO: formalize this convention
                float *depthBufPtr = reinterpret_cast<float *>(depthBuffer.buffPtr)
                                     + linearFragmentCoordinates;
                const auto storedDepth = *depthBufPtr;
                const auto currentNonLinearDepth = glm::dot(fragmentBarycentricCoordinates,
                                                            glm::vec3(tv1.first.z, tv2.first.z,
                                                                      tv3.first.z));

                //Serial.println("----- Computed depth");
                //Serial.println(currentNonLinearDepth);

                if (currentNonLinearDepth >= storedDepth)
                    continue;

                *depthBufPtr = currentNonLinearDepth;
                //Serial.println("------ Wrote depth");
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
                //Serial.println("----- Interpolated attributes");

                const glm::vec3 fragmentOutput = fragmentShaderPtr(interpolatedAttributes);
                //Serial.println("----- Computed color");
                printTriangleInfo(fragmentOutput);

                uint16_t *colorBufPtr = reinterpret_cast<uint16_t *>(colorBuffer.buffPtr)
                                        + linearFragmentCoordinates;

                *colorBufPtr = packRGB565(fragmentOutput);
                //Serial.println("------ Wrote color");
            }
        }
    }
}

ArduGL::ReturnInfo ArduGL::renderIndexedPrimitives() {}
