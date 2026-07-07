#include "ardugl.h"

#include "glm.hpp"

#include <cstring>
#include <vector>

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
    case ShaderType::ST_Fragment:
        fragmentShaderPtr = reinterpret_cast<FragmentShader>(shaderFuncPtr);
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
    ndcPos.y += 1.0;

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

std::vector<glm::vec2> rasterizeTriangle(const AABB &triangleAABB, const glm::vec4 &v1,
                                         const glm::vec4 &v2, const glm::vec4 &v3)
{
    const glm::vec3 v1Pos{ v1.x, v1.y, 0 };
    const glm::vec3 v2Pos{ v2.x, v2.y, 0 };
    const glm::vec3 v3Pos{ v3.x, v3.y, 0 };

    std::vector<glm::vec2> coveredFragments; // TODO: I believe this can be optimized

    for (int x = glm::ceil(triangleAABB.blX), xMax = triangleAABB.blX + triangleAABB.width;
         x < xMax; ++x)
    {
        for (int y = glm::ceil(triangleAABB.blY), yMax = triangleAABB.blY + triangleAABB.height;
             y < yMax; ++y)
        {
            const glm::vec3 fragmentCoordinates{ x, y, 0.0 };

            const bool pixelCenterIsCovered
                = glm::cross(fragmentCoordinates - v1Pos, v2Pos - v1Pos).z >= 0.0
                  && glm::cross(fragmentCoordinates - v2Pos, v3Pos - v2Pos).z >= 0.0
                  && glm::cross(fragmentCoordinates - v3Pos, v1Pos - v3Pos).z >= 0.0;
            if (pixelCenterIsCovered)
                coveredFragments.emplace_back(x, y);
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

    const float triangleArea = glm::cross(v3Pos - v1Pos, v2Pos - v1Pos).length();
    const float area1 = glm::cross(pointPos - v2Pos, pointPos - v3Pos).length();
    const float area2 = glm::cross(pointPos - v1Pos, pointPos - v3Pos).length();
    const float area3 = glm::cross(pointPos - v1Pos, pointPos - v2Pos).length();

    // actually, compares ratios of paralelogram areas
    return { area1 / triangleArea, area2 / triangleArea, area3 / triangleArea };
}

ArduGL::ReturnInfo ArduGL::renderPrimitives()
{
    for (int v = 0; v < vertexBuffer.buffSize; v += 3)
    {
        // primitive assembly + vertex shader
        const char *triangleAttributesStart = vertexBuffer.buffPtr + v * vertexBuffer.itemSize;

        VertexShaderOutput tv1 = vertexShaderPtr(triangleAttributesStart
                                                 + vertexBuffer.itemSize * 0);
        VertexShaderOutput tv2 = vertexShaderPtr(triangleAttributesStart
                                                 + vertexBuffer.itemSize * 1);
        VertexShaderOutput tv3 = vertexShaderPtr(triangleAttributesStart
                                                 + vertexBuffer.itemSize * 2);

        perspectiveDivide(tv1.first);
        perspectiveDivide(tv2.first);
        perspectiveDivide(tv3.first);

        // clipping - skipped for now
        // face culling - skipped for now

        assert(static_cast<int>(renderTargetDimensions.width * renderTargetDimensions.height)
               == (colorBuffer.buffSize / colorBuffer.itemSize));

        // shift z
        rescaleZ(tv1.first);
        rescaleZ(tv2.first);
        rescaleZ(tv3.first);

        // screen mapping
        mapToScreen(tv1.first);
        mapToScreen(tv2.first);
        mapToScreen(tv3.first);

        const AABB triangleAABB = computeTriangleAABB(tv1.first, tv2.first, tv3.first);

        // if the traingle is out of NDC -> skip it
        if (!checkAABBIntersect(renderTargetDimensions, triangleAABB))
            continue;

        // rasterization
        const auto coveredFragments = rasterizeTriangle(triangleAABB, tv1.first, tv2.first,
                                                        tv3.first);

        for (const auto &fragment : coveredFragments)
        {
            const int linearFragmentCoordinates = fragment.y * renderTargetDimensions.width
                                                  + fragment.x;
            const auto fragmentBarycentricCoordinates = computeBarycentricCoordinates(fragment,
                                                                                      tv1.first,
                                                                                      tv2.first,
                                                                                      tv3.first);

            const glm::vec3 oneOverWs = glm::vec3(1.0 / tv1.first.w, 1.0 / tv2.first.w,
                                                  1.0 / tv3.first.w);
            const float oneOverW = glm::dot(fragmentBarycentricCoordinates, oneOverWs);

            // depth processing
            {
                // TODO: formalize this convention
                float *depthBufPtr = reinterpret_cast<float *>(depthBuffer.buffPtr)
                                     + linearFragmentCoordinates;
                const auto storedDepth = *depthBufPtr;
                const auto currentNonLinearDepth = glm::dot(fragmentBarycentricCoordinates,
                                                            glm::vec3(tv1.first.z, tv2.first.z,
                                                                      tv3.first.z)
                                                                * oneOverWs);

                if (currentNonLinearDepth <= storedDepth)
                    continue;

                *depthBufPtr = currentNonLinearDepth;
            }

            // color processing
            {
                std::vector<float> interpolatedAttributes;
                interpolatedAttributes.reserve(tv1.second.size());
                for (int a = 0; a < interpolatedAttributes.size(); ++a)
                {
                    interpolatedAttributes.emplace_back(
                        glm::dot(fragmentBarycentricCoordinates,
                                 glm::vec3(tv1.second[a], tv2.second[a], tv3.second[a]) * oneOverWs)
                        / oneOverW);
                }

                const glm::vec3 fragmentOutput = fragmentShaderPtr(interpolatedAttributes);

                // TODO: formalize this convention and absence of alpha support
                glm::vec3 *colorBufPtr = reinterpret_cast<glm::vec3 *>(colorBuffer.buffPtr)
                                         + linearFragmentCoordinates;

                *colorBufPtr = fragmentOutput;
            }
        }
    }
}

ArduGL::ReturnInfo ArduGL::renderIndexedPrimitives() {}
