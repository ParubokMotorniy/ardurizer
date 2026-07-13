#include "testpipeline.h"
#include "ardugl.h"
#include "glm.hpp"
#include <ext/matrix_clip_space.hpp>
#include <ext/matrix_transform.hpp>

#include <Arduino.h>
#include <vector>

namespace
{
constexpr int screenWidth = 240 / 8;
constexpr int screenHeight = 135 / 8;

constexpr int depthBufferSize = screenWidth * screenHeight * sizeof(float);
char *depthBuffer = new char[depthBufferSize];

constexpr int colorBufferSize = screenWidth * screenHeight * sizeof(glm::vec3);
char *colorBuffer = new char[colorBufferSize];

constexpr int vertexBufferSize = 6 * 6 * sizeof(glm::vec3);
glm::vec3 *vertexPosBuffer = new glm::vec3[6 * 6]{
    // clang-format off
    { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 }, { 1.0, 0.0, 1.0 }, 
    { 1.0, 0.0, 1.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0 }, { 1.0, 0.0, 1.0 }, { 1.0, 1.0, 1.0 },
    { 1.0, 1.0, 1.0 }, { 1.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 },
    { 1.0, 1.0, 0.0 }, { 1.0, 1.0, 1.0 }, { 0.0, 1.0, 1.0 },
    { 0.0, 1.0, 1.0 }, { 0.0, 1.0, 0.0 }, { 1.0, 1.0, 0.0 },
    { 0.0, 1.0, 0.0 }, { 0.0, 1.0, 1.0 }, { 0.0, 0.0, 1.0 },
    { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 },
    { 1.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 1.0, 1.0 }, 
    { 0.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0 }, { 1.0, 0.0, 1.0 },
    { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, 
    { 0.0, 1.0, 0.0 }, { 1.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 } // clang-format on
};

glm::mat4 proj = glm::perspective(/*glm::*/ radians(45.0),
                                  (double)screenWidth / (double)screenHeight, 0.1, 100.0);
glm::mat4 view = glm::translate(glm::mat4(1.0), glm::vec3(0.0, 0.0, -3.0));

glm::mat4 model = glm::translate(glm::identity<glm::mat4>(), glm::vec3(-1.0, -1.0, -1.0))
                  * glm::rotate(glm::identity<glm::mat4>(), /*glm::*/ (float)radians(45.0),
                                glm::vec3(0.0, 1.0, 0.0))
                  * glm::scale(glm::identity<glm::mat4>(), glm::vec3(2.0, 2.0, 2.0));
} // namespace

using VertexShaderOutput
    = std::pair<glm::vec4 /*view space vertex*/,
                std::vector<float> /*attributes to be passed down the pipeline*/>;
VertexShaderOutput cubeVertexShader(const char *vertex /*vertex data from buffer*/)
{
    const glm::vec3 *vertexPos = reinterpret_cast<const glm::vec3 *>(vertex);
    // TODO: add model transform
    const glm::vec4 worldPos = model * glm::vec4(vertexPos->x, vertexPos->y, vertexPos->z, 1.0);
    const glm::vec4 transformedPos = proj * view * worldPos;
    return std::make_pair(transformedPos, std::vector<float>{ worldPos.x, worldPos.y, worldPos.z });
}

glm::vec3 cubeFragmentShader(const std::vector<float> &interpolatedAttributes)
{
    assert(interpolatedAttributes.size() == 3);
    return glm::normalize(glm::vec3{ interpolatedAttributes[0], interpolatedAttributes[1],
                                     interpolatedAttributes[2] });
}

void initializePipeline()
{
    ArduGL::setRenderTargetDimensions(screenWidth, screenHeight);

    ArduGL::bindBuffer(ArduGL::BufferType::BT_Depth, depthBuffer, depthBufferSize, sizeof(float));
    ArduGL::bindBuffer(ArduGL::BufferType::BT_Color, colorBuffer, colorBufferSize,
                       sizeof(glm::vec3));
    ArduGL::bindBuffer(ArduGL::BufferType::BT_VertexAttribute,
                       reinterpret_cast<char *>(vertexPosBuffer), vertexBufferSize,
                       sizeof(glm::vec3));

    ArduGL::bindShader(ArduGL::ShaderType::ST_Vertex, reinterpret_cast<void *>(&cubeVertexShader));
    ArduGL::bindShader(ArduGL::ShaderType::ST_Fragment,
                       reinterpret_cast<void *>(&cubeFragmentShader));

    Serial.begin(115200, SERIAL_8N1);
}

void drawCube()
{
    ArduGL::clearBuffer(ArduGL::BufferType::BT_Depth, 1.0);
    ArduGL::clearBuffer(ArduGL::BufferType::BT_Color, 0.3);

    ArduGL::renderPrimitives();

    // Serial.println(colorBufferSize + depthBufferSize);
    // Serial.println("Color buffer: ");
    Serial.write(colorBuffer, colorBufferSize);
    // Serial.println();
    // Serial.println("Depth buffer: ");
    Serial.write(depthBuffer, depthBufferSize);
    // Serial.println();
    // Serial.println("Over");
    Serial.flush();
}