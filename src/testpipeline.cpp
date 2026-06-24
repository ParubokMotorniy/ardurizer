#include "ardugl.h"
#include "glm.hpp"
#include "testpipeline.h"

namespace
{
const int screenWidth = 960;
const int screenHeight = 540;

char *depthBuffer = new char[screenWidth * screenHeight * sizeof(double)];
char *colorBuffer = new char[screenWidth * screenHeight * sizeof(glm::vec4)];

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

} // namespace

using VertexShaderOutput
    = std::pair<glm::vec4 /*view space vertex*/,
                std::vector<float> /*attributes to be passed down the pipeline*/>;
VertexShaderOutput cubeVertexShader(const char *vertex /*vertex data from buffer*/) {}

glm::vec4 cubeFragmentShader(const std::vector<float> &interpolatedAttributes) {}

void initializePipeline()
{
    ArduGL::setRenderTargetDimensions(screenWidth, screenHeight);

    ArduGL::bindBuffer(ArduGL::BufferType::BT_Depth, depthBuffer, sizeof(depthBuffer),
                       sizeof(double));
    ArduGL::bindBuffer(ArduGL::BufferType::BT_Color, colorBuffer, sizeof(colorBuffer),
                       sizeof(glm::vec4));
    ArduGL::bindBuffer(ArduGL::BufferType::BT_VertexAttribute,
                       reinterpret_cast<char *>(vertexPosBuffer), sizeof(vertexPosBuffer),
                       sizeof(glm::vec3));

    ArduGL::clearBuffer(ArduGL::BufferType::BT_Depth, 1.0);
    ArduGL::clearBuffer(ArduGL::BufferType::BT_Color, 0.3);

    ArduGL::bindShader(ArduGL::ShaderType::ST_Vertex, reinterpret_cast<void *>(&cubeVertexShader));
    ArduGL::bindShader(ArduGL::ShaderType::ST_Fragment,
                       reinterpret_cast<void *>(&cubeFragmentShader));
}

void drawCube()
{
    // TODO: create horribly simple vertex and fragment shaders to test the pipeline
}