#include "testpipeline.h"
#include "ardugl.h"
#include "glm.hpp"
#include <ext/matrix_clip_space.hpp>
#include <ext/matrix_transform.hpp>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Arduino.h>
#include <cstdint>
#include <vector>

#undef radians

namespace
{

constexpr int screenWidth = 240 / 4;
constexpr int screenHeight = 135 / 4;

constexpr int depthBufferSize = screenWidth * screenHeight * sizeof(float);
char *depthBuffer = new char[depthBufferSize];

constexpr int colorBufferSize = screenWidth * screenHeight * sizeof(uint16_t);
char *colorBuffer = new char[colorBufferSize];

struct Vertex
{
    glm::vec3 position;
    glm::vec3 color;
};

constexpr int vertexBufferSize = 6 * 6 * sizeof(Vertex);
Vertex *vertexBuffer = new Vertex[6 * 6]{
    // clang-format off
    Vertex{ {0.0, 0.0, 0.0}, {1.00, 0.18, 0.16} }, Vertex{ {0.0, 0.0, 2.0}, {1.00, 0.32, 0.12} }, Vertex{ {2.0, 0.0, 2.0}, {0.92, 0.24, 0.10} }, 
    Vertex{ {2.0, 0.0, 2.0}, {1.00, 0.55, 0.05} }, Vertex{ {2.0, 0.0, 0.0}, {0.95, 0.68, 0.10} }, Vertex{ {0.0, 0.0, 0.0}, {0.85, 0.48, 0.05} },
    Vertex{ {2.0, 0.0, 0.0}, {0.85, 0.90, 0.12} }, Vertex{ {2.0, 0.0, 2.0}, {0.70, 1.00, 0.16} }, Vertex{ {2.0, 2.0, 2.0}, {0.55, 0.82, 0.08} },
    Vertex{ {2.0, 2.0, 2.0}, {0.05, 0.85, 0.25} }, Vertex{ {2.0, 2.0, 0.0}, {0.12, 1.00, 0.42} }, Vertex{ {2.0, 0.0, 0.0}, {0.04, 0.65, 0.20} },
    Vertex{ {2.0, 2.0, 0.0}, {0.03, 0.78, 0.65} }, Vertex{ {2.0, 2.0, 2.0}, {0.08, 0.95, 0.80} }, Vertex{ {0.0, 2.0, 2.0}, {0.02, 0.58, 0.52} },
    Vertex{ {0.0, 2.0, 2.0}, {0.05, 0.65, 1.00} }, Vertex{ {0.0, 2.0, 0.0}, {0.12, 0.85, 1.00} }, Vertex{ {2.0, 2.0, 0.0}, {0.02, 0.48, 0.88} },
    Vertex{ {0.0, 2.0, 0.0}, {0.10, 0.25, 1.00} }, Vertex{ {0.0, 2.0, 2.0}, {0.25, 0.42, 1.00} }, Vertex{ {0.0, 0.0, 2.0}, {0.05, 0.18, 0.78} },
    Vertex{ {0.0, 0.0, 2.0}, {0.42, 0.18, 1.00} }, Vertex{ {0.0, 0.0, 0.0}, {0.58, 0.32, 1.00} }, Vertex{ {0.0, 2.0, 0.0}, {0.30, 0.12, 0.85} },
    Vertex{ {2.0, 0.0, 2.0}, {0.85, 0.12, 1.00} }, Vertex{ {0.0, 0.0, 2.0}, {1.00, 0.32, 0.90} }, Vertex{ {0.0, 2.0, 2.0}, {0.68, 0.06, 0.78} }, 
    Vertex{ {0.0, 2.0, 2.0}, {1.00, 0.14, 0.55} }, Vertex{ {2.0, 2.0, 2.0}, {1.00, 0.34, 0.68} }, Vertex{ {2.0, 0.0, 2.0}, {0.82, 0.08, 0.42} },
    Vertex{ {2.0, 0.0, 0.0}, {1.00, 0.72, 0.45} }, Vertex{ {0.0, 0.0, 0.0}, {0.90, 0.58, 0.34} }, Vertex{ {0.0, 2.0, 0.0}, {0.75, 0.42, 0.25} }, 
    Vertex{ {0.0, 2.0, 0.0}, {0.35, 1.00, 0.65} }, Vertex{ {2.0, 2.0, 0.0}, {0.52, 0.90, 0.80} }, Vertex{ {2.0, 0.0, 0.0}, {0.22, 0.72, 0.55} } // clang-format on
};

glm::mat4 proj = glm::perspective(glm::radians(45.0), (double)screenWidth / (double)screenHeight,
                                  0.1, 1000.0);
glm::mat4 view = glm::translate(glm::mat4(1.0), glm::vec3(0.0, 0.0, -5.0));

float runningParameter = 0.0;

glm::mat4 buildModelMatrix()
{
    return glm::translate(glm::identity<glm::mat4>(), glm::vec3(-1 + runningParameter))
           * glm::rotate(glm::identity<glm::mat4>(), runningParameter * 2.0f * glm::pi<float>(),
                         glm::vec3(0.0, 1.0, 1.0));
    //    * glm::scale(glm::identity<glm::mat4>(), glm::vec3(2.5));
    //    * glm::rotate(glm::identity<glm::mat4>(), glm::radians(runningParameter),
    //                  glm::vec3(0.0, 2.0, 0.0));
    //    * glm::scale(glm::identity<glm::mat4>(),
    //                 glm::vec3(3.0 * glm::sin(runningParameter * glm::pi<float>())));
}

glm::mat4 currentModelMatrix = buildModelMatrix();

Adafruit_ST7789 tft = Adafruit_ST7789(/*CS*/ 10, /*DC*/ 12, /*MOSI*/ 11, /*SCK*/ 13);

} // namespace

using VertexShaderOutput
    = std::pair<glm::vec4 /*view space vertex*/,
                std::vector<float> /*attributes to be passed down the pipeline*/>;
VertexShaderOutput cubeVertexShader(const char *rawVertex /*vertex data from buffer*/)
{
    const Vertex *vertex = reinterpret_cast<const Vertex *>(rawVertex);

    const glm::vec3 *vertexPos = &vertex->position;
    const glm::vec3 *vertexColor = &vertex->color;
    const glm::vec4 worldPos = currentModelMatrix
                               * glm::vec4(vertexPos->x, vertexPos->y, vertexPos->z, 1.0);
    const glm::vec4 transformedPos = proj * view * worldPos;

    return std::make_pair(transformedPos,
                          std::vector<float>{ vertexColor->x, vertexColor->y, vertexColor->z });
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
                       sizeof(uint16_t));
    ArduGL::bindBuffer(ArduGL::BufferType::BT_VertexAttribute,
                       reinterpret_cast<char *>(vertexBuffer), vertexBufferSize, sizeof(Vertex));

    ArduGL::bindShader(ArduGL::ShaderType::ST_Vertex, reinterpret_cast<void *>(&cubeVertexShader));
    ArduGL::bindShader(ArduGL::ShaderType::ST_Fragment,
                       reinterpret_cast<void *>(&cubeFragmentShader));

    Serial.begin(115200, SERIAL_8N1);

    tft.init(135, 240);
    tft.fillScreen(ST77XX_GREEN);
    delay(500);
    tft.fillScreen(ST77XX_BLUE);
    delay(500);
    tft.fillScreen(ST77XX_RED);
    delay(500);
    tft.fillScreen(ST77XX_BLACK);

    // a single pixel
    // tft.drawPixel(tft.width() / 2, tft.height() / 2, ST77XX_GREEN);
}

void drawCube()
{
    ArduGL::clearBuffer(ArduGL::BufferType::BT_Depth, 1.0);
    ArduGL::clearBuffer(ArduGL::BufferType::BT_Color, 0.2);

    currentModelMatrix = buildModelMatrix();
    runningParameter += 0.025;
    if (runningParameter > 1.0)
        runningParameter -= 1.0;

    ArduGL::renderPrimitives();

    Serial.write("FRAME_SEP");
    Serial.write(colorBuffer, colorBufferSize);
    Serial.write(depthBuffer, depthBufferSize);
    Serial.flush();


    //TODO: picture is jagged

    tft.startWrite();
    tft.setAddrWindow(tft.width() / 2, tft.height() / 2, screenWidth, screenHeight);
    tft.writePixels(reinterpret_cast<uint16_t *>(colorBuffer), screenWidth * screenWidth, true);
    tft.endWrite();

    // TODO: upscale buffers on their way out?
}
