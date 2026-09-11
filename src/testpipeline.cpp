#include "testpipeline.h"
#include "ardugl.h"
#include "bunny_vertex_buffer.h"
#include "glm.hpp"
#include <ext/matrix_clip_space.hpp>
#include <ext/matrix_transform.hpp>

#include <Arduino.h>
#include <cstdint>
#include <vector>

#if !ARDUGL_USE_HW_SPI_ASYNC
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#endif

#undef radians

namespace
{

constexpr int fullScreenWidth = 7 * 32;
constexpr int fullScreenHeight = 4 * 32;

glm::mat4 proj = glm::perspective(glm::radians(45.0),
                                  (double)fullScreenWidth / (double)fullScreenHeight, 0.1, 1000.0);
glm::mat4 view = glm::translate(glm::mat4(1.0), glm::vec3(0.0, 0.0, -5.0));

float runningParameter = 0.0;

glm::mat4 buildModelMatrix()
{
    return glm::translate(glm::identity<glm::mat4>(), glm::vec3(0.0, -2.0, 0.0))
           * glm::rotate(glm::identity<glm::mat4>(), runningParameter * 2.0f * glm::pi<float>(),
                         glm::vec3(0.0, 1.0, 0.0))
           * glm::scale(glm::identity<glm::mat4>(), glm::vec3(1.5 * runningParameter));
    //    * glm::rotate(glm::identity<glm::mat4>(), glm::radians(runningParameter),
    //                  glm::vec3(0.0, 2.0, 0.0));
    //    * glm::scale(glm::identity<glm::mat4>(),
    //                 glm::vec3(3.0 * glm::sin(runningParameter * glm::pi<float>())));
}

glm::mat4 currentModelMatrix = buildModelMatrix();

#if !ARDUGL_USE_HW_SPI_ASYNC
Adafruit_ST7789 tft = Adafruit_ST7789(/*CS*/ 10, /*DC*/ 9, 11, 13);
#endif

} // namespace

using VertexShaderOutput
    = std::pair<glm::vec4 /*view space vertex*/,
                std::vector<float> /*attributes to be passed down the pipeline*/>;
VertexShaderOutput cubeVertexShader(const char *rawVertex /*vertex data from buffer*/)
{
    const Vertex *vertex = reinterpret_cast<const Vertex *>(rawVertex);

    const glm::vec3 *vertexPos = &vertex->position;
    const glm::vec3 *vertexColor = &vertex->color;
    const glm::vec3 *vertexNormal = &vertex->normal;
    const glm::vec4 worldPos = currentModelMatrix
                               * glm::vec4(vertexPos->x, vertexPos->y, vertexPos->z, 1.0);
    const glm::vec4 transformedPos = proj * view * worldPos;

    // TODO: properly transforn normals
    return std::make_pair(transformedPos,
                          std::vector<float>{ vertexColor->x, vertexColor->y, vertexColor->z,
                                              vertexNormal->x, vertexNormal->y, vertexNormal->z });
}

glm::vec3 cubeFragmentShader(const std::vector<float> &interpolatedAttributes)
{
    // TODO: add more complex shading, e.g. basic blinn-phong
    assert(interpolatedAttributes.size() == 6);
    return glm::vec3{ interpolatedAttributes[0], interpolatedAttributes[1],
                      interpolatedAttributes[2] };
}

void initializePipeline()
{
    ArduGL::bindVertexBuffer(reinterpret_cast<const char *>(vertexBuffer), vertexBufferSize,
                             sizeof(Vertex));
    ArduGL::bindIndexBuffer(reinterpret_cast<const char *>(indexBuffer), indexBufferSize,
                            sizeof(uint16_t));
    ArduGL::bindShader(ArduGL::ShaderType::ST_Vertex, reinterpret_cast<void *>(&cubeVertexShader));
    ArduGL::bindShader(ArduGL::ShaderType::ST_Fragment,
                       reinterpret_cast<void *>(&cubeFragmentShader));

    ArduGL::setRenderTargetDimensions(fullScreenWidth, fullScreenHeight);
    ArduGL::setClearColor(0.2f, 0.7f, 0.65f);

#if ARDUGL_USE_HW_SPI_ASYNC
    ArduGL::initTiledPipeline(/*csPin=*/10, /*dcPin=*/9);
    ArduGL::fillDisplay(0x07E0); // green
    delay(500);
    ArduGL::fillDisplay(0x001F); // blue
    delay(500);
    ArduGL::fillDisplay(0xF800); // red
    delay(500);
    ArduGL::fillDisplay(0x0000); // black
#else
    tft.init(135, 240);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_GREEN);
    delay(500);
    tft.fillScreen(ST77XX_BLUE);
    delay(500);
    tft.fillScreen(ST77XX_RED);
    delay(500);
    tft.fillScreen(ST77XX_BLACK);

    ArduGL::initTiledPipeline(&tft, /*csPin=*/10, /*dcPin=*/9);
#endif
}

void drawCube()
{
    currentModelMatrix = buildModelMatrix();
    runningParameter += 0.025f;
    if (runningParameter > 1.0f)
        runningParameter -= 1.0f;

    ArduGL::drawFrame();
}
