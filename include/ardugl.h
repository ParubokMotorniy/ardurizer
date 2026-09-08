#pragma once

#include <cstddef>
#include <cstdint>

#ifndef ARDUGL_USE_HW_SPI_ASYNC
#ifdef ARDUGL_USE_HW_SPI_DMA
#define ARDUGL_USE_HW_SPI_ASYNC ARDUGL_USE_HW_SPI_DMA
#else
#define ARDUGL_USE_HW_SPI_ASYNC 1
#endif
#endif

#if !ARDUGL_USE_HW_SPI_ASYNC
class Adafruit_ST7789;
#endif

// ---------------------------------------------------------------------------
// Compile-time configuration
// ---------------------------------------------------------------------------

// Tile dimensions in pixels.
// Two ping-pong color tiles live in SRAM:
//   2 × ARDUGL_TILE_W × ARDUGL_TILE_H × 2 bytes  (RGB565)
// One depth tile:
//       ARDUGL_TILE_W × ARDUGL_TILE_H × 1 byte   (uint8)
// The async SPI path also uses one 16-bit RGB565 staging tile.
// At 32×32: 2×2048 + 1024 + 2048 = 7168 bytes total — very comfortable in
// 32 KB SRAM.
#ifndef ARDUGL_TILE_W
#define ARDUGL_TILE_W 32
#endif
#ifndef ARDUGL_TILE_H
#define ARDUGL_TILE_H 32
#endif

// Maximum number of triangles that can be binned per tile.
// Each entry is one uint16_t triangle index → 2 bytes each.
// tileBins[ARDUGL_MAX_TILES][ARDUGL_MAX_TRIS_PER_TILE] × 2 bytes must fit in SRAM.
// At 135 tiles × 12 entries: 135 × 12 × 2 = 3 240 bytes.
#ifndef ARDUGL_MAX_TRIS_PER_TILE
#define ARDUGL_MAX_TRIS_PER_TILE 12
#endif

// Maximum total triangles per frame (= vertex buffer capacity / 3).
// The cube has 36 triangles; 48 gives a comfortable margin.
#ifndef ARDUGL_MAX_TRIANGLES
#define ARDUGL_MAX_TRIANGLES 48
#endif

// Maximum number of tiles (tilesX * tilesY).
// At 240×135 with 32×32 tiles: ceil(240/32)*ceil(135/32) = 8*5 = 40 tiles.
#ifndef ARDUGL_MAX_TILES
#define ARDUGL_MAX_TILES 135
#endif

// Maximum number of float attributes passed from vertex shader to fragment
// shader per vertex.  Must be >= the number of floats your vertex shader
// returns.  The cube shader returns 3 (RGB color).
// CachedTriangle size = 3×vec4 + 3×ARDUGL_MAX_ATTRS×float + 1 bool
//   = 48 + 3×8×4 + 4 = 148 bytes × 48 triangles = 7 104 bytes.
#ifndef ARDUGL_MAX_ATTRS
#define ARDUGL_MAX_ATTRS 8
#endif

// Async FSP SPI clock. The 16-bit pixel path uses one interrupt per pixel.
#ifndef ARDUGL_SPI_BITRATE
#define ARDUGL_SPI_BITRATE 12000000UL
#endif

// Optional USB diagnostics for validating FSP callback activity on hardware.
#ifndef ARDUGL_SPI_DEBUG
#define ARDUGL_SPI_DEBUG 0
#endif

// ---------------------------------------------------------------------------
// Hardware SPI + asynchronous transfer gate
// ---------------------------------------------------------------------------
// Set ARDUGL_USE_HW_SPI_ASYNC to 1 to use the Renesas FSP SPI
// interrupt-driven ST7789 path. The old ARDUGL_USE_HW_SPI_DMA macro is
// accepted as a compatibility alias. The async path uses the standard UNO R4
// SPI pins (MOSI 11, SCK 13) and the supplied CS/DC pins.
// When 0, scheduleDisplayTransfer() pushes the committed tile synchronously
// via the Adafruit_ST7789 pointer supplied by initTiledPipeline().
namespace ArduGL
{

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

enum BufferType
{
    BT_VertexAttribute,
};

enum ShaderType
{
    ST_Vertex,
    ST_Fragment,
};

enum ErrorCode
{
    EC_OK,
    EC_UnsupportedBufferType,
    EC_UnsupportedShaderType,
    EC_InvalidOperation,
    EC_NotReady, ///< Display transfer still in progress
};

struct ReturnInfo
{
    bool success{ true };
    ErrorCode code{ EC_OK };
};

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

/// Set the background color used to clear each tile at the start of renderTile().
/// Components are in [0.0, 1.0].  Default is dark grey (0.2, 0.2, 0.2).
void setClearColor(float r, float g, float b);

/// Bind the vertex buffer.  Must be called before binTriangles().
/// buffPtr  — pointer to packed vertex data
/// buffSize — total size in bytes
/// itemSize — size of one vertex in bytes
ReturnInfo bindVertexBuffer(char *buffPtr, int buffSize, int itemSize);

/// Set the render-target size in pixels (e.g. 240 × 135).
ReturnInfo setRenderTargetDimensions(int width, int height);

#if ARDUGL_USE_HW_SPI_ASYNC
/// Initialise the ST7789 and FSP interrupt-driven tiled pipeline.
void initTiledPipeline(uint8_t csPin, uint8_t dcPin);

/// Fill the raw-initialized display synchronously, for setup diagnostics.
void fillDisplay(uint16_t color);
#else
/// Initialise the tiled pipeline after tft.init().
/// tft must remain valid for the lifetime of the synchronous pipeline.
void initTiledPipeline(Adafruit_ST7789 *tft, uint8_t csPin, uint8_t dcPin);
#endif

// ---------------------------------------------------------------------------
// Shader management
// ---------------------------------------------------------------------------

// Vertex shader signature (cast through void*):
//   std::pair<glm::vec4, std::vector<float>>
//       vertexShader(const char *vertex);
// Fragment shader signature:
//   glm::vec3 fragmentShader(const std::vector<float> &interpolatedAttributes);

ReturnInfo bindShader(ShaderType shType, void *shaderFuncPtr);
ReturnInfo unbindShader(ShaderType shType);

// ---------------------------------------------------------------------------
// Pipeline
// ---------------------------------------------------------------------------

/// Render and display one complete frame.
/// Transforms all geometry, bins triangles into tiles, rasterizes each tile
/// and pushes it to the display.  On the async path each tile's display
/// transfer overlaps with rasterization of the next tile.
ReturnInfo drawFrame();

/// Returns true while an asynchronous transfer to the display is in progress.
bool isDisplayTransferBusy();

} // namespace ArduGL
