#pragma once

#include <cstddef>
#include <cstdint>

// ---------------------------------------------------------------------------
// Compile-time configuration
// ---------------------------------------------------------------------------

// Tile dimensions in pixels. Must divide evenly into the render-target size.
// Color tile RAM cost  = ARDUGL_TILE_W * ARDUGL_TILE_H * 2  bytes (RGB565)
// Depth tile RAM cost  = ARDUGL_TILE_W * ARDUGL_TILE_H * 1  byte  (uint8)
// At 8×8 that is 128 + 64 = 192 bytes — very comfortable in 32 KB SRAM.
#ifndef ARDUGL_TILE_W
#define ARDUGL_TILE_W 8
#endif
#ifndef ARDUGL_TILE_H
#define ARDUGL_TILE_H 8
#endif

// Maximum number of triangles that can be binned per tile.
// Each entry is one uint16_t triangle index → 2 bytes each.
#ifndef ARDUGL_MAX_TRIS_PER_TILE
#define ARDUGL_MAX_TRIS_PER_TILE 64
#endif

// Maximum total triangles across all tiles (= vertex buffer capacity / 3).
#ifndef ARDUGL_MAX_TRIANGLES
#define ARDUGL_MAX_TRIANGLES 64
#endif

// Maximum number of tiles (tilesX * tilesY).
// At 60×33 with 8×8 tiles: ceil(60/8)*ceil(33/8) = 8*5 = 40 tiles.
#ifndef ARDUGL_MAX_TILES
#define ARDUGL_MAX_TILES 40
#endif

// ---------------------------------------------------------------------------
// Hardware SPI + DMA gate
// ---------------------------------------------------------------------------
// Set to 1 after rewiring DC from pin 12 to pin 9 and switching to the
// 2-argument Adafruit_ST7789 constructor (hardware SPI).
// When 0, the display write falls back to the existing soft-SPI path via
// the Adafruit writePixels() call supplied by the caller.
#ifndef ARDUGL_USE_HW_SPI_DMA
#define ARDUGL_USE_HW_SPI_DMA 0
#endif

// ---------------------------------------------------------------------------
// Flash ping-pong buffers
// ---------------------------------------------------------------------------
// Two full-resolution (240×135 RGB565 = 64 800 bytes each) color buffers
// live in code flash, reserved by the custom linker script (fsp.ld):
//
//   0x00020000 – 0x0002FFFF   64 KB   frame buffer 0  (.ardugl_fb0)
//   0x00030000 – 0x0003FFFF   64 KB   frame buffer 1  (.ardugl_fb1)
//
// They consume zero SRAM.  The tile rasterizer writes one 8×8 tile at a
// time into the in-RAM color tile (128 bytes), then commits it to flash
// via R_FLASH_LP_Erase + R_FLASH_LP_Write.
//
// Code flash programming is enabled by setting
//   FLASH_LP_CFG_CODE_FLASH_PROGRAMMING_ENABLE = 1
// in the framework config (already patched in
//   ~/.platformio/.../UNOWIFIR4/includes/ra_cfg/fsp_cfg/r_flash_lp_cfg.h).
//
// When DMA is enabled the inactive (committed) buffer is being streamed to
// the display while the CPU fills the active buffer tile-by-tile.
// ---------------------------------------------------------------------------

namespace ArduGL
{

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

enum BufferType
{
    BT_VertexAttribute,
    BT_VertexIndex,
    BT_Depth, ///< uint8_t per pixel, packed via packDepthIntoByte()
    BT_Color, ///< uint16_t RGB565 per pixel
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
    EC_NotReady,   ///< DMA transfer still in progress
    EC_FlashError, ///< FSP flash operation failed (reserved for future use)
};

struct ReturnInfo
{
    bool success{ true };
    ErrorCode code{ EC_OK };
};

// ---------------------------------------------------------------------------
// Flash frame buffer geometry
// The render target dimensions (set via setRenderTargetDimensions) determine
// how many pixels per row/column are written into each flash buffer.
// The full 240×135 layout is:
//   row stride  = 240 pixels × 2 bytes = 480 bytes
//   total frame = 240 × 135 × 2       = 64 800 bytes
// Both values must be multiples of the CF write size (8 bytes). ✓
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Buffer management
// ---------------------------------------------------------------------------

/// Set the color used to clear each tile at the start of renderTile().
/// Components are in [0.0, 1.0].  Default is dark grey (0.2, 0.2, 0.2).
void setClearColor(float r, float g, float b);

/// Clear the in-RAM color or depth tile (not the full-res ping-pong buffer).
ReturnInfo clearTile(BufferType buffType, float clearValue = 0.0f);

/// Legacy full-buffer clear — kept for backward compatibility with testpipeline.
ReturnInfo clearBuffer(BufferType buffType, float clearValue = 0.0f);

/// Bind a raw buffer (vertex attributes, vertex indices, legacy color/depth).
ReturnInfo bindBuffer(BufferType buffType, char *buffPtr, int buffSize, int itemSize);
ReturnInfo unbindBuffer(BufferType buffType);

/// Set the full render-target size in pixels (e.g. 60 × 33).
ReturnInfo setRenderTargetDimensions(int width, int height);

/// Initialise the flash frame buffer subsystem.
/// Must be called once before binTriangles() / renderTile() / commitTile().
/// renderW × renderH must match setRenderTargetDimensions().
/// renderW must be a multiple of 4 (write-size alignment: 4 × 2 bytes = 8).
ReturnInfo initFlashBuffers(int renderW, int renderH);

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
// Pipeline execution — legacy (renders directly into the bound BT_Color
// buffer, no tiling).  Kept so testpipeline.cpp continues to compile.
// ---------------------------------------------------------------------------
ReturnInfo renderPrimitives();
ReturnInfo renderIndexedPrimitives();

// ---------------------------------------------------------------------------
// Pipeline execution — tile-based
// ---------------------------------------------------------------------------

/// Step 1 – Run vertex shader on all triangles and bin them into tiles.
/// Call once per frame before any renderTile() calls.
ReturnInfo binTriangles();

/// Step 2 – Rasterize all triangles that overlap tile (tileCol, tileRow)
/// into the in-RAM color/depth tiles.  Clears the tiles first.
/// tileCol in [0, tilesX), tileRow in [0, tilesY).
ReturnInfo renderTile(int tileCol, int tileRow);

/// Step 3 – Copy the finished in-RAM color tile into the active ping-pong
/// buffer at the correct pixel offset.
ReturnInfo commitTile(int tileCol, int tileRow);

/// Step 4 – Schedule a DMA transfer of the active ping-pong buffer to the
/// display via hardware SPI, then flip to the other buffer.
/// When ARDUGL_USE_HW_SPI_DMA == 0 this is a no-op; the caller is
/// responsible for pushing pixels via Adafruit writePixels().
/// Returns EC_NotReady if a previous transfer is still in progress.
ReturnInfo scheduleDisplayTransfer(uint8_t csPin, uint8_t dcPin);

/// Returns true while a DMA transfer to the display is in progress.
bool isDisplayTransferBusy();

/// Returns a pointer to the last fully-committed (inactive) flash frame
/// buffer so the caller can push it via Adafruit writePixels() or DMA.
/// The pointer is a direct code-flash address (memory-mapped, read-only).
const uint16_t *getCommittedBuffer();

/// Number of tile columns and rows for the current render-target dimensions.
int getTilesX();
int getTilesY();

} // namespace ArduGL
