// =============================================================================
// ardugl.cpp  —  ArduGL software rasterizer, tile-based edition
//
// Architecture overview
// ─────────────────────
// • Two tiny ping-pong COLOR tiles live in SRAM (each TILE_W×TILE_H×2 bytes).
//   One single DEPTH tile is shared (TILE_W×TILE_H×1 byte).
//   At 16×16: 2×512 + 256 = 1280 bytes total.
//
// • Triangle binning: all triangles are transformed once per frame by
//   binTriangles().  Each triangle is recorded in every tile whose AABB
//   overlaps the triangle's screen-space AABB.
//
// • Per-tile loop (caller side):
//     binTriangles();
//     renderTile(0, 0);                          // prime
//     for each subsequent tile (tx, ty):
//         scheduleDisplayTransfer(prev_tx, prev_ty); // push committed, flip
//         renderTile(tx, ty);                        // rasterize into active
//     scheduleDisplayTransfer(last_tx, last_ty);     // push final tile
//
// • Soft-SPI path (ARDUGL_USE_HW_SPI_DMA == 0, default):
//   scheduleDisplayTransfer() calls tft.setAddrWindow() + tft.writePixels()
//   synchronously for the committed tile, then flips active/committed.
//
// • DMA path (ARDUGL_USE_HW_SPI_DMA == 1, requires DC rewired off pin 12):
//   scheduleDisplayTransfer() blocks until the previous tile's DMA finishes,
//   then arms DMAC channel 0 for the committed tile asynchronously and flips.
//   The CPU overlaps the next renderTile() with the ongoing DMA transfer.
// =============================================================================

#include "ardugl.h"

#include "glm.hpp"

#include <Adafruit_ST7789.h>
#include <Arduino.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

#if ARDUGL_USE_HW_SPI_DMA
// r_dmac.h pulls in bsp_api.h → renesas.h → R7FA4M1AB.h transitively.
#include "r_dmac.h"
// bsp_elc.h is not on the standard include path — use the path relative to
// the framework variant prefix (matched by -iwithprefixbefore in includes.txt).
#include "../../src/bsp/mcu/ra4m1/bsp_elc.h"
#endif

#undef abs
#undef radians

// Bring ArduGL types into file scope so internal static functions can use
// ReturnInfo, EC_OK etc. without qualifying every occurrence.
using namespace ArduGL;

// =============================================================================
// Data structures
// =============================================================================

struct Buffer
{
    char *buffPtr = nullptr;
    int buffSize = 0;
    int itemSize = 0;
};

struct AABB
{
    // origin: bottom-left, Y-up (OpenGL convention)
    float blX = 0;
    float blY = 0;
    float width = 0;
    float height = 0;
};

// Transformed triangle cached for the duration of one frame (binTriangles).
// attrs[][]: fixed-size flat array — avoids heap allocation per triangle.
// numAttrs: actual number of attributes returned by the vertex shader.
struct CachedTriangle
{
    glm::vec4 sv[3];                       // screen-space positions
    float attrs[3][ARDUGL_MAX_ATTRS] = {}; // per-vertex attributes
    int numAttrs = 0;
    bool valid = false;
};

// =============================================================================
// Module-level state
// =============================================================================

static Buffer vertexBuffer;

static AABB renderTargetDimensions; // kept for mapToScreen(); mirrors renderW/renderH

// --- In-RAM ping-pong color tiles + single depth tile ---
// pingPongTile[0] and [1] alternate: one is being rasterized into while
// the other is being (or has just been) sent to the display.
// depthTile is shared — it is cleared at the start of every renderTile().
static uint16_t pingPongTile[2][ARDUGL_TILE_W * ARDUGL_TILE_H];
static uint8_t depthTile[ARDUGL_TILE_W * ARDUGL_TILE_H];

// Clear color applied at the start of every renderTile() call.
// Stored pre-packed as RGB565 to avoid re-packing per tile.
static uint16_t clearColorPacked = 0x18C6; // packRGB565({0.2, 0.2, 0.2})

// Ping-pong indices.
// activeTileIdx    — the tile currently being rasterized into.
// committedTileIdx — the tile last finished, ready for display.
static int activeTileIdx = 0;
static int committedTileIdx = 1;

// Render-target dimensions (set by setRenderTargetDimensions).
static int renderW = 0;
static int renderH = 0;

// --- Triangle bin ---
// For each tile: a list of triangle indices (into cachedTriangles[]) that
// overlap that tile.  Stored as a flat 2-D array.
static uint16_t tileBins[ARDUGL_MAX_TILES][ARDUGL_MAX_TRIS_PER_TILE];
static uint8_t tileBinCount[ARDUGL_MAX_TILES]; // number of entries per tile

// Cached per-frame triangle data produced by binTriangles().
static CachedTriangle cachedTriangles[ARDUGL_MAX_TRIANGLES];

// --- Display handle (soft-SPI path) ---
// Set by initTiledPipeline(); used by scheduleDisplayTransfer().
static Adafruit_ST7789 *s_tft = nullptr;

// --- DMA state ---
static volatile bool dmaTransferBusy = false;

#if ARDUGL_USE_HW_SPI_DMA
static dmac_instance_ctrl_t dmacCtrl;
static transfer_info_t dmacInfo;
static dmac_extended_cfg_t dmacExtCfg;
static transfer_cfg_t dmacCfg;
static uint8_t dmaCsPin = 10;
static uint8_t dmaDcPin = 9;

extern "C" void ardugl_spi_tei_isr();
#endif

// =============================================================================
// Packing helpers
// =============================================================================

static uint16_t quantizeChannel(float value, uint16_t maxValue)
{
    if (!(value >= 0.0f))
        return 0;
    if (value > 1.0f)
        value = 1.0f;
    return static_cast<uint16_t>(value * static_cast<float>(maxValue) + 0.5f);
}

static uint16_t packRGB565(const glm::vec3 &color)
{
    const uint16_t r = quantizeChannel(color.r, 31);
    const uint16_t g = quantizeChannel(color.g, 63);
    const uint16_t b = quantizeChannel(color.b, 31);
    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

static uint8_t packDepthIntoByte(float depth)
{
    if (depth < 0.0f)
        depth = 0.0f;
    if (depth > 1.0f)
        depth = 1.0f;
    return static_cast<uint8_t>(depth * 255.0f);
}

// =============================================================================
// Buffer / pipeline setup
// =============================================================================

void ArduGL::setClearColor(float r, float g, float b)
{
    clearColorPacked = packRGB565(glm::vec3(r, g, b));
}

ArduGL::ReturnInfo ArduGL::bindVertexBuffer(char *buffPtr, int buffSize, int itemSize)
{
    vertexBuffer = Buffer{ .buffPtr = buffPtr, .buffSize = buffSize, .itemSize = itemSize };
    return ReturnInfo{ true, EC_OK };
}

ArduGL::ReturnInfo ArduGL::setRenderTargetDimensions(int width, int height)
{
    renderTargetDimensions = AABB{ .blX = 0.0f,
                                   .blY = 0.0f,
                                   .width = static_cast<float>(width),
                                   .height = static_cast<float>(height) };
    renderW = width;
    renderH = height;
    return ReturnInfo{ true, EC_OK };
}

void ArduGL::initTiledPipeline(Adafruit_ST7789 *tft, uint8_t csPin, uint8_t dcPin)
{
    s_tft = tft;
    activeTileIdx = 0;
    committedTileIdx = 1;

#if ARDUGL_USE_HW_SPI_DMA
    dmaCsPin = csPin;
    dmaDcPin = dcPin;

    // Install TEI ISR once — fires when the last SPI byte leaves the shift
    // register, deasserts CS and clears dmaTransferBusy.
    constexpr IRQn_Type kTeiIrq = static_cast<IRQn_Type>(8);
    R_ICU->IELSR[kTeiIrq] = ELC_EVENT_SPI0_TEI;
    NVIC_SetVector(kTeiIrq, reinterpret_cast<uint32_t>(ardugl_spi_tei_isr));
    NVIC_SetPriority(kTeiIrq, 12);
    NVIC_EnableIRQ(kTeiIrq);

    // Open DMAC channel 0 once — only dmacInfo.p_src / .length change per tile.
    dmacInfo.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
    dmacInfo.transfer_settings_word_b.size = TRANSFER_SIZE_2_BYTE;
    dmacInfo.transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
    dmacInfo.transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
    dmacInfo.transfer_settings_word_b.irq = TRANSFER_IRQ_END;
    dmacInfo.p_dest = reinterpret_cast<void *>(
        const_cast<uint16_t *>(reinterpret_cast<volatile uint16_t *>(&R_SPI0->SPDR)));
    dmacInfo.p_src = nullptr; // set per tile
    dmacInfo.length = 0;      // set per tile
    dmacInfo.num_blocks = 0;

    dmacExtCfg.channel = 0;
    dmacExtCfg.irq = FSP_INVALID_VECTOR;
    dmacExtCfg.ipl = 0;
    dmacExtCfg.activation_source = ELC_EVENT_SPI0_TXI;
    dmacExtCfg.p_callback = nullptr;
    dmacExtCfg.p_context = nullptr;

    dmacCfg.p_info = &dmacInfo;
    dmacCfg.p_extend = &dmacExtCfg;

    R_DMAC_Open(&dmacCtrl, &dmacCfg);
#else
    (void)csPin;
    (void)dcPin;
#endif
}

static int getTilesX() { return (renderW + ARDUGL_TILE_W - 1) / ARDUGL_TILE_W; }

static int getTilesY() { return (renderH + ARDUGL_TILE_H - 1) / ARDUGL_TILE_H; }

// =============================================================================
// Shader management
// =============================================================================

using VertexShaderOutput
    = std::pair<glm::vec4 /*clip-space vertex*/,
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

// =============================================================================
// Pipeline geometry helpers
// =============================================================================

static void perspectiveDivide(glm::vec4 &clipPos)
{
    clipPos.x /= clipPos.w;
    clipPos.y /= clipPos.w;
    clipPos.z /= clipPos.w;
}

// NDC → screen space.  Y=0 is bottom (OpenGL convention).
static void mapToScreen(glm::vec4 &ndcPos)
{
    ndcPos.z = (ndcPos.z + 1.0f) * 0.5f;
    ndcPos.x = (ndcPos.x + 1.0f) * 0.5f * renderTargetDimensions.width;
    ndcPos.y = (ndcPos.y + 1.0f) * 0.5f * renderTargetDimensions.height;
}

// AABB in screen space (Y-up, origin bottom-left).
static AABB computeTriangleAABB(const glm::vec4 &v1, const glm::vec4 &v2, const glm::vec4 &v3)
{
    const float minX = glm::min(glm::min(v1.x, v2.x), v3.x);
    const float maxX = glm::max(glm::max(v1.x, v2.x), v3.x);
    const float minY = glm::min(glm::min(v1.y, v2.y), v3.y);
    const float maxY = glm::max(glm::max(v1.y, v2.y), v3.y);
    return AABB{ .blX = minX, .blY = minY, .width = maxX - minX, .height = maxY - minY };
}

static bool checkAABBIntersect(const AABB &b1, const AABB &b2)
{
    return !((b1.blX + b1.width) < b2.blX || (b2.blX + b2.width) < b1.blX
             || (b1.blY + b1.height) < b2.blY || (b2.blY + b2.height) < b1.blY);
}

static glm::vec3 computeTriCrossProduct(const glm::vec4 &v1, const glm::vec4 &v2,
                                        const glm::vec4 &v3)
{
    return glm::cross(glm::vec3(v2.x - v1.x, v2.y - v1.y, 0.0f),
                      glm::vec3(v3.x - v1.x, v3.y - v1.y, 0.0f));
}

static glm::vec3 computeBarycentricCoordinates(const glm::vec2 &point, const glm::vec4 &v1,
                                               const glm::vec4 &v2, const glm::vec4 &v3)
{
    const glm::vec3 p{ point.x, point.y, 0.0f };
    const glm::vec3 a{ v1.x, v1.y, 0.0f };
    const glm::vec3 b{ v2.x, v2.y, 0.0f };
    const glm::vec3 c{ v3.x, v3.y, 0.0f };

    const float totalArea = glm::length(glm::cross(c - a, b - a));
    if (totalArea < 1e-10f)
        return { 1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f };

    const float w1 = glm::length(glm::cross(p - b, p - c)) / totalArea;
    const float w2 = glm::length(glm::cross(p - a, p - c)) / totalArea;
    const float w3 = glm::length(glm::cross(p - a, p - b)) / totalArea;
    return { w1, w2, w3 };
}

// Shade a single fragment and write into the provided color/depth buffers.
// bufWidth is the stride (pixels per row) of those buffers.
// fragX, fragY are in screen space (Y-up, absolute render-target coordinates).
static void shadeFragment(int fragX, int fragY, int bufOffsetX, int bufOffsetY, int bufWidth,
                          const glm::vec4 &sv1, const glm::vec4 &sv2, const glm::vec4 &sv3,
                          const float attrs1[], const float attrs2[], const float attrs3[],
                          int numAttrs, uint16_t *colorBuf, uint8_t *depthBuf)
{
    const glm::vec2 fc{ static_cast<float>(fragX) + 0.5f, static_cast<float>(fragY) + 0.5f };
    const glm::vec3 bary = computeBarycentricCoordinates(fc, sv1, sv2, sv3);

    const glm::vec3 oneOverWs{ 1.0f / sv1.w, 1.0f / sv2.w, 1.0f / sv3.w };
    const float oneOverW = glm::dot(bary, oneOverWs);

    // Depth test
    const int localIdx = (fragY - bufOffsetY) * bufWidth + (fragX - bufOffsetX);
    const float depth = glm::dot(bary, glm::vec3(sv1.z, sv2.z, sv3.z));
    const float storedDepth = depthBuf[localIdx] / 255.0f;
    if (depth >= storedDepth)
        return;
    depthBuf[localIdx] = packDepthIntoByte(depth);

    // Perspective-correct attribute interpolation into a stack-allocated buffer.
    // No heap allocation — fixed size matches ARDUGL_MAX_ATTRS.
    float interp[ARDUGL_MAX_ATTRS];
    for (int a = 0; a < numAttrs; ++a)
    {
        interp[a] = glm::dot(bary, glm::vec3(attrs1[a], attrs2[a], attrs3[a]) * oneOverWs)
                    / oneOverW;
    }

    // Pass as std::vector to keep the fragment shader signature unchanged.
    // This is a single small heap allocation per shaded fragment; acceptable
    // for now — can be eliminated later by changing the shader signature.
    const std::vector<float> interpVec(interp, interp + numAttrs);
    colorBuf[localIdx] = packRGB565(fragmentShaderPtr(interpVec));
}

// =============================================================================
// Tiled pipeline — binTriangles
// =============================================================================

static ArduGL::ReturnInfo binTriangles()
{
    assert(vertexShaderPtr && "Vertex shader not bound");
    assert(vertexBuffer.buffPtr && "Vertex buffer not bound");

    const int totalTriangles = vertexBuffer.buffSize / (3 * vertexBuffer.itemSize);
    assert(totalTriangles <= ARDUGL_MAX_TRIANGLES && "Increase ARDUGL_MAX_TRIANGLES");

    const int tilesX = getTilesX();
    const int tilesY = getTilesY();
    const int totalTiles = tilesX * tilesY;
    assert(totalTiles <= ARDUGL_MAX_TILES && "Increase ARDUGL_MAX_TILES");

    memset(tileBinCount, 0, sizeof(uint8_t) * totalTiles);

    for (int t = 0; t < totalTriangles; ++t)
    {
        const char *base = vertexBuffer.buffPtr + t * 3 * vertexBuffer.itemSize;

        VertexShaderOutput tv1 = vertexShaderPtr(base + vertexBuffer.itemSize * 0);
        VertexShaderOutput tv2 = vertexShaderPtr(base + vertexBuffer.itemSize * 1);
        VertexShaderOutput tv3 = vertexShaderPtr(base + vertexBuffer.itemSize * 2);

        perspectiveDivide(tv1.first);
        perspectiveDivide(tv2.first);
        perspectiveDivide(tv3.first);

        mapToScreen(tv1.first);
        mapToScreen(tv2.first);
        mapToScreen(tv3.first);

        // Back-face cull
        if (computeTriCrossProduct(tv1.first, tv2.first, tv3.first).z >= 0.0f)
        {
            cachedTriangles[t].valid = false;
            continue;
        }

        const AABB triAABB = computeTriangleAABB(tv1.first, tv2.first, tv3.first);

        // Frustum cull
        if (!checkAABBIntersect(renderTargetDimensions, triAABB))
        {
            cachedTriangles[t].valid = false;
            continue;
        }

        // Cache the transformed triangle
        const int nAttrs = static_cast<int>(tv1.second.size());
        assert(nAttrs <= ARDUGL_MAX_ATTRS && "Increase ARDUGL_MAX_ATTRS");
        cachedTriangles[t].valid = true;
        cachedTriangles[t].sv[0] = tv1.first;
        cachedTriangles[t].sv[1] = tv2.first;
        cachedTriangles[t].sv[2] = tv3.first;
        cachedTriangles[t].numAttrs = nAttrs;
        for (int a = 0; a < nAttrs; ++a)
        {
            cachedTriangles[t].attrs[0][a] = tv1.second[a];
            cachedTriangles[t].attrs[1][a] = tv2.second[a];
            cachedTriangles[t].attrs[2][a] = tv3.second[a];
        }
        // Bin into overlapping tiles
        for (int ty = 0; ty < tilesY; ++ty)
        {
            for (int tx = 0; tx < tilesX; ++tx)
            {
                // Tile AABB in screen space (Y-up)
                const float tileBlX = static_cast<float>(tx * ARDUGL_TILE_W);
                const float tileBlY = static_cast<float>(ty * ARDUGL_TILE_H);
                const AABB tileAABB{ .blX = tileBlX,
                                     .blY = tileBlY,
                                     .width = static_cast<float>(ARDUGL_TILE_W),
                                     .height = static_cast<float>(ARDUGL_TILE_H) };

                if (!checkAABBIntersect(triAABB, tileAABB))
                    continue;

                const int tileIdx = ty * tilesX + tx;
                if (tileBinCount[tileIdx] < ARDUGL_MAX_TRIS_PER_TILE)
                {
                    tileBins[tileIdx][tileBinCount[tileIdx]++] = static_cast<uint16_t>(t);
                }
            }
        }
    }

    return ReturnInfo{ true, EC_OK };
}

// =============================================================================
// Tiled pipeline — renderTile
// =============================================================================

static ArduGL::ReturnInfo renderTile(int tileCol, int tileRow)
{
    assert(fragmentShaderPtr && "Fragment shader not bound");

    // Clear the active ping-pong color tile and the shared depth tile
    for (int i = 0; i < ARDUGL_TILE_W * ARDUGL_TILE_H; ++i)
        pingPongTile[activeTileIdx][i] = clearColorPacked;
    memset(depthTile, 0xFF, sizeof(depthTile)); // 0xFF = max depth (far)

    const int tilesX = getTilesX();
    const int tileIdx = tileRow * tilesX + tileCol;

    // Tile origin in screen space (Y-up)
    const int tileOriginX = tileCol * ARDUGL_TILE_W;
    const int tileOriginY = tileRow * ARDUGL_TILE_H;

    // Clamp tile to render target bounds
    const int tileEndX = glm::min(tileOriginX + ARDUGL_TILE_W,
                                  static_cast<int>(renderTargetDimensions.width));
    const int tileEndY = glm::min(tileOriginY + ARDUGL_TILE_H,
                                  static_cast<int>(renderTargetDimensions.height));

    for (int b = 0; b < tileBinCount[tileIdx]; ++b)
    {
        const int t = tileBins[tileIdx][b];
        const CachedTriangle &tri = cachedTriangles[t];
        if (!tri.valid)
            continue;

        const AABB triAABB = computeTriangleAABB(tri.sv[0], tri.sv[1], tri.sv[2]);

        const glm::vec3 v1Pos{ tri.sv[0].x, tri.sv[0].y, 0.0f };
        const glm::vec3 v2Pos{ tri.sv[1].x, tri.sv[1].y, 0.0f };
        const glm::vec3 v3Pos{ tri.sv[2].x, tri.sv[2].y, 0.0f };

        const int xMin = glm::max(static_cast<int>(glm::ceil(triAABB.blX)), tileOriginX);
        const int xMax = glm::min(static_cast<int>(glm::ceil(triAABB.blX + triAABB.width)),
                                  tileEndX);
        const int yMin = glm::max(static_cast<int>(glm::ceil(triAABB.blY)), tileOriginY);
        const int yMax = glm::min(static_cast<int>(glm::ceil(triAABB.blY + triAABB.height)),
                                  tileEndY);

        if (xMin >= xMax || yMin >= yMax)
            continue;

        constexpr float kEps = 1.0e-6f;

        for (int x = xMin; x < xMax; ++x)
        {
            for (int y = yMin; y < yMax; ++y)
            {
                const glm::vec3 fp{ x + 0.5f, y + 0.5f, 0.0f };
                const float ce1 = glm::cross(fp - v1Pos, v2Pos - v1Pos).z;
                const float ce2 = glm::cross(fp - v2Pos, v3Pos - v2Pos).z;
                const float ce3 = glm::cross(fp - v3Pos, v1Pos - v3Pos).z;

                if (ce1 < -kEps || ce2 < -kEps || ce3 < -kEps)
                    continue;

                // Top-left rule for on-edge pixels
                if (ce1 <= kEps || ce2 <= kEps || ce3 <= kEps)
                {
                    auto isLeftOrTop = [&](const glm::vec3 &a, const glm::vec3 &b,
                                           const glm::vec3 &o) -> bool {
                        const bool top = glm::abs(a.y - b.y) < kEps && a.y > o.y && b.y > o.y;
                        const bool left = !top
                                          && ((a.x < b.x && a.x < o.x) || (b.x < a.x && b.x < o.x));
                        return top || left;
                    };
                    if (ce1 <= kEps && !isLeftOrTop(v1Pos, v2Pos, v3Pos))
                        continue;
                    if (ce2 <= kEps && !isLeftOrTop(v2Pos, v3Pos, v1Pos))
                        continue;
                    if (ce3 <= kEps && !isLeftOrTop(v3Pos, v1Pos, v2Pos))
                        continue;
                }

                shadeFragment(x, y, tileOriginX, tileOriginY, ARDUGL_TILE_W, tri.sv[0], tri.sv[1],
                              tri.sv[2], tri.attrs[0], tri.attrs[1], tri.attrs[2], tri.numAttrs,
                              pingPongTile[activeTileIdx], depthTile);
            }
        }
    }

    return ReturnInfo{ true, EC_OK };
}

// =============================================================================
// Tiled pipeline — scheduleDisplayTransfer
//
// Pushes the committed (last finished) tile to the display, then flips
// the ping-pong indices so the next renderTile() writes into the other tile.
//
// tileCol / tileRow identify the screen window for the committed tile.
// They refer to the tile that was rendered in the PREVIOUS renderTile() call,
// i.e. the caller's loop looks like:
//
//   renderTile(0, 0);                         // fill tile 0 into active
//   for each subsequent tile (tx, ty):
//       scheduleDisplayTransfer(prev_tx, prev_ty);  // push committed, flip
//       renderTile(tx, ty);                         // fill next tile
//   scheduleDisplayTransfer(last_tx, last_ty);      // push final tile
//
// With DMA enabled the push is asynchronous: the CPU starts renderTile() for
// the next tile while the DMAC streams the committed tile over SPI.
// =============================================================================

bool ArduGL::isDisplayTransferBusy() { return dmaTransferBusy; }

#if ARDUGL_USE_HW_SPI_DMA

// SPI0 TEI ISR — fires when the last byte has left the shift register.
// Deasserts CS and clears the busy flag.
extern "C" void ardugl_spi_tei_isr()
{
    R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)dmaCsPin, BSP_IO_LEVEL_HIGH);
    R_SPI0->SPCR &= ~(1u << 3); // clear SPE
    dmaTransferBusy = false;
}

// ---------------------------------------------------------------------------
// SPI helpers for the DMA path
// ---------------------------------------------------------------------------

// Wait for the SPI shift register to drain.
static inline void spiWaitTxEmpty()
{
    while (!(R_SPI0->SPSR & (1u << 7)))
    {
    } // SPSR.IDLNF: wait until idle
}

// Send one byte synchronously over hardware SPI (DC already set by caller).
static inline void spiWriteByte(uint8_t b)
{
    R_SPI0->SPCR |= (1u << 3); // SPE on
    R_SPI0->SPDR = b;
    spiWaitTxEmpty();
    R_SPI0->SPCR &= ~(1u << 3); // SPE off
}

// Send one 16-bit word synchronously (used for CASET/RASET coordinate pairs).
static inline void spiWriteWord(uint16_t w)
{
    spiWriteByte(static_cast<uint8_t>(w >> 8));
    spiWriteByte(static_cast<uint8_t>(w & 0xFF));
}

// Send a ST7789 command byte (DC low) then switch DC high for data.
static inline void spiCommand(uint8_t cmd)
{
    R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)dmaDcPin, BSP_IO_LEVEL_LOW);
    spiWriteByte(cmd);
    R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)dmaDcPin, BSP_IO_LEVEL_HIGH);
}

// Set the ST7789 address window synchronously.
// x0,y0 — top-left corner in display coordinates (Y-down).
// w, h   — width and height in pixels.
static void spiSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h)
{
    // CASET — column address
    spiCommand(0x2A);
    spiWriteWord(x0);
    spiWriteWord(static_cast<uint16_t>(x0 + w - 1));
    // RASET — row address
    spiCommand(0x2B);
    spiWriteWord(y0);
    spiWriteWord(static_cast<uint16_t>(y0 + h - 1));
    // RAMWR — begin pixel data stream
    spiCommand(0x2C);
    // DC is now high (data); leave CS asserted for the DMA burst.
}

// ---------------------------------------------------------------------------

static ArduGL::ReturnInfo scheduleDisplayTransfer(int tileCol, int tileRow)
{
    // Block until the previous tile's DMA transfer completes.
    while (dmaTransferBusy)
    {
    }

    // Flip ping-pong: active (just rasterized) becomes committed (to display).
    committedTileIdx = activeTileIdx;
    activeTileIdx = 1 - activeTileIdx;

    const int tileOriginX = tileCol * ARDUGL_TILE_W;
    const int tileOriginY = tileRow * ARDUGL_TILE_H;

    // Clamp to render-target bounds (edge tiles may be smaller).
    const int tileW = glm::min(ARDUGL_TILE_W, renderW - tileOriginX);
    const int tileH = glm::min(ARDUGL_TILE_H, renderH - tileOriginY);

    // Y-flip: rasterizer row 0 = bottom; display row 0 = top.
    const int displayY = renderH - tileOriginY - tileH;

    // Assert CS, send CASET/RASET/RAMWR synchronously, leave DC high for data.
    R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)dmaCsPin, BSP_IO_LEVEL_LOW);
    spiSetAddrWindow(static_cast<uint16_t>(tileOriginX), static_cast<uint16_t>(displayY),
                     static_cast<uint16_t>(tileW), static_cast<uint16_t>(tileH));

    // The committed tile rows are stored Y-up (row 0 = bottom of tile).
    // DMA streams them in memory order, so we point it at the last row and
    // send rows in reverse — each row is a separate DMA burst of tileW pixels.
    // For full tiles (tileH == ARDUGL_TILE_H) this is always 16 bursts of 16.
    for (int row = tileH - 1; row >= 0; --row)
    {
        const uint16_t *src = pingPongTile[committedTileIdx] + row * ARDUGL_TILE_W;

        // Reconfigure source pointer and length for this row.
        dmacInfo.p_src = src;
        dmacInfo.length = static_cast<uint16_t>(tileW);
        R_DMAC_Reconfigure(&dmacCtrl, &dmacInfo);
        R_DMAC_Enable(&dmacCtrl);

        // Trigger first transfer by enabling SPI TX interrupt.
        dmaTransferBusy = true;
        R_SPI0->SPCR |= (1u << 3) | (1u << 7); // SPE | SPTIE

        // Wait for this row's DMA to finish before starting the next row.
        // (True overlap only happens across tiles, not within a tile.)
        while (dmaTransferBusy)
        {
        }
    }

    // CS is deasserted by the ISR after the last row; we are already done
    // spinning above, so dmaTransferBusy is false and CS is high here.
    return ReturnInfo{ true, EC_OK };
}

#else // ARDUGL_USE_HW_SPI_DMA == 0

static ArduGL::ReturnInfo scheduleDisplayTransfer(int tileCol, int tileRow)
{
    s_tft->SPI_CS_HIGH();

    assert(s_tft && "initTiledPipeline() not called");

    // Flip ping-pong: active (just rasterized) becomes committed (to display).
    committedTileIdx = activeTileIdx;
    activeTileIdx = 1 - activeTileIdx;

    const int tileOriginX = tileCol * ARDUGL_TILE_W;
    const int tileOriginY = tileRow * ARDUGL_TILE_H;

    // Clamp tile dimensions to render-target bounds.
    const int tileW = glm::min(ARDUGL_TILE_W, renderW - tileOriginX);
    const int tileH = glm::min(ARDUGL_TILE_H, renderH - tileOriginY);

    // Y-flip: rasterizer row 0 = bottom; display row 0 = top.
    // The tile occupies rasterizer rows [tileOriginY, tileOriginY+tileH).
    // In display coordinates that maps to [(renderH - tileOriginY - tileH),
    //                                      (renderH - tileOriginY - 1)].
    const int displayY = renderH - tileOriginY - tileH;

    // The committed tile's rows are stored Y-up (row 0 = bottom of tile).
    // We need to send them Y-down, so we write rows in reverse order.
    s_tft->startWrite();
    s_tft->setAddrWindow(tileOriginX, displayY, tileW, tileH);
    const uint16_t *tile = pingPongTile[committedTileIdx];
    for (int row = tileH - 1; row >= 0; --row)
    {
        s_tft->writePixels(const_cast<uint16_t *>(tile + row * ARDUGL_TILE_W), tileW, true);
    }
    s_tft->endWrite();

    s_tft->SPI_CS_LOW();

    return ReturnInfo{ true, EC_OK };
}

#endif // ARDUGL_USE_HW_SPI_DMA

// =============================================================================
// Public entry point — drawFrame
// =============================================================================

ArduGL::ReturnInfo ArduGL::drawFrame()
{
    ReturnInfo r = binTriangles();
    if (!r.success)
        return r;

    const int tilesX = getTilesX();
    const int tilesY = getTilesY();

    // Prime the pipeline: rasterize the first tile before entering the loop
    // so there is always a committed tile ready to push on every iteration.
    renderTile(0, 0);

    for (int ty = 0; ty < tilesY; ++ty)
    {
        for (int tx = 0; tx < tilesX; ++tx)
        {
            if (tx == 0 && ty == 0)
                continue;

            // Push the previously rasterized (committed) tile to the display.
            // On the DMA path this returns immediately and the transfer runs
            // in the background while the CPU rasterizes the next tile.
            const int prevTx = (tx > 0) ? tx - 1 : tilesX - 1;
            const int prevTy = (tx > 0) ? ty : ty - 1;
            r = scheduleDisplayTransfer(prevTx, prevTy);
            if (!r.success)
                return r;

            renderTile(tx, ty);
        }
    }

    // Push the final tile.
    return scheduleDisplayTransfer(tilesX - 1, tilesY - 1);
}
