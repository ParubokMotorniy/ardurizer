// =============================================================================
// ardugl.cpp  —  ArduGL software rasterizer, tile-based edition
//
// Architecture overview
// ─────────────────────
// • Two full-resolution ping-pong color buffers live in SRAM.
//   (Code-flash programming is disabled in the Arduino RA4M1 framework config;
//    data flash is only 8 KB — too small for a 240×135×2 = 64 800-byte frame.)
//
// • Two small in-RAM tiles (color + depth) are the only working surfaces the
//   rasterizer writes into.  After each tile is finished it is committed
//   (memcpy'd) into the active ping-pong buffer at the correct offset.
//
// • Triangle binning: all triangles are transformed once per frame by
//   binTriangles().  Each triangle is recorded in every tile whose AABB
//   overlaps the triangle's screen-space AABB.
//
// • DMA path (ARDUGL_USE_HW_SPI_DMA == 1, requires DC rewired off pin 12):
//   scheduleDisplayTransfer() arms DMAC channel 0 to stream the committed
//   buffer to R_SPI0->SPDR, triggered by ELC_EVENT_SPI0_TXI.  The SPI0_TEI
//   ISR deasserts CS and clears the busy flag.
//
// • Soft-SPI fallback (ARDUGL_USE_HW_SPI_DMA == 0, current default):
//   scheduleDisplayTransfer() is a no-op.  The caller uses
//   getCommittedBuffer() + Adafruit writePixels() as before.
// =============================================================================

#include "ardugl.h"

#include "glm.hpp"

#include <Arduino.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <malloc.h>
#include <vector>

// FSP flash LP driver — code-flash programming enabled in r_flash_lp_cfg.h
#include "r_flash_lp.h"
// Note: hal_data.h declares g_flash0_ctrl / g_flash0_cfg as extern but never
// defines them (no hal_data.c in the Arduino framework).  We define our own
// flash instance here in blocking (non-BGO) mode — no IRQ needed.
static flash_lp_instance_ctrl_t s_flash_ctrl;
static const flash_cfg_t s_flash_cfg =
{
    .data_flash_bgo      = false,   // blocking mode — no background operation
    .p_callback          = nullptr,
    .p_extend            = nullptr,
    .p_context           = nullptr,
    .ipl                 = (BSP_IRQ_DISABLED),
    .irq                 = FSP_INVALID_VECTOR,
    .err_ipl             = (BSP_IRQ_DISABLED),
    .err_irq             = FSP_INVALID_VECTOR,
};

#if ARDUGL_USE_HW_SPI_DMA
#include "R7FA4M1AB.h"
#include "r_dmac.h"
#include "bsp_elc.h"
#include "vector_data.h"
#endif

// undef Arduino helpers - got my own (well, glm's)
#undef abs
#undef radians

// =============================================================================
// Internal helpers
// =============================================================================

static void printTriangleInfo(const glm::vec3 &) {}

extern char __HeapBase;
extern char __StackTop;

// =============================================================================
// Data structures
// =============================================================================

struct Buffer
{
    char *buffPtr = nullptr;
    int   buffSize = 0;
    int   itemSize = 0;
};

struct AABB
{
    // origin: bottom-left, Y-up (OpenGL convention)
    float blX    = 0;
    float blY    = 0;
    float width  = 0;
    float height = 0;
};

// Transformed triangle cached for the duration of one frame (binTriangles).
struct CachedTriangle
{
    glm::vec4 sv[3];                    // screen-space positions after mapToScreen()
    std::vector<float> attrs[3];        // per-vertex shader attributes
    bool valid = false;                 // false = culled / off-screen
};

// =============================================================================
// Module-level state
// =============================================================================

// --- Legacy full-buffer bindings (used by renderPrimitives / testpipeline) ---
Buffer vertexBuffer;
Buffer indexBuffer;
Buffer colorBuffer;   // legacy: points to caller-managed full-res buffer
Buffer depthBuffer;   // legacy: points to caller-managed full-res buffer

AABB renderTargetDimensions;

// --- In-RAM tiles (allocated once, reused every tile) ---
// Color tile: ARDUGL_TILE_W * ARDUGL_TILE_H * sizeof(uint16_t)
// Depth tile: ARDUGL_TILE_W * ARDUGL_TILE_H * sizeof(uint8_t)
static uint16_t colorTile[ARDUGL_TILE_W * ARDUGL_TILE_H];
static uint8_t  depthTile[ARDUGL_TILE_W * ARDUGL_TILE_H];

// Clear color applied at the start of every renderTile() call.
// Stored pre-packed as RGB565 to avoid re-packing per tile.
static uint16_t clearColorPacked = 0x18C6; // packRGB565({0.2, 0.2, 0.2})

// --- Flash ping-pong frame buffers ---
// Linker symbols defined in fsp.ld — these are the physical flash addresses.
extern uint16_t __ardugl_fb0_start[];
extern uint16_t __ardugl_fb1_start[];

// Pointers to the two flash frame buffers (set by initFlashBuffers).
static uint16_t *flashFrameBuf[2] = { nullptr, nullptr };
static int       flashFrameRenderW  = 0;  // pixels per row
static int       flashFrameRenderH  = 0;  // rows per frame
static size_t    flashFrameSize     = 0;  // bytes per frame
static int       activeBufIdx       = 0;  // being rasterized into
static int       committedBufIdx    = 1;  // last fully committed (display-ready)

// CF erase block size = 2 KB (from BSP_FEATURE_FLASH_LP_CF_BLOCK_SIZE)
constexpr uint32_t CF_BLOCK_SIZE  = 0x800U;
// CF write granularity = 8 bytes (from BSP_FEATURE_FLASH_LP_CF_WRITE_SIZE)
constexpr uint32_t CF_WRITE_SIZE  = 8U;

// Per-frame erase tracker: one bit per 2 KB CF block.
// 256 KB / 2 KB = 128 blocks → 128 bits = 16 bytes.
// Reset at the start of each frame by binTriangles().
static uint8_t eraseBlockDone[16] = {};

static inline bool isBlockErased(uint32_t blockIdx)
{
    return (eraseBlockDone[blockIdx >> 3] >> (blockIdx & 7)) & 1u;
}
static inline void markBlockErased(uint32_t blockIdx)
{
    eraseBlockDone[blockIdx >> 3] |= static_cast<uint8_t>(1u << (blockIdx & 7));
}

// --- Triangle bin ---
// For each tile: a list of triangle indices (into cachedTriangles[]) that
// overlap that tile.  Stored as a flat 2-D array.
static uint16_t  tileBins[ARDUGL_MAX_TILES][ARDUGL_MAX_TRIS_PER_TILE];
static uint8_t   tileBinCount[ARDUGL_MAX_TILES]; // number of entries per tile

// Cached per-frame triangle data produced by binTriangles().
static CachedTriangle cachedTriangles[ARDUGL_MAX_TRIANGLES];
static int            cachedTriangleCount = 0;

// --- DMA state ---
static volatile bool dmaTransferBusy = false;

#if ARDUGL_USE_HW_SPI_DMA
static dmac_instance_ctrl_t  dmacCtrl;
static transfer_info_t       dmacInfo;
static dmac_extended_cfg_t   dmacExtCfg;
static transfer_cfg_t        dmacCfg;
static uint8_t               dmaCsPin  = 10;
static uint8_t               dmaDcPin  = 9;

extern "C" void ardugl_spi_tei_isr();
#endif

// =============================================================================
// Packing helpers
// =============================================================================

static uint16_t quantizeChannel(float value, uint16_t maxValue)
{
    if (!(value >= 0.0f)) return 0;
    if (value > 1.0f)     value = 1.0f;
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
    if (depth < 0.0f) depth = 0.0f;
    if (depth > 1.0f) depth = 1.0f;
    return static_cast<uint8_t>(depth * 255.0f);
}

// =============================================================================
// Buffer management
// =============================================================================

// --- Tile clear ---

ArduGL::ReturnInfo ArduGL::clearTile(BufferType buffType, float clearValue)
{
    switch (buffType)
    {
    case BufferType::BT_Depth:
    {
        const uint8_t v = packDepthIntoByte(clearValue);
        memset(depthTile, v, sizeof(depthTile));
        break;
    }
    case BufferType::BT_Color:
    {
        const uint16_t v = packRGB565(glm::vec3(clearValue));
        for (int i = 0; i < ARDUGL_TILE_W * ARDUGL_TILE_H; ++i)
            colorTile[i] = v;
        break;
    }
    default:
        return ReturnInfo{ false, EC_InvalidOperation };
    }
    return ReturnInfo{ true, EC_OK };
}

void ArduGL::setClearColor(float r, float g, float b)
{
    clearColorPacked = packRGB565(glm::vec3(r, g, b));
}

// --- Legacy full-buffer clear (used by testpipeline / renderPrimitives) ---

ArduGL::ReturnInfo ArduGL::clearBuffer(BufferType buffType, float clearValue)
{
    switch (buffType)
    {
    case BufferType::BT_VertexAttribute:
    case BufferType::BT_VertexIndex:
        return ReturnInfo{ false, EC_InvalidOperation };
    case BufferType::BT_Depth:
    {
        if (!depthBuffer.buffPtr) return ReturnInfo{ false, EC_InvalidOperation };
        const int   count = depthBuffer.buffSize / depthBuffer.itemSize;
        const uint8_t v   = packDepthIntoByte(clearValue);
        memset(depthBuffer.buffPtr, v, count * sizeof(uint8_t));
        break;
    }
    case BufferType::BT_Color:
    {
        if (!colorBuffer.buffPtr) return ReturnInfo{ false, EC_InvalidOperation };
        uint16_t      *p     = reinterpret_cast<uint16_t *>(colorBuffer.buffPtr);
        const int      count = colorBuffer.buffSize / colorBuffer.itemSize;
        const uint16_t v     = packRGB565(glm::vec3(clearValue));
        for (int i = 0; i < count; ++i) p[i] = v;
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
    renderTargetDimensions = AABB{ .blX    = 0.0f,
                                   .blY    = 0.0f,
                                   .width  = static_cast<float>(width),
                                   .height = static_cast<float>(height) };
    return ReturnInfo{ true, EC_OK };
}

ArduGL::ReturnInfo ArduGL::initFlashBuffers(int renderW, int renderH)
{
    assert(renderW > 0 && renderH > 0);
    // Write size alignment: each row must be a multiple of CF_WRITE_SIZE bytes.
    // CF_WRITE_SIZE = 8 bytes = 4 pixels (uint16_t). Enforce at API boundary.
    assert((renderW % 4) == 0 && "renderW must be a multiple of 4 for CF write alignment");

    flashFrameBuf[0]   = __ardugl_fb0_start;
    flashFrameBuf[1]   = __ardugl_fb1_start;
    flashFrameRenderW  = renderW;
    flashFrameRenderH  = renderH;
    flashFrameSize     = static_cast<size_t>(renderW) * renderH * sizeof(uint16_t);
    activeBufIdx       = 0;
    committedBufIdx    = 1;

    // Open the FSP flash driver (safe to call even if already open —
    // R_FLASH_LP_Open returns FSP_ERR_ALREADY_OPEN in that case, which we ignore).
    R_FLASH_LP_Open(&s_flash_ctrl, &s_flash_cfg);

    return ReturnInfo{ true, EC_OK };
}

int ArduGL::getTilesX()
{
    return (flashFrameRenderW + ARDUGL_TILE_W - 1) / ARDUGL_TILE_W;
}

int ArduGL::getTilesY()
{
    return (flashFrameRenderH + ARDUGL_TILE_H - 1) / ARDUGL_TILE_H;
}

const uint16_t *ArduGL::getCommittedBuffer()
{
    return flashFrameBuf[committedBufIdx];
}

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
// Pipeline geometry helpers (shared by legacy and tiled paths)
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
    return !((b1.blX + b1.width)  < b2.blX || (b2.blX + b2.width)  < b1.blX
          || (b1.blY + b1.height) < b2.blY || (b2.blY + b2.height) < b1.blY);
}

static glm::vec3 computeTriCrossProduct(const glm::vec4 &v1, const glm::vec4 &v2,
                                        const glm::vec4 &v3)
{
    return glm::cross(glm::vec3(v2.x - v1.x, v2.y - v1.y, 0.0f),
                      glm::vec3(v3.x - v1.x, v3.y - v1.y, 0.0f));
}

// Rasterize triangle into coveredFragments, clamped to [xClampMin,xClampMax) x [yClampMin,yClampMax).
// All coordinates are in screen space (Y-up).
static void rasterizeTriangle(const AABB &triAABB, const glm::vec4 &v1, const glm::vec4 &v2,
                               const glm::vec4 &v3, std::vector<glm::vec2> &coveredFragments,
                               int xClampMin, int xClampMax, int yClampMin, int yClampMax)
{
    const glm::vec3 v1Pos{ v1.x, v1.y, 0.0f };
    const glm::vec3 v2Pos{ v2.x, v2.y, 0.0f };
    const glm::vec3 v3Pos{ v3.x, v3.y, 0.0f };
    coveredFragments.clear();

    const int xMin = glm::max(static_cast<int>(glm::ceil(triAABB.blX)),              xClampMin);
    const int xMax = glm::min(static_cast<int>(glm::ceil(triAABB.blX + triAABB.width)),  xClampMax);
    const int yMin = glm::max(static_cast<int>(glm::ceil(triAABB.blY)),              yClampMin);
    const int yMax = glm::min(static_cast<int>(glm::ceil(triAABB.blY + triAABB.height)), yClampMax);

    if (xMin >= xMax || yMin >= yMax) return;

    constexpr float epsilon = 1.0e-6f;

    for (int x = xMin; x < xMax; ++x)
    {
        for (int y = yMin; y < yMax; ++y)
        {
            const glm::vec3 fp{ static_cast<float>(x) + 0.5f,
                                static_cast<float>(y) + 0.5f, 0.0f };

            const float ce1 = glm::cross(fp - v1Pos, v2Pos - v1Pos).z;
            const float ce2 = glm::cross(fp - v2Pos, v3Pos - v2Pos).z;
            const float ce3 = glm::cross(fp - v3Pos, v1Pos - v3Pos).z;

            if (ce1 > epsilon && ce2 > epsilon && ce3 > epsilon)
            {
                coveredFragments.emplace_back(static_cast<float>(x), static_cast<float>(y));
                continue;
            }

            // Top-left rule for edge pixels
            if (ce1 < 0.0f || ce2 < 0.0f || ce3 < 0.0f) continue;

            auto edgeIsLeftOrTop = [&](const glm::vec3 &a, const glm::vec3 &b,
                                       const glm::vec3 &other) -> bool {
                const bool isTop  = glm::abs(a.y - b.y) < epsilon
                                    && a.y > other.y && b.y > other.y;
                const bool isLeft = !isTop
                                    && ((a.x < b.x && a.x < other.x)
                                        || (b.x < a.x && b.x < other.x));
                return isTop || isLeft;
            };

            const bool e1ok = (ce1 <= epsilon) ? edgeIsLeftOrTop(v1Pos, v2Pos, v3Pos) : true;
            const bool e2ok = (ce2 <= epsilon) ? edgeIsLeftOrTop(v2Pos, v3Pos, v1Pos) : true;
            const bool e3ok = (ce3 <= epsilon) ? edgeIsLeftOrTop(v3Pos, v1Pos, v2Pos) : true;

            if (e1ok && e2ok && e3ok)
                coveredFragments.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
    }
}

static glm::vec3 computeBarycentricCoordinates(const glm::vec2 &point, const glm::vec4 &v1,
                                               const glm::vec4 &v2, const glm::vec4 &v3)
{
    const glm::vec3 p  { point.x, point.y, 0.0f };
    const glm::vec3 a  { v1.x,    v1.y,    0.0f };
    const glm::vec3 b  { v2.x,    v2.y,    0.0f };
    const glm::vec3 c  { v3.x,    v3.y,    0.0f };

    const float totalArea = glm::length(glm::cross(c - a, b - a));
    if (totalArea < 1e-10f) return { 1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f };

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
                          const std::vector<float> &attrs1, const std::vector<float> &attrs2,
                          const std::vector<float> &attrs3,
                          uint16_t *colorBuf, uint8_t *depthBuf)
{
    const glm::vec2 fc{ static_cast<float>(fragX) + 0.5f,
                        static_cast<float>(fragY) + 0.5f };
    const glm::vec3 bary = computeBarycentricCoordinates(fc, sv1, sv2, sv3);

    const glm::vec3 oneOverWs{ 1.0f / sv1.w, 1.0f / sv2.w, 1.0f / sv3.w };
    const float     oneOverW  = glm::dot(bary, oneOverWs);

    // Depth test (write into local tile buffer)
    const int localIdx = (fragY - bufOffsetY) * bufWidth + (fragX - bufOffsetX);

    const float depth = glm::dot(bary, glm::vec3(sv1.z, sv2.z, sv3.z));
    const float storedDepth = depthBuf[localIdx] / 255.0f;
    if (depth >= storedDepth) return;
    depthBuf[localIdx] = packDepthIntoByte(depth);

    // Attribute interpolation (perspective-correct)
    const int numAttrs = static_cast<int>(attrs1.size());
    std::vector<float> interp;
    interp.reserve(numAttrs);
    for (int a = 0; a < numAttrs; ++a)
    {
        interp.emplace_back(
            glm::dot(bary, glm::vec3(attrs1[a], attrs2[a], attrs3[a]) * oneOverWs) / oneOverW);
    }

    colorBuf[localIdx] = packRGB565(fragmentShaderPtr(interp));
}

// =============================================================================
// Tiled pipeline — binTriangles
// =============================================================================

ArduGL::ReturnInfo ArduGL::binTriangles()
{
    assert(vertexShaderPtr && "Vertex shader not bound");
    assert(vertexBuffer.buffPtr && "Vertex buffer not bound");

    const int totalTriangles = vertexBuffer.buffSize / (3 * vertexBuffer.itemSize);
    assert(totalTriangles <= ARDUGL_MAX_TRIANGLES && "Increase ARDUGL_MAX_TRIANGLES");

    const int tilesX = getTilesX();
    const int tilesY = getTilesY();
    const int totalTiles = tilesX * tilesY;
    assert(totalTiles <= ARDUGL_MAX_TILES && "Increase ARDUGL_MAX_TILES");

    // Reset bins and per-frame flash erase tracker
    memset(tileBinCount,  0, sizeof(uint8_t) * totalTiles);
    memset(eraseBlockDone, 0, sizeof(eraseBlockDone));
    cachedTriangleCount = 0;

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

        // Back-face culling
        if (computeTriCrossProduct(tv1.first, tv2.first, tv3.first).z >= 0.0f)
        {
            cachedTriangles[t].valid = false;
            ++cachedTriangleCount;
            continue;
        }

        const AABB triAABB = computeTriangleAABB(tv1.first, tv2.first, tv3.first);

        // Frustum cull
        if (!checkAABBIntersect(renderTargetDimensions, triAABB))
        {
            cachedTriangles[t].valid = false;
            ++cachedTriangleCount;
            continue;
        }

        // Cache the transformed triangle
        cachedTriangles[t].valid    = true;
        cachedTriangles[t].sv[0]    = tv1.first;
        cachedTriangles[t].sv[1]    = tv2.first;
        cachedTriangles[t].sv[2]    = tv3.first;
        cachedTriangles[t].attrs[0] = tv1.second;
        cachedTriangles[t].attrs[1] = tv2.second;
        cachedTriangles[t].attrs[2] = tv3.second;
        ++cachedTriangleCount;

        // Bin into overlapping tiles
        for (int ty = 0; ty < tilesY; ++ty)
        {
            for (int tx = 0; tx < tilesX; ++tx)
            {
                // Tile AABB in screen space (Y-up)
                const float tileBlX = static_cast<float>(tx * ARDUGL_TILE_W);
                const float tileBlY = static_cast<float>(ty * ARDUGL_TILE_H);
                const AABB tileAABB{ .blX    = tileBlX,
                                     .blY    = tileBlY,
                                     .width  = static_cast<float>(ARDUGL_TILE_W),
                                     .height = static_cast<float>(ARDUGL_TILE_H) };

                if (!checkAABBIntersect(triAABB, tileAABB)) continue;

                const int tileIdx = ty * tilesX + tx;
                if (tileBinCount[tileIdx] < ARDUGL_MAX_TRIS_PER_TILE)
                {
                    tileBins[tileIdx][tileBinCount[tileIdx]++] =
                        static_cast<uint16_t>(t);
                }
            }
        }
    }

    return ReturnInfo{ true, EC_OK };
}

// =============================================================================
// Tiled pipeline — renderTile
// =============================================================================

ArduGL::ReturnInfo ArduGL::renderTile(int tileCol, int tileRow)
{
    assert(fragmentShaderPtr && "Fragment shader not bound");

    // Clear in-RAM tiles with the user-supplied clear color
    for (int i = 0; i < ARDUGL_TILE_W * ARDUGL_TILE_H; ++i) colorTile[i] = clearColorPacked;
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

    std::vector<glm::vec2> coveredFragments;

    for (int b = 0; b < tileBinCount[tileIdx]; ++b)
    {
        const int t = tileBins[tileIdx][b];
        const CachedTriangle &tri = cachedTriangles[t];
        if (!tri.valid) continue;

        const AABB triAABB = computeTriangleAABB(tri.sv[0], tri.sv[1], tri.sv[2]);

        rasterizeTriangle(triAABB, tri.sv[0], tri.sv[1], tri.sv[2], coveredFragments,
                          tileOriginX, tileEndX, tileOriginY, tileEndY);

        for (const glm::vec2 &frag : coveredFragments)
        {
            shadeFragment(static_cast<int>(frag.x), static_cast<int>(frag.y),
                          tileOriginX, tileOriginY, ARDUGL_TILE_W,
                          tri.sv[0], tri.sv[1], tri.sv[2],
                          tri.attrs[0], tri.attrs[1], tri.attrs[2],
                          colorTile, depthTile);
        }
    }

    return ReturnInfo{ true, EC_OK };
}

// =============================================================================
// Tiled pipeline — commitTile
//
// Writes the finished in-RAM color tile into the active flash frame buffer.
//
// Y-flip: the rasterizer uses Y-up (OpenGL convention, row 0 = bottom).
// The display expects Y-down (row 0 = top).  The flip is applied here so
// the committed flash buffer is display-ready.
//
// Flash write strategy:
//   • Erase granularity : 2 KB (CF_BLOCK_SIZE)
//   • Write granularity : 8 bytes (CF_WRITE_SIZE) = 4 pixels
//   • We must erase a full 2 KB block before writing any byte in it.
//   • To avoid erasing the same block twice when two tile rows land in the
//     same 2 KB block, we track which blocks have already been erased this
//     frame in a small bitmask (one bit per 2 KB block, 128 blocks max for
//     256 KB flash → 16 bytes).
//
// The erase bitmask is reset at the start of each frame by binTriangles().
// =============================================================================

ArduGL::ReturnInfo ArduGL::commitTile(int tileCol, int tileRow)
{
    assert(flashFrameBuf[activeBufIdx] && "initFlashBuffers() not called");

    const int renderW     = flashFrameRenderW;
    const int renderH     = flashFrameRenderH;
    const int tileOriginX = tileCol * ARDUGL_TILE_W;
    const int tileOriginY = tileRow * ARDUGL_TILE_H;
    const int tileEndX    = glm::min(tileOriginX + ARDUGL_TILE_W, renderW);
    const int tileEndY    = glm::min(tileOriginY + ARDUGL_TILE_H, renderH);
    const int rowPixels   = tileEndX - tileOriginX;

    // Base address of the active flash frame buffer.
    const uint32_t fbBase = reinterpret_cast<uint32_t>(flashFrameBuf[activeBufIdx]);

    // Temporary 8-byte aligned write buffer (CF_WRITE_SIZE = 8 bytes = 4 pixels).
    // A tile row is at most ARDUGL_TILE_W pixels = ARDUGL_TILE_W*2 bytes.
    // We pad to the next multiple of CF_WRITE_SIZE.
    constexpr int kMaxRowBytes = ((ARDUGL_TILE_W * 2 + CF_WRITE_SIZE - 1)
                                  / CF_WRITE_SIZE) * CF_WRITE_SIZE;
    uint8_t rowBuf[kMaxRowBytes];

    for (int y = tileOriginY; y < tileEndY; ++y)
    {
        // Y-flip: screen row y (Y-up) → display row (renderH-1-y) (Y-down)
        const int displayRow  = (renderH - 1 - y);
        const uint32_t dstAddr = fbBase
                                 + static_cast<uint32_t>(displayRow * renderW + tileOriginX)
                                   * sizeof(uint16_t);

        // Erase every 2 KB block that this row touches (first time only).
        // A row write spans [dstAddr, dstAddr + rowPixels*2) bytes.
        const uint32_t rowEndAddr = dstAddr + static_cast<uint32_t>(rowPixels) * 2;
        for (uint32_t addr = dstAddr & ~(CF_BLOCK_SIZE - 1);
             addr < rowEndAddr;
             addr += CF_BLOCK_SIZE)
        {
            const uint32_t blockIdx = addr / CF_BLOCK_SIZE;
            if (!isBlockErased(blockIdx))
            {
                fsp_err_t err = R_FLASH_LP_Erase(&s_flash_ctrl, addr, 1);
                if (err != FSP_SUCCESS) return ReturnInfo{ false, EC_FlashError };
                markBlockErased(blockIdx);
            }
        }

        // Copy tile row into aligned write buffer, padding with 0xFF.
        const int tileLocalY  = y - tileOriginY;
        const int srcRowStart = tileLocalY * ARDUGL_TILE_W;
        const int rowBytes    = rowPixels * static_cast<int>(sizeof(uint16_t));
        memcpy(rowBuf, colorTile + srcRowStart, rowBytes);
        if (rowBytes < kMaxRowBytes)
            memset(rowBuf + rowBytes, 0xFF, kMaxRowBytes - rowBytes);

        // Write to flash (must be multiple of CF_WRITE_SIZE bytes).
        const int writeBytes = ((rowBytes + CF_WRITE_SIZE - 1) / CF_WRITE_SIZE) * CF_WRITE_SIZE;
        fsp_err_t err = R_FLASH_LP_Write(&s_flash_ctrl,
                                         reinterpret_cast<uint32_t>(rowBuf),
                                         dstAddr,
                                         static_cast<uint32_t>(writeBytes));
        if (err != FSP_SUCCESS) return ReturnInfo{ false, EC_FlashError };
    }

    return ReturnInfo{ true, EC_OK };
}

// =============================================================================
// Tiled pipeline — scheduleDisplayTransfer
// =============================================================================

bool ArduGL::isDisplayTransferBusy()
{
    return dmaTransferBusy;
}

#if ARDUGL_USE_HW_SPI_DMA

// SPI0 TEI ISR — fires when the last byte has left the shift register.
// Deasserts CS, clears the busy flag, and flips the ping-pong index.
extern "C" void ardugl_spi_tei_isr()
{
    // Deassert CS (active-low)
    R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)dmaCsPin, BSP_IO_LEVEL_HIGH);

    // Disable SPI TX
    R_SPI0->SPCR &= ~(1u << 3); // clear SPE

    // Flip ping-pong: the buffer we just sent is now the committed one
    committedBufIdx = activeBufIdx;
    activeBufIdx    = 1 - activeBufIdx;

    dmaTransferBusy = false;
}

ArduGL::ReturnInfo ArduGL::scheduleDisplayTransfer(uint8_t csPin, uint8_t dcPin)
{
    if (dmaTransferBusy) return ReturnInfo{ false, EC_NotReady };

    dmaCsPin = csPin;
    dmaDcPin = dcPin;

    const uint16_t *src    = pingPongBuf[activeBufIdx];
    const uint32_t  nBytes = static_cast<uint32_t>(pingPongSize);
    // DMAC normal mode: max 0xFFFF transfers per open.  For buffers larger
    // than 65535 bytes a block-mode or chained approach is needed; at
    // 60×33×2 = 3960 bytes we are well within the limit.
    const uint16_t nTransfers = static_cast<uint16_t>(nBytes / 2); // 16-bit transfers

    // Assert DC (data mode)
    R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)dcPin, BSP_IO_LEVEL_HIGH);
    // Assert CS
    R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)csPin, BSP_IO_LEVEL_LOW);

    // Configure DMAC transfer info
    dmacInfo.transfer_settings_word_b.mode          = TRANSFER_MODE_NORMAL;
    dmacInfo.transfer_settings_word_b.size          = TRANSFER_SIZE_2_BYTE;
    dmacInfo.transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
    dmacInfo.transfer_settings_word_b.dest_addr_mode= TRANSFER_ADDR_MODE_FIXED;
    dmacInfo.transfer_settings_word_b.irq           = TRANSFER_IRQ_END;
    dmacInfo.p_src   = src;
    dmacInfo.p_dest  = &R_SPI0->SPDR;
    dmacInfo.length  = nTransfers;
    dmacInfo.num_blocks = 0;

    dmacExtCfg.channel           = 0;
    dmacExtCfg.irq               = FSP_INVALID_VECTOR;
    dmacExtCfg.ipl               = 0;
    dmacExtCfg.activation_source = ELC_EVENT_SPI0_TXI;
    dmacExtCfg.p_callback        = nullptr;
    dmacExtCfg.p_context         = nullptr;

    dmacCfg.p_info   = &dmacInfo;
    dmacCfg.p_extend = &dmacExtCfg;

    // Install TEI ISR for CS deassert
    // SPI0_TEI is not in the pre-built vector table for UNO R4 WiFi;
    // we install it directly into the ICU software-configurable slot.
    // IRQ slot 8 is unused in the framework vector_data.h.
    constexpr IRQn_Type kTeiIrq = static_cast<IRQn_Type>(8);
    R_ICU->IELSR[kTeiIrq] = ELC_EVENT_SPI0_TEI;
    NVIC_SetVector(kTeiIrq, reinterpret_cast<uint32_t>(ardugl_spi_tei_isr));
    NVIC_SetPriority(kTeiIrq, 12);
    NVIC_EnableIRQ(kTeiIrq);

    // Open (or reconfigure) DMAC
    R_DMAC_Close(&dmacCtrl);
    fsp_err_t err = R_DMAC_Open(&dmacCtrl, &dmacCfg);
    if (err != FSP_SUCCESS)
    {
        R_IOPORT_PinWrite(nullptr, (bsp_io_port_pin_t)csPin, BSP_IO_LEVEL_HIGH);
        return ReturnInfo{ false, EC_FlashError };
    }
    R_DMAC_Enable(&dmacCtrl);

    // Enable SPI TX (SPE + SPTIE)
    R_SPI0->SPCR |= (1u << 3) | (1u << 7); // SPE | SPTIE

    dmaTransferBusy = true;
    return ReturnInfo{ true, EC_OK };
}

#else // ARDUGL_USE_HW_SPI_DMA == 0

ArduGL::ReturnInfo ArduGL::scheduleDisplayTransfer(uint8_t /*csPin*/, uint8_t /*dcPin*/)
{
    // Soft-SPI fallback: flip ping-pong immediately (no async transfer).
    // The caller reads getCommittedBuffer() and pushes via Adafruit writePixels().
    // The active buffer (just finished) becomes the committed (display-ready) one.
    committedBufIdx = activeBufIdx;
    activeBufIdx    = 1 - activeBufIdx;
    return ReturnInfo{ true, EC_OK };
}

#endif // ARDUGL_USE_HW_SPI_DMA

// =============================================================================
// Legacy renderPrimitives — writes directly into the bound BT_Color / BT_Depth
// buffers (no tiling).  Kept so testpipeline.cpp continues to work unchanged.
// =============================================================================

ArduGL::ReturnInfo ArduGL::renderPrimitives()
{
    assert(vertexShaderPtr   && "Vertex shader not bound");
    assert(fragmentShaderPtr && "Fragment shader not bound");
    assert(colorBuffer.buffPtr && depthBuffer.buffPtr && "Color/depth buffers not bound");

    std::vector<glm::vec2> coveredFragments;
    const int renderW = static_cast<int>(renderTargetDimensions.width);

    const int totalTriangles = vertexBuffer.buffSize / (3 * vertexBuffer.itemSize);

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

        if (computeTriCrossProduct(tv1.first, tv2.first, tv3.first).z >= 0.0f) continue;

        const AABB triAABB = computeTriangleAABB(tv1.first, tv2.first, tv3.first);
        if (!checkAABBIntersect(renderTargetDimensions, triAABB)) continue;

        rasterizeTriangle(triAABB, tv1.first, tv2.first, tv3.first, coveredFragments,
                          0, renderW,
                          0, static_cast<int>(renderTargetDimensions.height));

        uint16_t *colorBuf = reinterpret_cast<uint16_t *>(colorBuffer.buffPtr);
        uint8_t  *depthBuf = reinterpret_cast<uint8_t  *>(depthBuffer.buffPtr);

        for (const glm::vec2 &frag : coveredFragments)
        {
            const int fx = static_cast<int>(frag.x);
            const int fy = static_cast<int>(frag.y);
            const int idx = fy * renderW + fx;

            const glm::vec2 fc{ static_cast<float>(fx) + 0.5f,
                                static_cast<float>(fy) + 0.5f };
            const glm::vec3 bary = computeBarycentricCoordinates(fc,
                                       tv1.first, tv2.first, tv3.first);
            const glm::vec3 oneOverWs{ 1.0f / tv1.first.w,
                                       1.0f / tv2.first.w,
                                       1.0f / tv3.first.w };
            const float oneOverW = glm::dot(bary, oneOverWs);

            // Depth test
            const float depth       = glm::dot(bary, glm::vec3(tv1.first.z,
                                                                tv2.first.z,
                                                                tv3.first.z));
            const float storedDepth = depthBuf[idx] / 255.0f;
            if (depth >= storedDepth) continue;
            depthBuf[idx] = packDepthIntoByte(depth);

            // Attribute interpolation
            const int numAttrs = static_cast<int>(tv1.second.size());
            std::vector<float> interp;
            interp.reserve(numAttrs);
            for (int a = 0; a < numAttrs; ++a)
            {
                interp.emplace_back(
                    glm::dot(bary,
                             glm::vec3(tv1.second[a], tv2.second[a], tv3.second[a])
                             * oneOverWs)
                    / oneOverW);
            }

            colorBuf[idx] = packRGB565(fragmentShaderPtr(interp));
        }
    }

    return ReturnInfo{ true, EC_OK };
}

ArduGL::ReturnInfo ArduGL::renderIndexedPrimitives()
{
    // Not yet implemented.
    return ReturnInfo{ false, EC_InvalidOperation };
}
