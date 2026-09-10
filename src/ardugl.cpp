#include "ardugl.h"

#include "glm.hpp"

#include <Arduino.h>

#if !ARDUGL_USE_HW_SPI_ASYNC
#include <Adafruit_ST7789.h>
#else
#include "IRQManager.h"
#include "r_spi.h"
#endif

#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

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
    const char *buffPtr = nullptr;
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
static uint16_t pingPongTile[2][ARDUGL_TILE_W * ARDUGL_TILE_H];
static uint8_t depthTile[ARDUGL_TILE_W * ARDUGL_TILE_H];

// Clear color applied at the start of every renderTile() call.
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
static uint8_t tileBins[ARDUGL_MAX_TILES][ARDUGL_MAX_TRIS_PER_TILE];
static uint8_t tileBinCount[ARDUGL_MAX_TILES]; // number of entries per tile

// Cached per-frame triangle data produced by binTriangles().
static CachedTriangle cachedTriangles[ARDUGL_MAX_TRIANGLES];

// --- Display handle (soft-SPI path) ---
#if !ARDUGL_USE_HW_SPI_ASYNC
static Adafruit_ST7789 *s_tft = nullptr;
#endif

// --- Asynchronous display state ---
static volatile bool spiTransferBusy = false;

#if ARDUGL_USE_HW_SPI_ASYNC
// The staging tile is display-order RGB565 in the same byte order as the
// working synchronous path. FSP's halfword SPI mode sends the high byte first
// with MSB-first and byte swapping disabled. Keeping it separate from the
// rasterizer tiles keeps the source buffer stable while R_SPI_Write() runs.
static uint16_t spiPixelTile[ARDUGL_TILE_W * ARDUGL_TILE_H];
static spi_instance_ctrl_t spiCtrl{};
static spi_cfg_t spiCfg{};
static spi_extended_cfg_t spiExtCfg{};
static bool spiOpened = false;
static volatile bool spiCsRaiseOnComplete = false;
static volatile bool spiTransferFailed = false;
static uint8_t spiCsPin = 10;
static uint8_t spiDcPin = 9;
// Adafruit's 135x240 ST7789 initialization uses x=40 and y=52 for rotation 1.
static uint16_t spiXOffset = 40;
static uint16_t spiYOffset = 52;

static void spiCallback(spi_callback_args_t *p_args);
static void spiInitSt7789();
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

ArduGL::ReturnInfo ArduGL::bindVertexBuffer(const char *buffPtr, int buffSize, int itemSize)
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

#if ARDUGL_USE_HW_SPI_ASYNC
static void initAsyncSpiPipeline(uint8_t csPin, uint8_t dcPin)
{
    if (spiOpened)
        return;

    activeTileIdx = 0;
    committedTileIdx = 1;
    spiCsPin = csPin;
    spiDcPin = dcPin;
    spiTransferBusy = false;
    spiTransferFailed = false;

    pinMode(spiCsPin, OUTPUT);
    pinMode(spiDcPin, OUTPUT);
    digitalWrite(spiCsPin, HIGH);
    digitalWrite(spiDcPin, HIGH);

    // The FSP SPI driver does not configure Arduino pin muxing. Configure the
    // UNO R4's hardware SPI pins directly, then let R_SPI_Open own SPI0.
    const fsp_err_t mosiPinErr = R_IOPORT_PinCfg(&g_ioport_ctrl, digitalPinToBspPin(11),
                                                 IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SPI);
    assert(mosiPinErr == FSP_SUCCESS && "Unable to mux D11 as SPI MOSI");
    // The ST7789 is write-only here, so no MISO pin is required. D12 remains
    // available as an ordinary GPIO.
    const fsp_err_t sckPinErr = R_IOPORT_PinCfg(&g_ioport_ctrl, digitalPinToBspPin(13),
                                                IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SPI);
    assert(sckPinErr == FSP_SUCCESS && "Unable to mux D13 as SPI clock");

    spiCfg.channel = 0;
    spiCfg.rxi_irq = FSP_INVALID_VECTOR;
    spiCfg.txi_irq = FSP_INVALID_VECTOR;
    spiCfg.tei_irq = FSP_INVALID_VECTOR;
    spiCfg.eri_irq = FSP_INVALID_VECTOR;
    spiCfg.rxi_ipl = 12;
    spiCfg.txi_ipl = 12;
    spiCfg.tei_ipl = 12;
    spiCfg.eri_ipl = 12;
    spiCfg.operating_mode = SPI_MODE_MASTER;
    spiCfg.clk_phase = SPI_CLK_PHASE_EDGE_ODD;
    spiCfg.clk_polarity = SPI_CLK_POLARITY_LOW;
    spiCfg.mode_fault = SPI_MODE_FAULT_ERROR_DISABLE;
    spiCfg.bit_order = SPI_BIT_ORDER_MSB_FIRST;
    spiCfg.p_transfer_tx = nullptr;
    spiCfg.p_transfer_rx = nullptr;
    spiCfg.p_callback = spiCallback;
    spiCfg.p_context = nullptr;
    spiCfg.p_extend = &spiExtCfg;

    spiExtCfg.spi_clksyn = SPI_SSL_MODE_CLK_SYN;
    spiExtCfg.spi_comm = SPI_COMMUNICATION_TRANSMIT_ONLY;
    spiExtCfg.ssl_polarity = SPI_SSLP_LOW;
    spiExtCfg.ssl_select = SPI_SSL_SELECT_SSL0;
    spiExtCfg.mosi_idle = SPI_MOSI_IDLE_VALUE_FIXING_DISABLE;
    spiExtCfg.parity = SPI_PARITY_MODE_DISABLE;
    spiExtCfg.byte_swap = SPI_BYTE_SWAP_DISABLE;
    spiExtCfg.spck_delay = SPI_DELAY_COUNT_1;
    spiExtCfg.ssl_negation_delay = SPI_DELAY_COUNT_1;
    spiExtCfg.next_access_delay = SPI_DELAY_COUNT_1;
    // FSP transmits one MSB-first RGB565 halfword per pixel.
    const fsp_err_t bitrateErr = R_SPI_CalculateBitrate(ARDUGL_SPI_BITRATE, &spiExtCfg.spck_div);
    assert(bitrateErr == FSP_SUCCESS && "Unable to calculate SPI bitrate");

    // Arduino's SPI wrapper intentionally leaves these vectors invalid because
    // it performs synchronous register transfers. The FSP driver needs the
    // programmable IRQ slots so its TXI/RXI/TEI handlers can run.
    SpiMasterIrqReq_t irqReq{ .ctrl = &spiCtrl, .cfg = &spiCfg, .hw_channel = 0 };
    const bool irqConfigured = IRQManager::getInstance().addPeripheral(IRQ_SPI_MASTER, &irqReq);
    assert(irqConfigured && "Unable to configure the SPI interrupt vectors");

    const fsp_err_t openErr = R_SPI_Open(&spiCtrl, &spiCfg);
    assert(openErr == FSP_SUCCESS && "Unable to open the FSP SPI channel");
    spiOpened = (openErr == FSP_SUCCESS);

    // Keep all controller commands synchronous. Only pixel payloads use the
    // asynchronous R_SPI_Write() path below.
    spiInitSt7789();
}

void ArduGL::initTiledPipeline(uint8_t csPin, uint8_t dcPin) { initAsyncSpiPipeline(csPin, dcPin); }
#else
void ArduGL::initTiledPipeline(Adafruit_ST7789 *tft, uint8_t csPin, uint8_t dcPin)
{
    s_tft = tft;
    activeTileIdx = 0;
    committedTileIdx = 1;
    (void)csPin;
    (void)dcPin;
}
#endif

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
// =============================================================================

bool ArduGL::isDisplayTransferBusy() { return spiTransferBusy; }

#if ARDUGL_USE_HW_SPI_ASYNC

// R_SPI_Write() completes from the SPI TEI interrupt after the final frame
// leaves the shift register. Keep CS asserted until that callback arrives.
static void spiCallback(spi_callback_args_t *p_args)
{
    if (p_args == nullptr)
        return;

    if (p_args->event == SPI_EVENT_TRANSFER_COMPLETE)
    {
        if (spiCsRaiseOnComplete)
        {
            // End the display transaction before changing D/C. The panel
            // latches D/C only while CS is active, so this leaves the bus in
            // a quiet command-state between tile transfers.
            R_IOPORT_PinWrite(nullptr, digitalPinToBspPin(spiCsPin), BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(nullptr, digitalPinToBspPin(spiDcPin), BSP_IO_LEVEL_LOW);
            spiCsRaiseOnComplete = false;
        }
        spiTransferBusy = false;
        return;
    }

    spiTransferFailed = true;
    if (spiCsRaiseOnComplete)
    {
        R_IOPORT_PinWrite(nullptr, digitalPinToBspPin(spiCsPin), BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(nullptr, digitalPinToBspPin(spiDcPin), BSP_IO_LEVEL_LOW);
        spiCsRaiseOnComplete = false;
    }
    spiTransferBusy = false;
}

// ---------------------------------------------------------------------------
// SPI helpers for the FSP interrupt-driven path
// ---------------------------------------------------------------------------

static inline void spiSetCs(bsp_io_level_t level)
{
    R_IOPORT_PinWrite(nullptr, digitalPinToBspPin(spiCsPin), level);
}

static inline void spiSetDc(bsp_io_level_t level)
{
    R_IOPORT_PinWrite(nullptr, digitalPinToBspPin(spiDcPin), level);
}

static bool spiWaitForIdle()
{
    while (spiTransferBusy)
    {
    }

    if (!spiTransferFailed)
        return true;

    spiTransferFailed = false;
    spiSetCs(BSP_IO_LEVEL_HIGH);
    spiSetDc(BSP_IO_LEVEL_LOW);
    return false;
}

// Synchronous wrapper used only during initialization and address setup.
static bool spiWriteBlocking(const void *data, size_t length,
                             spi_bit_width_t bitWidth = SPI_BIT_WIDTH_8_BITS)
{
    if (!spiWaitForIdle())
        return false;

    spiTransferFailed = false;
    spiCsRaiseOnComplete = false;
    spiTransferBusy = true;
    const fsp_err_t err = R_SPI_Write(&spiCtrl, data, static_cast<uint32_t>(length), bitWidth);
    if (err != FSP_SUCCESS)
    {
        spiTransferBusy = false;
        spiTransferFailed = true;
        return false;
    }

    while (spiTransferBusy)
    {
    }
    return !spiTransferFailed;
}

static inline void spiWriteByte(uint8_t b)
{
    assert(spiWriteBlocking(&b, 1) && "Synchronous SPI write failed");
}

static inline void spiWriteWord(uint16_t w)
{
    const uint8_t bytes[] = { static_cast<uint8_t>(w >> 8), static_cast<uint8_t>(w) };
    assert(spiWriteBlocking(bytes, sizeof(bytes)) && "Synchronous SPI write failed");
}

// Send a ST7789 command byte (DC low) then switch DC high for data.
static inline void spiCommand(uint8_t cmd)
{
    spiSetDc(BSP_IO_LEVEL_LOW);
    spiWriteByte(cmd);
    spiSetDc(BSP_IO_LEVEL_HIGH);
}

// Set the ST7789 address window synchronously.
// x0,y0 — top-left corner in display coordinates (Y-down).
// w, h   — width and height in pixels.
static void spiSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h)
{
    x0 = static_cast<uint16_t>(x0 + spiXOffset);
    y0 = static_cast<uint16_t>(y0 + spiYOffset);

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
    // DC is now high (data); leave CS asserted for the async pixel burst.
}

static void spiSendCommand(uint8_t command, const uint8_t *data = nullptr, size_t dataLength = 0)
{
    spiSetCs(BSP_IO_LEVEL_LOW);
    spiSetDc(BSP_IO_LEVEL_LOW);
    spiWriteByte(command);
    spiSetDc(BSP_IO_LEVEL_HIGH);
    for (size_t i = 0; i < dataLength; ++i)
        spiWriteByte(data[i]);
    spiSetCs(BSP_IO_LEVEL_HIGH);
}

static void spiInitSt7789()
{
    // Match the generic ST7789 sequence used by the working synchronous path.
    spiSendCommand(0x01); // SWRESET
    delay(150);
    spiSendCommand(0x11); // SLPOUT
    delay(10);

    const uint8_t colorMode = 0x55; // 16-bit RGB565
    spiSendCommand(0x3A, &colorMode, 1);
    delay(10);

    const uint8_t genericMadctl = 0x08;
    spiSendCommand(0x36, &genericMadctl, 1);

    const uint8_t columnRange[] = { 0x00, 0x00, 0x00, 0xF0 };
    spiSendCommand(0x2A, columnRange, sizeof(columnRange));
    const uint8_t rowRange[] = { 0x00, 0x00, 0x01, 0x40 };
    spiSendCommand(0x2B, rowRange, sizeof(rowRange));

    spiSendCommand(0x21); // INVON
    delay(10);
    spiSendCommand(0x13); // NORON
    delay(10);
    spiSendCommand(0x29); // DISPON
    delay(10);

    const uint8_t rotation = 0xA0; // MY | MV | RGB, rotation 1
    spiSendCommand(0x36, &rotation, 1);
}

void ArduGL::fillDisplay(uint16_t color)
{
    assert(spiWaitForIdle() && "Display transfer failed before fill");

    for (uint16_t &pixel : spiPixelTile)
        pixel = color;

    spiSetCs(BSP_IO_LEVEL_LOW);
    spiSetAddrWindow(0, 0, 240, 135);
    size_t remaining = static_cast<size_t>(240) * 135;
    while (remaining != 0)
    {
        const size_t count = glm::min(remaining, sizeof(spiPixelTile) / sizeof(spiPixelTile[0]));
        assert(spiWriteBlocking(spiPixelTile, count, SPI_BIT_WIDTH_16_BITS)
               && "Display fill SPI write failed");
        remaining -= count;
    }
    spiSetCs(BSP_IO_LEVEL_HIGH);
    spiSetDc(BSP_IO_LEVEL_LOW);
}

static ArduGL::ReturnInfo scheduleDisplayTransfer(int tileCol, int tileRow)
{
    // Block until the previous tile's SPI transfer completes.
    if (!spiWaitForIdle())
        return ReturnInfo{ false, EC_InvalidOperation };

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
    spiSetCs(BSP_IO_LEVEL_LOW);
    spiSetAddrWindow(static_cast<uint16_t>(tileOriginX), static_cast<uint16_t>(displayY),
                     static_cast<uint16_t>(tileW), static_cast<uint16_t>(tileH));

    // The committed tile is stored Y-up. Pack it into display order so the
    // whole rectangular window is one contiguous SPI transfer, including
    // non-full edge tiles.
    for (int outRow = 0; outRow < tileH; ++outRow)
    {
        const int srcRow = tileH - 1 - outRow;
        for (int x = 0; x < tileW; ++x)
        {
            const uint16_t pixel = pingPongTile[committedTileIdx][srcRow * ARDUGL_TILE_W + x];
            const size_t out = static_cast<size_t>(outRow * tileW + x);
            spiPixelTile[out] = pixel;
        }
    }

    // Start the pixel stream asynchronously. The buffer remains untouched
    // until SPI_EVENT_TRANSFER_COMPLETE, so the CPU can rasterize the next
    // tile into the other ping-pong tile meanwhile.
    spiCsRaiseOnComplete = true;
    spiTransferFailed = false;
    spiTransferBusy = true;
    const fsp_err_t err = R_SPI_Write(&spiCtrl, spiPixelTile, static_cast<uint32_t>(tileW * tileH),
                                      SPI_BIT_WIDTH_16_BITS);
    if (err != FSP_SUCCESS)
    {
        spiCsRaiseOnComplete = false;
        spiSetCs(BSP_IO_LEVEL_HIGH);
        spiTransferBusy = false;
        return ReturnInfo{ false, EC_InvalidOperation };
    }

    return ReturnInfo{ true, EC_OK };
}

#else // ARDUGL_USE_HW_SPI_ASYNC == 0

static ArduGL::ReturnInfo scheduleDisplayTransfer(int tileCol, int tileRow)
{
    assert(s_tft && "initTiledPipeline() not called");
    s_tft->SPI_CS_HIGH();

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

#endif // ARDUGL_USE_HW_SPI_ASYNC

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
            // On the async SPI path this returns immediately and the transfer runs
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
