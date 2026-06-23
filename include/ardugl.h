#pragma once

namespace ArduGL
{
enum BufferType
{
    BT_VertexAttribute,
    BT_VertexIndex,
    BT_Depth,
    BT_Color,
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
};

struct ReturnInfo
{
    bool success{ true };
    ErrorCode code{ EC_OK };
};

// buffer management

ReturnInfo clearBuffer(BufferType buffType, char clearValue = 0);
ReturnInfo bindBuffer(BufferType buffType, char *buffPtr, int buffSize, int itemSize);
ReturnInfo unbindBuffer(BufferType buffType);
ReturnInfo setRenderTargetDimensions(int width, int height);

// shader management

// The client code ensures a) provided function pointers match the signature b) provided raw bytes
// are meaningfully and safely casted
//  std::pair<glm::vec4 /*ndc vertex*/, std::vector<char> /*attributes to be passed down the
//  pipeline*/> vertexShader(const char * vertex); glm::vec4 /*color*/ fragmentShader(const
//  std::vector<char> &interpolatedAttributes);
ReturnInfo bindShader(ShaderType shType, const void *shaderFuncPtr);
ReturnInfo unbindShader(ShaderType shType);

// pipeline operation
ReturnInfo renderPrimitives();
ReturnInfo renderIndexedPrimitives();

// pipeline control variables
} // namespace ArduGL