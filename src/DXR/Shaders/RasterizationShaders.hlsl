#include "BRDF.hlsl"
#include "Common.hlsl"

// Constant Buffer
ConstantBuffer<RasterConstantBuffer> g_rasterCB : register(b0);

// Materials data buffers
StructuredBuffer<Material> Materials : register(t1, space0);

// Envmap
Texture2D<float4> EnvMap : register(t0, space1);

// Default sampler
SamplerState Sampler : register(s0, space1);

// Per draw call data
ConstantBuffer<RasterPerDrawData> DrawData : register(b1);

struct PSInput {
    float4 position : SV_POSITION;
    float4 normal : NORMAL;
    float4 tangent : TANGENT;
    float4 uv : TEXCOORD;
};

[shader("vertex")] 
PSInput VS_Main(
    float4 position : POSITION, 
    float4 normal : NORMAL, 
    float4 tangent : TANGENT, 
    float4 uv : TEXCOORD) 
{
    PSInput result;

    position.w = 1.0f;
    normal.w = 0.0f;
    position = mul(DrawData.modelMatrix, position);
    float tangent_sign = tangent.w;
    result.position = mul(g_rasterCB.viewProjection, position);
    result.normal = mul(DrawData.normalMatrix, normal);
    result.tangent = mul(DrawData.modelMatrix, tangent);
    result.tangent.w = tangent_sign;
    result.uv = uv;

    return result;
}

[shader("pixel")]
float4 PS_Main(PSInput input) : SV_TARGET {
    Material mat = Materials[DrawData.meshID];
    float4 color = sample_albedo(mat, input.uv.xy, Sampler);
    return color;
}

// Background pass

struct BG_VS_OUTPUT {
    float4 position : SV_POSITION;
    float3 viewDir : TEXCOORD0;
};

[shader("vertex")] 
BG_VS_OUTPUT VS_Background(uint vID : SV_VERTEXID) {
    BG_VS_OUTPUT output;

    // 1. Generate NDC coordinates procedurally
    // vID 0: (-1, -1), vID 1: (-1, 3), vID 2: (3, -1)
    float2 uv = float2((vID << 1) & 2, vID & 2);
    output.position = float4(uv * 2.0f - 1.0f, 0.0f, 1.0f);

    // 2. Reconstruct World-Space direction
    float4 worldPos = mul(g_rasterCB.projectionToWorld, output.position);

    // Divide by W to get the actual world position,
    output.viewDir = worldPos.xyz / worldPos.w;
    output.position.zw = 1.0f;

    return output;
}

[shader("pixel")]
float4 PS_Background(BG_VS_OUTPUT input)
    : SV_TARGET {
    float y_rotation = g_rasterCB.envmapRotation;
    float3 dir = normalize(input.viewDir);
    float2 uv = float2(atan2(-dir.z, -dir.x) + y_rotation, -2.0f * asin(dir.y)) * (1.0f / PI);
    uv = uv * 0.5f + 0.5f;
    float4 envColor = EnvMap.SampleLevel(Sampler, uv, 0);
    envColor.w = 1.0f;
    return envColor;
}
