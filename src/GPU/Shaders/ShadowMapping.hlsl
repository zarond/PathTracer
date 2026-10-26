#include "Common.hlsl"

struct ShadowConstantBuffer {
    float4x4 viewProjection;
    float4 lightDirection;
    float4 lightPosition;
    int isPointLight;
};

struct ShadowPerDrawData {
    row_major float3x4 modelMatrix;
    int meshID;
};

// Constant Buffer
ConstantBuffer<ShadowConstantBuffer> g_shadowCB : register(b0);

// Materials data buffers
StructuredBuffer<Material> Materials : register(t0, space0);

// Default sampler
SamplerState Sampler : register(s0, space0);

// Anisotropic sampler
SamplerState ASampler : register(s1, space0);

// Per draw call data
ConstantBuffer<ShadowPerDrawData> DrawData : register(b1);

struct SHInput {
    float4 ndc_position : SV_POSITION;
    float4 world_position : POSITION;
    float4 uv : TEXCOORD;
};

[shader("vertex")] 
SHInput VS_Shadows(
    float4 position : POSITION, 
    float4 normal : NORMAL, 
    float4 tangent : TANGENT, 
    float4 uv : TEXCOORD) 
{
    SHInput result;

    position.w = 1.0f;
    position.xyz = mul(DrawData.modelMatrix, position);
    result.world_position = position;
    result.ndc_position = mul(g_shadowCB.viewProjection, position);
    result.uv = uv;

    return result;
}

[shader("pixel")] 
float4 PS_Shadows(SHInput input) : SV_TARGET {
    Material mat = Materials[DrawData.meshID];
    float2 uv = input.uv.xy;
    float alpha = sample_albedo_filtered(mat, uv, ASampler).w;
    float D;
    float3 L = input.world_position.xyz - g_shadowCB.lightPosition.xyz;
    if (g_shadowCB.isPointLight) {
        D = length(L);
    } else {
        D = dot(L, g_shadowCB.lightDirection.xyz);
    }
    D += fwidth( D ); // main slope-based bias
    if (alpha < mat.alpha_cutoff) discard;  // alpha-test in the same shader for simplicity, but it disables early-z optimisation
    return float4(D, D * D, 0, 0);
}
