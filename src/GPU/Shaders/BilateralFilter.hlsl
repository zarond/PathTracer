#include "BRDF.hlsl"

struct FilterCSInput {
    uint2 FrameSize;
    float2 texel_size;
    float rcp_spatialDenom;
    float rcp_DepthRangeDenom;
    float rcp_NormalRangeDenom;
};

// Gbuff
Texture2D<float4> Gbuffer : register(t0, space0);

// Depth
Texture2D<float> Depth : register(t1, space0);

// Input
Texture2D<float4> Input : register(t2, space0);

// Output
RWTexture2D<float4> Output : register(u0, space0);

// Constant Buffer
ConstantBuffer<FilterCSInput> g_CB : register(b0);

// Default sampler
SamplerState Sampler : register(s0, space0);

[numthreads(16, 16, 1)] 
void CS_Bilateral_filter(uint3 DTid : SV_DispatchThreadID) 
{
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;

    float2 texel = g_CB.texel_size;
    float2 uv = (DTid.xy + 0.5f) * texel;

    float3 centerNormal = Gbuffer.Load(DTid).xyz;
    float4 centerColor = Input.Load(DTid);

    float centerDepth = Depth.Load(DTid);
    if (centerDepth >= 1.0f || centerDepth <= 0.0f) {
        Output[DTid.xy] = centerColor;
        return;
    }

    const float rcp_spatialDenom = g_CB.rcp_spatialDenom;
    const float rcp_DepthRangeDenom = g_CB.rcp_DepthRangeDenom;
    const float rcp_normalRangeDenom = g_CB.rcp_NormalRangeDenom;

    const int Radius = 2;

    float4 accColor = 0;
    float acc_weight = 0;

    for (int y = -Radius; y <= Radius; ++y) {
        for (int x = -Radius; x <= Radius; ++x) {
            int2 samplePos = DTid.xy + int2(x, y);

            samplePos.x = clamp(samplePos.x, 0, int(g_CB.FrameSize.x) - 1);
            samplePos.y = clamp(samplePos.y, 0, int(g_CB.FrameSize.y) - 1);

            float4 sampleColor = Input.Load(int3(samplePos, 0));
            float sampleDepth = Depth.Load(int3(samplePos, 0));
            float3 sampleNormal = Gbuffer.Load(int3(samplePos, 0)).xyz;

             // Spatial Gaussian
            float spatialDist2 = float(x * x + y * y);
            float spatialWeight = exp(-spatialDist2 * rcp_spatialDenom);

            // Range Depth
            float depthDiff = sampleDepth - centerDepth;
            float depthWeight = exp(-(depthDiff * depthDiff) * rcp_DepthRangeDenom);

            // Range Normal
            float normalDiff = 1.0f - max(dot(sampleNormal, centerNormal), 0.0f);
            float normalWeight = exp(-(normalDiff * normalDiff) * rcp_normalRangeDenom);

            float weight = spatialWeight * depthWeight * normalWeight;

            accColor += sampleColor * weight;
            acc_weight += weight;
        }
    }

    Output[DTid.xy] = accColor / max(acc_weight, 1e-5);
}