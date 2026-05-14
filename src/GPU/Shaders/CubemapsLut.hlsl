#include "BRDF.hlsl"
#include "Common.hlsl"

// Output texture, accessed as a UAV
RWTexture2DArray<float4> gOutput : register(u0);

// Envmap
Texture2D<float4> EnvMap : register(t0, space1);
SamplerState EnvMapSampler : register(s0, space1);

float3 CalculateCubeDirection(float2 uv, uint faceIndex)
{
    float3 dir = 0;
    switch(faceIndex)
    {
        case 0: dir = float3( 1.0f,  uv.y, -uv.x); break; // +X
        case 1: dir = float3(-1.0f,  uv.y,  uv.x); break; // -X
        case 2: dir = float3( uv.x,  1.0f, -uv.y); break; // +Y
        case 3: dir = float3( uv.x, -1.0f,  uv.y); break; // -Y
        case 4: dir = float3( uv.x,  uv.y,  1.0f); break; // +Z
        case 5: dir = float3(-uv.x,  uv.y, -1.0f); break; // -Z
    }
    return normalize(dir);
}

void CalculateCubeClosestTangents(out float3 T, out float3 B, uint faceIndex)
{
    switch(faceIndex)
    {
        case 0: T = float3(0, 1, 0); B = float3(0, 0, 1); break; // +X
        case 1: T = float3(0, 0, 1); B = float3(0, 1, 0); break; // -X
        case 2: T = float3(0, 0, 1); B = float3(1, 0, 0); break; // +Y
        case 3: T = float3(1, 0, 0); B = float3(0, 0, 1); break; // -Y
        case 4: T = float3(1, 0, 0); B = float3(0, 1, 0); break; // +Z
        case 5: T = float3(0, 1, 0); B = float3(1, 0, 0); break; // -Z
    }
    return;
}

float3 SampleEnvmap(float3 dir) {  // dir is expected to be normalized
    float2 uv = float2(atan2(-dir.z, -dir.x), -2.0f * asin(dir.y)) * (1.0f / PI);
    uv = uv * 0.5f + 0.5f;
    return EnvMap.SampleLevel(EnvMapSampler, uv, 0).xyz;
}

[numthreads(8, 8, 1)] 
void CS_Diffuse_Lut(uint3 DTid : SV_DispatchThreadID) {
    uint3 destSize;
    gOutput.GetDimensions(destSize.x, destSize.y, destSize.z);

    if (DTid.x >= (uint)destSize.x || DTid.y >= (uint)destSize.y || DTid.z >= 6) return;

    float2 uv = (DTid.xy + 0.5f) / destSize;
    uv = uv * 2.0f - 1.0f;
    uv.y *= -1.0f;  // Flip Y because texture space is top-down

    float3 N = CalculateCubeDirection(uv, DTid.z);
    float3 T;
    float3 B;
    CalculateCubeClosestTangents(T, B, DTid.z);

    float3x3 TBN = construct_TBN(T, B, N);

    uint3 seed = uint3(DTid.x, DTid.y, DTid.z);
    float2 jitter = float2(pcg3d16(seed).xy) / float(0xFFFF);

    float3 result = 0.0f;
    const int N_samples = 2048 * 4;  // todo: prefilter the envmap and use fewer samples for rougher mip levels
    const float inv_N_samples = 1.0f / N_samples;
    for (int i = 0; i < N_samples; ++i) {
        float2 rand = fibonacci2D(i, inv_N_samples);
        rand = fmod(rand + jitter, 1.0f);
        float3 l = ImportanceSampleCosDir(rand);
        l = Tangent2World(l, TBN);

        const float LoN = saturate(dot(l, N));
        if (LoN > 0) {
            result += SampleEnvmap(l);
        }
    }
    result *= inv_N_samples;

    gOutput[DTid.xyz] = float4(result, 1.0);
}