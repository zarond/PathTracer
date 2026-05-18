
#include "ColorUtility.hlsl"

struct MipCSInput {
    int numMips;
    int isSRGB;
    int isNormalMap;
    int startingMip;
};

// Bind an array containing a UAV for each mip level
RWTexture2D<float4> OutputMips[] : register(u0, space0);

// A global buffer holding atomic counters to coordinate thread groups
RWByteAddressBuffer GroupCounters : register(u0, space1); // unused for now

// Input parameters
ConstantBuffer<MipCSInput> InputInfo : register(b0);

// Shared memory used to pass data between threads inside this group
groupshared float4 LDS_Color[16][16];

float4 Combine(float4 c0, float4 c1, float4 c2, float4 c3) {
    if (InputInfo.isSRGB) {
        float4 avg = (srgb_to_linear(c0) + srgb_to_linear(c1) + srgb_to_linear(c2) + srgb_to_linear(c3)) * 0.25f;
        return linear_to_srgb(avg);
    } else if (InputInfo.isNormalMap) {
        float3 avg = (c0.xyz + c1.xyz + c2.xyz + c3.xyz) * 0.25f;
        avg = normalize(avg * 2.0f - 1.0f) * 0.5f + 0.5f;
        return float4(avg, 1.0f);
    } else {
        return (c0 + c1 + c2 + c3) * 0.25f; 
    }
}

// Assuming 256 threads per group (16x16)
[numthreads(16, 16, 1)] void CS_SinglePassMips(
    uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) {
    
    if (InputInfo.numMips < 2) return;

    // --- MIP 0 to MIP 1 ---
    // Load a 2x2 quad from the Source Texture (Mip 0)
    uint2 srcCoord = DTid.xy * 2;
    float4 p0 = OutputMips[0][srcCoord + uint2(0, 0)];
    float4 p1 = OutputMips[0][srcCoord + uint2(1, 0)];
    float4 p2 = OutputMips[0][srcCoord + uint2(0, 1)];
    float4 p3 = OutputMips[0][srcCoord + uint2(1, 1)];

    float4 mip1Color = Combine(p0, p1, p2, p3);

    // Write Mip 1 out to global memory
    OutputMips[1][DTid.xy] = mip1Color;

    if (InputInfo.numMips < 3) return; 

    // Store in Local Data Store (LDS) for the next step
    LDS_Color[Tid.x][Tid.y] = mip1Color;
    GroupMemoryBarrierWithGroupSync();

    // --- MIP 1 to MIP 2 ---
    // Only 1/4 of the threads (an 8x8 grid) continue to compute Mip 2
    if ((Tid.x % 2 == 0) && (Tid.y % 2 == 0)) {
        float4 m0 = LDS_Color[Tid.x + 0][Tid.y + 0];
        float4 m1 = LDS_Color[Tid.x + 1][Tid.y + 0];
        float4 m2 = LDS_Color[Tid.x + 0][Tid.y + 1];
        float4 m3 = LDS_Color[Tid.x + 1][Tid.y + 1];

        float4 mip2Color = Combine(m0, m1, m2, m3);
        OutputMips[2][DTid.xy / 2] = mip2Color;

        // Overwrite the LDS layout compaction
        LDS_Color[Tid.x][Tid.y] = mip2Color;
    }

    if (InputInfo.numMips < 4) return; 

    GroupMemoryBarrierWithGroupSync();

    // --- MIP 2 to MIP 3 ---
    // A 4x4 grid continues
    if ((Tid.x % 4 == 0) && (Tid.y % 4 == 0)) {
        float4 m0 = LDS_Color[Tid.x + 0][Tid.y + 0];
        float4 m1 = LDS_Color[Tid.x + 2][Tid.y + 0];
        float4 m2 = LDS_Color[Tid.x + 0][Tid.y + 2];
        float4 m3 = LDS_Color[Tid.x + 2][Tid.y + 2];

        float4 mip3Color = Combine(m0, m1, m2, m3);
        OutputMips[3][DTid.xy / 4] = mip3Color;

        LDS_Color[Tid.x][Tid.y] = mip3Color;
    }

    if (InputInfo.numMips < 5) return;

    GroupMemoryBarrierWithGroupSync();

    // --- MIP 3 to MIP 4 ---
    // A 2x2 grid continues
    if ((Tid.x % 8 == 0) && (Tid.y % 8 == 0)) {
        float4 m0 = LDS_Color[Tid.x + 0][Tid.y + 0];
        float4 m1 = LDS_Color[Tid.x + 4][Tid.y + 0];
        float4 m2 = LDS_Color[Tid.x + 0][Tid.y + 4];
        float4 m3 = LDS_Color[Tid.x + 4][Tid.y + 4];

        float4 mip4Color = Combine(m0, m1, m2, m3);
        OutputMips[4][DTid.xy / 8] = mip4Color;

        LDS_Color[Tid.x][Tid.y] = mip4Color;
    }

    if (InputInfo.numMips < 6) return;

    GroupMemoryBarrierWithGroupSync();

    // --- MIP 4 to MIP 5 ---
    // Only 1 thread per group (the 16x16 block manager) reaches here
    if (Gidx == 0) {
        float4 m0 = LDS_Color[0][0];
        float4 m1 = LDS_Color[8][0];
        float4 m2 = LDS_Color[0][8];
        float4 m3 = LDS_Color[8][8];

        float4 mip5Color = Combine(m0, m1, m2, m3);
        OutputMips[5][Gid.xy] = mip5Color;
    }
}