
#include "ColorUtility.hlsl"

struct MipCSInput {
    int numMips;
    int isSRGB;
    int isNormalMap;
    int numGroups;
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

// Warning: with non-power-of-2 textures mips have slight texture offset
void Compute5Mips(int base_mip, uint3 DTid, uint3 Gid, uint3 Tid, uint Gidx, uint2 imageSize) {
    if (base_mip + 1 >= InputInfo.numMips) return;

    imageSize = max(1, imageSize / 2);

    float4 mip1Color = 0.0f;
    if (all(DTid.xy < imageSize)) {
        // --- MIP 0 to MIP 1 ---
        // Load a 2x2 quad from the Source Texture (Mip 0)
        uint2 srcCoord = DTid.xy * 2;
        float4 p0 = OutputMips[base_mip][srcCoord + uint2(0, 0)];
        float4 p1 = OutputMips[base_mip][srcCoord + uint2(1, 0)];
        float4 p2 = OutputMips[base_mip][srcCoord + uint2(0, 1)];
        float4 p3 = OutputMips[base_mip][srcCoord + uint2(1, 1)];

        mip1Color = Combine(p0, p1, p2, p3);

        // Write Mip 1 out to global memory
        OutputMips[base_mip + 1][DTid.xy] = mip1Color;
    }

    if (base_mip + 2 >= InputInfo.numMips) return;

    // Store in Local Data Store (LDS) for the next step
    LDS_Color[Tid.x][Tid.y] = mip1Color;
    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 1 to MIP 2 ---
    // Only 1/4 of the threads (an 8x8 grid) continue to compute Mip 2
    if (Gidx < 64) {
        uint2 newTid = uint2(Gidx % 8, Gidx / 8);
        uint2 ldsSrc = newTid * 2;

        float4 m0 = LDS_Color[ldsSrc.x + 0][ldsSrc.y + 0];
        float4 m1 = LDS_Color[ldsSrc.x + 1][ldsSrc.y + 0];
        float4 m2 = LDS_Color[ldsSrc.x + 0][ldsSrc.y + 1];
        float4 m3 = LDS_Color[ldsSrc.x + 1][ldsSrc.y + 1];

        float4 mip2Color = Combine(m0, m1, m2, m3);
        uint2 outCoord = Gid.xy * 8 + newTid;
        
        if (all(outCoord < imageSize)) {
            OutputMips[base_mip + 2][outCoord] = mip2Color;
        }

        LDS_Color[ldsSrc.x + newTid.y % 2][ldsSrc.y] = mip2Color;  // stair-stepping offset on X to minimize memory bank conflicts
    }

    if (base_mip + 3 >= InputInfo.numMips) return;

    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 2 to MIP 3 ---
    // A 4x4 grid continues
    if (Gidx < 16) {
        uint2 newTid = uint2(Gidx % 4, Gidx / 4);
        uint2 ldsSrc = newTid * 4;
        uint2 Delta = uint2(newTid.y % 2, (newTid.y + 1) % 2);  // stair-stepping offset on X to minimize memory bank conflicts

        float4 m0 = LDS_Color[ldsSrc.x + Delta.x + 0][ldsSrc.y + 0];
        float4 m1 = LDS_Color[ldsSrc.x + Delta.x + 2][ldsSrc.y + 0];
        float4 m2 = LDS_Color[ldsSrc.x + Delta.y + 0][ldsSrc.y + 2];
        float4 m3 = LDS_Color[ldsSrc.x + Delta.y + 2][ldsSrc.y + 2];

        float4 mip3Color = Combine(m0, m1, m2, m3);
        uint2 outCoord = Gid.xy * 4 + newTid;
        if (all(outCoord < imageSize)) {
            OutputMips[base_mip + 3][outCoord] = mip3Color;
        }

        LDS_Color[ldsSrc.x + newTid.y % 4][ldsSrc.y] = mip3Color;
    }

    if (base_mip + 4 >= InputInfo.numMips) return;

    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 3 to MIP 4 ---
    // A 2x2 grid continues
    if (Gidx < 4) {
        uint2 newTid = uint2(Gidx % 2, Gidx / 2);
        uint2 ldsSrc = newTid * 8;
        uint2 Delta = uint2(newTid.y % 4, (newTid.y + 1) % 4);  // stair-stepping offset on X to minimize memory bank conflicts

        float4 m0 = LDS_Color[ldsSrc.x + Delta.x + 0][ldsSrc.y + 0];
        float4 m1 = LDS_Color[ldsSrc.x + Delta.x + 4][ldsSrc.y + 0];
        float4 m2 = LDS_Color[ldsSrc.x + Delta.y + 0][ldsSrc.y + 4];
        float4 m3 = LDS_Color[ldsSrc.x + Delta.y + 4][ldsSrc.y + 4];

        float4 mip4Color = Combine(m0, m1, m2, m3);
        uint2 outCoord = Gid.xy * 2 + newTid;
        if (all(outCoord < imageSize)) {
            OutputMips[base_mip + 4][outCoord] = mip4Color;
        }

        LDS_Color[ldsSrc.x + newTid.y % 8][ldsSrc.y] = mip4Color;
    }

    if (base_mip + 5 >= InputInfo.numMips) return;

    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 4 to MIP 5 ---
    // Only 1 thread per group (the 16x16 block manager) reaches here
    if (Gidx == 0) {
        if (all(Gid.xy < imageSize)) {
            float4 m0 = LDS_Color[0][0];
            float4 m1 = LDS_Color[8][0];
            float4 m2 = LDS_Color[0 + 1][8];
            float4 m3 = LDS_Color[8 + 1][8];

            float4 mip5Color = Combine(m0, m1, m2, m3);
            OutputMips[base_mip + 5][Gid.xy] = mip5Color;
        }
    }
}

// Assuming 256 threads per group (16x16)
[numthreads(16, 16, 1)] 
void CS_SinglePassMips(
    uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) {
    uint2 imageSize;
    OutputMips[0].GetDimensions(imageSize.x, imageSize.y);

    Compute5Mips(0, DTid, Gid, Tid, Gidx, imageSize);

    if (InputInfo.numMips < 7) return;

    GroupMemoryBarrierWithGroupSync();

    if (Gidx == 0) {
        LDS_Color[0][0] = 0.0f;

        uint currentGroupFinishedCount;
        GroupCounters.InterlockedAdd(0, 1, currentGroupFinishedCount);

        // If this is the last group to finish, it is the survivor!
        if (currentGroupFinishedCount == InputInfo.numGroups - 1) {
            // Reset counter for next frame
            GroupCounters.Store(0, 0);
            // Set value in shared memory indicating that this is a surviving group;
            LDS_Color[0][0] = 1.0f;
        }
    }

    GroupMemoryBarrierWithGroupSync();

    if (LDS_Color[0][0].x == 0.0f) return; // finish all groups except surviving group

    GroupMemoryBarrierWithGroupSync();

    for (int base_mip = 5; base_mip < InputInfo.numMips - 1; base_mip += 5) {
        imageSize = max(1, imageSize >> 5);

        int max_i = (int)imageSize.y;
        int max_j = (int)imageSize.x;

        for (int i = 0; i < max_i; i += 16) {
            for (int j = 0; j < max_j; j += 16) {
                uint3 DTid_v = uint3(j + Tid.x, i + Tid.y, 0);
                uint3 Gid_v = uint3(j / 16, i / 16, 0);

                Compute5Mips(base_mip, DTid_v, Gid_v, Tid, Gidx, imageSize);

                GroupMemoryBarrierWithGroupSync();
            }
        }
    }
}