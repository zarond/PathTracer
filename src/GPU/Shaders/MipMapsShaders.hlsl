
#include "ColorUtility.hlsl"

struct MipCSInput {
    int numMips;
    int isSRGB;
    int isNormalMap;
    int minFilter;
    int numGroups;
    int baseMip;
};

// Bind an array containing a UAV for each mip level
RWTexture2D<float4> OutputMips[] : register(u0, space0);

// A global buffer holding atomic counters to coordinate thread groups
RWByteAddressBuffer GroupCounters : register(u0, space1);

// Input parameters
ConstantBuffer<MipCSInput> InputInfo : register(b0);

// Shared memory used to pass data between threads inside this group
groupshared float4 LDS_Color[16][16];
groupshared uint SurvivorFlag;

float4 Combine(float4 c0, float4 c1, float4 c2, float4 c3) {
    if (InputInfo.isSRGB) {
        float4 avg = (srgb_to_linear(c0) + srgb_to_linear(c1) + srgb_to_linear(c2) + srgb_to_linear(c3)) * 0.25f;
        return linear_to_srgb(avg);
    } else if (InputInfo.isNormalMap) {
        float3 avg = (c0.xyz + c1.xyz + c2.xyz + c3.xyz) * 0.25f;
        avg = normalize(avg * 2.0f - 1.0f) * 0.5f + 0.5f;
        return float4(avg, 1.0f);
    } else if (InputInfo.minFilter) {
        return min(min(c0, c1), min(c2, c3));
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
    LDS_Color[Tid.y][Tid.x] = mip1Color;
    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 1 to MIP 2 ---
    // Only 1/4 of the threads (an 8x8 grid) continue to compute Mip 2
    if (Gidx < 64) {
        uint2 newTid = uint2(Gidx % 8, Gidx / 8);
        uint2 ldsSrc = newTid * 2;

        float4 m0 = LDS_Color[ldsSrc.y + 0][ldsSrc.x + 0];
        float4 m1 = LDS_Color[ldsSrc.y + 0][ldsSrc.x + 1];
        float4 m2 = LDS_Color[ldsSrc.y + 1][ldsSrc.x + 0];
        float4 m3 = LDS_Color[ldsSrc.y + 1][ldsSrc.x + 1];

        float4 mip2Color = Combine(m0, m1, m2, m3);
        uint2 outCoord = Gid.xy * 8 + newTid;
        
        if (all(outCoord < imageSize)) {
            OutputMips[base_mip + 2][outCoord] = mip2Color;
        }

        LDS_Color[ldsSrc.y][ldsSrc.x + newTid.y % 2] = mip2Color;  // stair-stepping offset on X to minimize memory bank conflicts
    }

    if (base_mip + 3 >= InputInfo.numMips) return;

    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 2 to MIP 3 ---
    // A 4x4 grid continues
    if (Gidx < 16) {
        uint2 newTid = uint2(Gidx % 4, Gidx / 4);
        uint2 ldsSrc = newTid * 4;
        //uint2 Delta = uint2((newTid.y * 2) % 2, ((newTid.y * 2) + 1) % 2);
        uint2 Delta = uint2(0, 1);  // stair-stepping offset on X to minimize memory bank conflicts

        float4 m0 = LDS_Color[ldsSrc.y + 0][ldsSrc.x + Delta.x + 0];
        float4 m1 = LDS_Color[ldsSrc.y + 0][ldsSrc.x + Delta.x + 2];
        float4 m2 = LDS_Color[ldsSrc.y + 2][ldsSrc.x + Delta.y + 0];
        float4 m3 = LDS_Color[ldsSrc.y + 2][ldsSrc.x + Delta.y + 2];

        float4 mip3Color = Combine(m0, m1, m2, m3);
        uint2 outCoord = Gid.xy * 4 + newTid;
        if (all(outCoord < imageSize)) {
            OutputMips[base_mip + 3][outCoord] = mip3Color;
        }

        LDS_Color[ldsSrc.y][ldsSrc.x + newTid.y % 4] = mip3Color;
    }

    if (base_mip + 4 >= InputInfo.numMips) return;

    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 3 to MIP 4 ---
    // A 2x2 grid continues
    if (Gidx < 4) {
        uint2 newTid = uint2(Gidx % 2, Gidx / 2);
        uint2 ldsSrc = newTid * 8;
        uint2 Delta = uint2((newTid.y * 2) % 4, ((newTid.y * 2) + 1) % 4);  // stair-stepping offset on X to minimize memory bank conflicts

        float4 m0 = LDS_Color[ldsSrc.y + 0][ldsSrc.x + Delta.x + 0];
        float4 m1 = LDS_Color[ldsSrc.y + 0][ldsSrc.x + Delta.x + 4];
        float4 m2 = LDS_Color[ldsSrc.y + 4][ldsSrc.x + Delta.y + 0];
        float4 m3 = LDS_Color[ldsSrc.y + 4][ldsSrc.x + Delta.y + 4];

        float4 mip4Color = Combine(m0, m1, m2, m3);
        uint2 outCoord = Gid.xy * 2 + newTid;
        if (all(outCoord < imageSize)) {
            OutputMips[base_mip + 4][outCoord] = mip4Color;
        }

        LDS_Color[ldsSrc.y][ldsSrc.x + newTid.y % 8] = mip4Color;
    }

    if (base_mip + 5 >= InputInfo.numMips) return;

    GroupMemoryBarrierWithGroupSync();

    imageSize = max(1, imageSize / 2);
    // --- MIP 4 to MIP 5 ---
    // Only 1 thread per group (the 16x16 block manager) reaches here
    if (Gidx == 0) {
        if (all(Gid.xy < imageSize)) {
            float4 m0 = LDS_Color[0][0];
            float4 m1 = LDS_Color[0][8];
            float4 m2 = LDS_Color[8][0 + 1];
            float4 m3 = LDS_Color[8][8 + 1];

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
    SurvivorFlag = 0;

    AllMemoryBarrierWithGroupSync();

    if (Gidx == 0) {
        uint currentGroupFinishedCount;
        GroupCounters.InterlockedAdd(0, 1, currentGroupFinishedCount);

        // If this is the last group to finish, it is the survivor!
        if (currentGroupFinishedCount == InputInfo.numGroups - 1) {
            // Reset counter for next frame
            GroupCounters.Store(0, 0);
            // Set value in shared memory indicating that this is a surviving group;
            SurvivorFlag = 1;
        }
    }

    GroupMemoryBarrierWithGroupSync();

    if (SurvivorFlag == 0) return;  // finish all groups except surviving group

    for (int base_mip = 5; base_mip < InputInfo.numMips - 1; base_mip += 5) {
        imageSize = max(1, imageSize >> 5);

        int max_i = (int)max(1, imageSize.y >> 1);
        int max_j = (int)max(1, imageSize.x >> 1);

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


//--------------------------------------------------------------------------
// Separate version for Hi-Z buffer construction with NPOT sizes handling
// Unfortunately, it is not a single pass solution :-(
// Computes 2 mip map levels
//--------------------------------------------------------------------------

groupshared float LDS_Zbuf[17][17];

float CombineMin(float c0, float c1, float c2, float c3) {
    return min(min(c0, c1), min(c2, c3));
}
float CombineMin(float c0, float c1, float c2) {
    return min(min(c0, c1), c2);
}
float CombineMin(float c0, float c1) {
    return min(c0, c1);
}

float getFirstMip(int base_mip, uint2 srcCoord, uint2 imageSize, int2 NeedExtraSample) {
    float p0 = OutputMips[base_mip][min(srcCoord + uint2(0, 0), imageSize - 1)].r;
    float p1 = OutputMips[base_mip][min(srcCoord + uint2(1, 0), imageSize - 1)].r;
    float p2 = OutputMips[base_mip][min(srcCoord + uint2(0, 1), imageSize - 1)].r;
    float p3 = OutputMips[base_mip][min(srcCoord + uint2(1, 1), imageSize - 1)].r;

    float mip1Color = CombineMin(p0, p1, p2, p3);

    if (NeedExtraSample.x) {
        p0 = OutputMips[base_mip][min(srcCoord + uint2(2, 0), imageSize - 1)].r;
        p1 = OutputMips[base_mip][min(srcCoord + uint2(2, 1), imageSize - 1)].r;
        mip1Color = CombineMin(mip1Color, p0, p1);
    }
    if (NeedExtraSample.y) {
        p0 = OutputMips[base_mip][min(srcCoord + uint2(0, 2), imageSize - 1)].r;
        p1 = OutputMips[base_mip][min(srcCoord + uint2(1, 2), imageSize - 1)].r;
        mip1Color = CombineMin(mip1Color, p0, p1);
    }
    if (NeedExtraSample.x && NeedExtraSample.y) {
        p0 = OutputMips[base_mip][min(srcCoord + uint2(2, 2), imageSize - 1)].r;
        mip1Color = CombineMin(mip1Color, p0);
    }
    return mip1Color;
}

// Assuming 256 threads per group (16x16)
[numthreads(16, 16, 1)] 
void CS_MinFilter(uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) 
{
    int base_mip = InputInfo.baseMip;

    uint2 imageSize0;
    OutputMips[base_mip].GetDimensions(imageSize0.x, imageSize0.y);

    if (base_mip + 1 >= InputInfo.numMips) return;

    bool2 NeedExtraSample1 = imageSize0 % 2;
    uint2 imageSize1 = max(1, imageSize0 / 2);
    bool2 NeedExtraSample2 = imageSize1 % 2;
    uint2 imageSize2 = max(1, imageSize1 / 2);

    bool2 NeedExtraSampleAny = or(NeedExtraSample1, NeedExtraSample2);

    // --- MIP 0 to MIP 1 ---
    // Load a 2x2 quad from the Source Texture (Mip 0)
    float mip1Color = 1.0f;

    if (all(DTid.xy < imageSize1)) {
        uint2 srcCoord = DTid.xy * 2;
        mip1Color = getFirstMip(base_mip, srcCoord, imageSize0, NeedExtraSample1);
        // Write Mip 1 out to global memory
        OutputMips[base_mip + 1][DTid.xy] = mip1Color;
    }

    if (base_mip + 2 >= InputInfo.numMips) return;

    // Store in Local Data Store (LDS) for the next step
    LDS_Zbuf[Tid.y][Tid.x] = mip1Color;

    if (Tid.x == 15 && NeedExtraSampleAny.x) {
        uint2 srcCoord = DTid.xy * 2 + uint2(2, 0);
        mip1Color = getFirstMip(base_mip, srcCoord, imageSize0, NeedExtraSample1);
        LDS_Zbuf[Tid.y][16] = mip1Color;
    }
    if (Tid.y == 15 && NeedExtraSampleAny.y) {
        uint2 srcCoord = DTid.xy * 2 + uint2(0, 2);
        mip1Color = getFirstMip(base_mip, srcCoord, imageSize0, NeedExtraSample1);
        LDS_Zbuf[16][Tid.x] = mip1Color;
    }
    if (Tid.x == 15 && Tid.y == 15 && NeedExtraSampleAny.x && NeedExtraSampleAny.y) {
        uint2 srcCoord = DTid.xy * 2 + uint2(2, 2);
        mip1Color = getFirstMip(base_mip, srcCoord, imageSize0, NeedExtraSample1);
        LDS_Zbuf[16][16] = mip1Color;
    }
    GroupMemoryBarrierWithGroupSync();

    // --- MIP 1 to MIP 2 ---
    // Only 1/4 of the threads (an 8x8 grid) continue to compute Mip 2
    if (Gidx < 64) {
        uint2 newTid = uint2(Gidx % 8, Gidx / 8);
        uint2 ldsSrc = newTid * 2;

        float m0 = LDS_Zbuf[ldsSrc.y + 0][ldsSrc.x + 0];
        float m1 = LDS_Zbuf[ldsSrc.y + 0][ldsSrc.x + 1];
        float m2 = LDS_Zbuf[ldsSrc.y + 1][ldsSrc.x + 0];
        float m3 = LDS_Zbuf[ldsSrc.y + 1][ldsSrc.x + 1];

        float mip2Color = CombineMin(m0, m1, m2, m3);

        if (NeedExtraSample2.x) {
            m0 = LDS_Zbuf[ldsSrc.y + 0][ldsSrc.x + 2];
            m1 = LDS_Zbuf[ldsSrc.y + 1][ldsSrc.x + 2];
            mip2Color = CombineMin(mip2Color, m0, m1);
        }
        if (NeedExtraSample2.y) {
            m0 = LDS_Zbuf[ldsSrc.y + 2][ldsSrc.x + 0];
            m1 = LDS_Zbuf[ldsSrc.y + 2][ldsSrc.x + 1];
            mip2Color = CombineMin(mip2Color, m0, m1);
        }
        if (NeedExtraSample2.x && NeedExtraSample2.y) {
            m0 = LDS_Zbuf[ldsSrc.y + 2][ldsSrc.x + 2];
            mip2Color = CombineMin(mip2Color, m0);
        }

        uint2 outCoord = Gid.xy * 8 + newTid;

        if (all(outCoord < imageSize2)) {
            OutputMips[base_mip + 2][outCoord] = mip2Color;
        }
    }
}