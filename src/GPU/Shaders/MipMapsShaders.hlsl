
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

// Assuming 256 threads per group (16x16)
[numthreads(16, 16, 1)] void CS_SinglePassMips(
    uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) {
   
    int baseMip = InputInfo.startingMip;
    
    if (InputInfo.numMips < baseMip + 2) return;

    // --- MIP 0 to MIP 1 ---
    // Load a 2x2 quad from the Source Texture (Mip 0)
    uint2 srcCoord = DTid.xy * 2;
    float4 p0 = OutputMips[baseMip][srcCoord + uint2(0, 0)];
    float4 p1 = OutputMips[baseMip][srcCoord + uint2(1, 0)];
    float4 p2 = OutputMips[baseMip][srcCoord + uint2(0, 1)];
    float4 p3 = OutputMips[baseMip][srcCoord + uint2(1, 1)];

    float4 mip1Color = (p0 + p1 + p2 + p3) * 0.25f;

    // Write Mip 1 out to global memory
    OutputMips[baseMip + 1][DTid.xy] = mip1Color;

    if (InputInfo.numMips < baseMip + 3) return; 

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

        float4 mip2Color = (m0 + m1 + m2 + m3) * 0.25f;
        OutputMips[baseMip + 2][DTid.xy / 2] = mip2Color;

        // Overwrite the LDS layout compaction
        LDS_Color[Tid.x][Tid.y] = mip2Color;
    }

    if (InputInfo.numMips < baseMip + 4) return; 

    GroupMemoryBarrierWithGroupSync();

    // --- MIP 2 to MIP 3 ---
    // A 4x4 grid continues
    if ((Tid.x % 4 == 0) && (Tid.y % 4 == 0)) {
        float4 m0 = LDS_Color[Tid.x + 0][Tid.y + 0];
        float4 m1 = LDS_Color[Tid.x + 2][Tid.y + 0];
        float4 m2 = LDS_Color[Tid.x + 0][Tid.y + 2];
        float4 m3 = LDS_Color[Tid.x + 2][Tid.y + 2];

        float4 mip3Color = (m0 + m1 + m2 + m3) * 0.25f;
        OutputMips[baseMip + 3][DTid.xy / 4] = mip3Color;

        LDS_Color[Tid.x][Tid.y] = mip3Color;
    }

    if (InputInfo.numMips < baseMip + 5) return;

    GroupMemoryBarrierWithGroupSync();

    // --- MIP 3 to MIP 4 ---
    // A 2x2 grid continues
    if ((Tid.x % 8 == 0) && (Tid.y % 8 == 0)) {
        float4 m0 = LDS_Color[Tid.x + 0][Tid.y + 0];
        float4 m1 = LDS_Color[Tid.x + 4][Tid.y + 0];
        float4 m2 = LDS_Color[Tid.x + 0][Tid.y + 4];
        float4 m3 = LDS_Color[Tid.x + 4][Tid.y + 4];

        float4 mip4Color = (m0 + m1 + m2 + m3) * 0.25f;
        OutputMips[baseMip + 4][DTid.xy / 8] = mip4Color;

        LDS_Color[Tid.x][Tid.y] = mip4Color;
    }

    if (InputInfo.numMips < baseMip + 6) return;

    GroupMemoryBarrierWithGroupSync();

    // --- MIP 4 to MIP 5 ---
    // Only 1 thread per group (the 16x16 block manager) reaches here
    if (Gidx == 0) {
        float4 m0 = LDS_Color[0][0];
        float4 m1 = LDS_Color[8][0];
        float4 m2 = LDS_Color[0][8];
        float4 m3 = LDS_Color[8][8];

        float4 mip5Color = (m0 + m1 + m2 + m3) * 0.25f;
        OutputMips[baseMip + 5][Gid.xy] = mip5Color;
    }
}