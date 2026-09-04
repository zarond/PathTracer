static const float PI = 3.14159265359f;

struct SHCSInput {
    uint2 FrameSize;
    float2 inv_FrameSize;
    uint2 GroupsSize;
    uint numGroups;
    uint useUVcoords;
    uint isCubemap;
};

Texture2D<float4> SrcTexture : register(t0);
TextureCube<float4> SrcTextureCube : register(t1);
globallycoherent RWStructuredBuffer<float4> TmpBuffer : register(u0);

RWByteAddressBuffer GroupCounters : register(u0, space1);

ConstantBuffer<SHCSInput> g_CB : register(b0);

// Default sampler
SamplerState Sampler : register(s0, space0);

groupshared float4 sharedArray[9][256 / 16];     // Warning: assuming minimum wavesize of 16, recommended is 32
groupshared uint SurvivorFlag;

void evalSH9(float3 d, out float sh[9]) {
    const float x = d.x;
    const float y = d.y;
    const float z = d.z;

    sh[0] = 0.2820947918;   // L00

    sh[1] = 0.4886025119 * z;   // L1_1
    sh[2] = 0.4886025119 * y;   // L10
    sh[3] = 0.4886025119 * x;   // L11

    sh[4] = 1.0925484306 * x * z;   // L2_2
    sh[5] = 1.0925484306 * y * z;   // L2_1
    sh[6] = 0.3153915653 * (3.0 * y * y - 1.0); // L20
    sh[7] = 1.0925484306 * x * y;   // L21
    sh[8] = 0.5462742153 * (x * x - z * z); // L22
}

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

// Helper function for the analytical solid angle calculation
float AreaElement(float x, float y) {
    return atan2(x * y, sqrt(x * x + y * y + 1.0));
}

float TexelSolidAngle(float2 ndc) { // ndc is uv in a range from -1 to 1
    // Find the four corners of the texel in texture space
    float x0 = ndc.x - g_CB.inv_FrameSize.x;
    float y0 = ndc.y - g_CB.inv_FrameSize.y;
    float x1 = ndc.x + g_CB.inv_FrameSize.x;
    float y1 = ndc.y + g_CB.inv_FrameSize.y;
    
    // Integrate solid angle over the texel boundaries
    return AreaElement(x0, y0) - AreaElement(x0, y1) - AreaElement(x1, y0) + AreaElement(x1, y1);
}

[numthreads(16, 16, 1)]
void CS_SphericalHarmonicsIrradiance(uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint Gidx : SV_GroupIndex)
{
    bool use_thread = true;
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) use_thread = false;
    uint2 coord = clamp(DTid.xy, 0, g_CB.FrameSize - 1);
    float3 L;
    float3 cubemap_dir;
    float2 uv = (coord + 0.5f) * g_CB.inv_FrameSize;
    uv.y = 1.0 - uv.y;
    if (g_CB.isCubemap) {
        uv = uv * 2.0f - 1.0f;
        cubemap_dir = CalculateCubeDirection(uv, DTid.z);
        L = SrcTextureCube.SampleLevel(Sampler, cubemap_dir, 0);
    } else if (g_CB.useUVcoords) { // for ultra low resolution envmaps
        L = SrcTexture.SampleLevel(Sampler, uv, 0);
    } else {
        L = SrcTexture.Load(int3(coord, 0)).rgb;
    }
    
    float3 w;
    float weight;

    if (g_CB.isCubemap) {
        w = cubemap_dir;
        float dOmega = TexelSolidAngle(uv);
        weight = use_thread ? dOmega : 0;
    } else {
        float d_phi = 2 * PI * g_CB.inv_FrameSize.x;
        float d_theta = PI * g_CB.inv_FrameSize.y;

        float phi = (coord.x + 0.5f) * d_phi;
        float theta = (coord.y + 0.5f) * d_theta;

        float sin_phi, cos_phi;
        sincos(phi, sin_phi, cos_phi);
        float sin_theta, cos_theta;
        sincos(theta, sin_theta, cos_theta);
        w = float3(sin_theta * cos_phi, cos_theta, sin_theta * sin_phi);
        weight = use_thread ? d_phi * d_theta * sin_theta : 0;
    }
    
    float sh[9];
    evalSH9(w, sh);
    
    float3 L_weight = L * weight;
    
    const uint lane = WaveGetLaneIndex();
    const uint waveSize = WaveGetLaneCount();
    const uint waveCount = 256 / waveSize; // 256 threads per group, so 8 waves of 32 threads each, or 16 waves of 16 each
    const uint waveIdx = Gidx / waveSize;
    
    [unroll]
    for (int i = 0; i < 9; ++i) {
        float3 contribution = L_weight * sh[i];
        float3 contribution_sum_wave = WaveActiveSum( contribution );
        if (lane == 0) {
            sharedArray[i][waveIdx] = float4(contribution_sum_wave, 0.0f);
        }
    }
    
    GroupMemoryBarrierWithGroupSync();
    
    if (Gidx < waveCount) {
        [unroll]
        for (int i = 0; i < 9; ++i) {
            float3 finalWaveVal = sharedArray[i][Gidx].rgb;
            float3 totalGroupSum = WaveActiveSum(finalWaveVal); // Sum across the whole group because we only have 8 (256 / 32) elements, or 16 (256 / 16)

            if (Gidx == 0) {
                uint Gid_lin = Gid.x + Gid.y * g_CB.GroupsSize.x + Gid.z * g_CB.GroupsSize.x * g_CB.GroupsSize.y;
                TmpBuffer[9 * Gid_lin + i] = float4(totalGroupSum, 0.0f);
            }
        }
    }

    SurvivorFlag = 0;
    
    AllMemoryBarrierWithGroupSync();
    
    if (Gidx == 0) {
        uint currentGroupFinishedCount;
        GroupCounters.InterlockedAdd(0, 1, currentGroupFinishedCount);

        // If this is the last group to finish, it is the survivor!
        if (currentGroupFinishedCount == g_CB.numGroups - 1) {
            // Reset counter for next frame
            GroupCounters.Store(0, 0);
            // Set value in shared memory indicating that this is a surviving group;
            SurvivorFlag = 1;
        }
    }

    GroupMemoryBarrierWithGroupSync();

    if (SurvivorFlag == 0) return;  // finish all groups except surviving group
    
    for (int k = 0; k < 9; ++k) {
        float3 value = 0.0f;
        for (int i = Gidx; i < g_CB.numGroups; i += 256) {
            value += TmpBuffer[9 * i + k].rgb;
        }

        float3 value_sum_wave = WaveActiveSum( value );
        if (lane == 0) {
            sharedArray[k][waveIdx] = float4(value_sum_wave, 0.0f);
        }
    }
    
    AllMemoryBarrierWithGroupSync();
    
    if (Gidx < waveCount) {
        [unroll]
        for (int i = 0; i < 9; ++i) {
            float3 finalWaveVal = sharedArray[i][Gidx].rgb;
            float3 totalGroupSum = WaveActiveSum(finalWaveVal); // Sum across the whole group because we only have 8 (256 / 32) elements, or 16 (256 / 16)

            if (Gidx == 0) {
                TmpBuffer[i] = float4(totalGroupSum, 0.0f);
            }
        }
    }
}
