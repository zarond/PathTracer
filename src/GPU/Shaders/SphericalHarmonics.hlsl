static const float PI = 3.14159265359f;

struct SHCSInput {
    uint2 FrameSize;
    float2 inv_FrameSize;
    uint2 GroupsSize;
    uint numGroups;
};

Texture2D<float4> SrcTexture : register(t0);
globallycoherent RWStructuredBuffer<float4> TmpBuffer : register(u0);

RWByteAddressBuffer GroupCounters : register(u0, space1);

ConstantBuffer<SHCSInput> g_CB : register(b0);

groupshared float4 sharedArray[9][256 / 32];     // Warning: assuming fixed wavesize of 32
groupshared uint SurvivorFlag;

void evalSH9(float3 d, out float sh[9]) {
    const float x = d.x;
    const float y = d.y;
    const float z = d.z;

    sh[0] = 0.2820947918;   // L00

    sh[1] = 0.4886025119 * y;   // L1_1
    sh[2] = 0.4886025119 * z;   // L10
    sh[3] = 0.4886025119 * x;   // L11

    sh[4] = 1.0925484306 * x * y;   // L2_2
    sh[5] = 1.0925484306 * y * z;   // L2_1
    sh[6] = 0.3153915653 * (3.0 * z * z - 1.0); // L20
    sh[7] = 1.0925484306 * x * z;   // L21
    sh[8] = 0.5462742153 * (x * x - y * y); // L22
}

[WaveSize(32)]  // Warning: fixed wavesize
[numthreads(16, 16, 1)]
void CS_SphericalHarmonicsIrradiance(uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint Gidx : SV_GroupIndex)
{
    bool use_thread = true;
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) use_thread = false;
    uint2 coord = clamp(DTid.xy, 0, g_CB.FrameSize - 1);
    float3 L = SrcTexture.Load(int3(coord, 0)).rgb;
    
    float d_phi = 2 * PI * g_CB.inv_FrameSize.x;
    float d_theta = PI * g_CB.inv_FrameSize.y;
    
    float phi = (coord.x + 0.5f) * d_phi;
    float theta = (coord.y + 0.5f) * d_theta;
    
    float sin_phi, cos_phi;
    sincos(phi, sin_phi, cos_phi);
    float sin_theta, cos_theta;
    sincos(theta, sin_theta, cos_theta);
    
    float3 w = float3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
    float weight = use_thread ? d_phi * d_theta * sin_theta : 0;
    
    float sh[9];
    evalSH9(w, sh);
    
    float3 L_weight = L * weight;
    
    const uint lane = WaveGetLaneIndex();
    const uint waveSize = WaveGetLaneCount();
    const uint waveCount = 256 / waveSize; // Assuming 256 threads per group, so 8 waves of 32 threads each
    
    [unroll]
    for (int i = 0; i < 9; ++i) {
        float3 contribution = L_weight * sh[i];
        float3 contribution_sum_wave = WaveActiveSum( contribution );
        if (lane == 0) {
            sharedArray[i][Gidx / waveSize] = float4(contribution_sum_wave, 0.0f);
        }
    }
    
    GroupMemoryBarrierWithGroupSync();
    
    [unroll]
    if (Gidx < waveCount) {
        for (int i = 0; i < 9; ++i) {
            float3 finalWaveVal = sharedArray[i][Gidx].rgb;
            float3 totalGroupSum = WaveActiveSum(finalWaveVal); // Sum across the whole group because we only have 8 (256 / 32) elements

            if (Gidx == 0) {
                uint Gid_lin = Gid.x + Gid.y * g_CB.GroupsSize.x;
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
            sharedArray[k][Gidx / waveSize] = float4(value_sum_wave, 0.0f);
        }
    }
    
    AllMemoryBarrierWithGroupSync();
    
    [unroll]
    if (Gidx < waveCount) {
        for (int i = 0; i < 9; ++i) {
            float3 finalWaveVal = sharedArray[i][Gidx].rgb;
            float3 totalGroupSum = WaveActiveSum(finalWaveVal); // Sum across the whole group because we only have 8 (256 / 32) elements

            if (Gidx == 0) {
                TmpBuffer[i] = float4(totalGroupSum, 0.0f);
                //result[i] = totalGroupSum;
            }
        }
    }
}
