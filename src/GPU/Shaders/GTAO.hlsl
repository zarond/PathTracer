#include "BRDF.hlsl"

struct GTAOCSInput {
    float4x4 Projection;
    float4x4 invProjection;
    uint2 FrameSize;
    float2 texel_size;
    float AO_distance;
    unsigned int frameID;
};

// Gbuff
Texture2D<float4> Gbuffer : register(t0, space0);

// Depth
Texture2D<float> Depth : register(t1, space0);

// Output
RWTexture2D<float4> Output : register(u0, space0);

// Constant Buffer
ConstantBuffer<GTAOCSInput> g_CB : register(b0);

// Default sampler
SamplerState Sampler : register(s0, space0);

float3 ReconstructViewPosition(float2 uv, float depth) {
    // depth assumed non-linear [0,1] as sampled from Depth texture
    // Build clip-space position and multiply by inverse projection to get view-space position
    float2 ndcXY = uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f);
    float4 clip = float4(ndcXY, depth, 1.0f);
    float4 viewH = mul(g_CB.invProjection, clip);
    return viewH.xyz / viewH.w;
}

float3 ViewPositionToUV(float3 pos) {
    float4 clip = mul(g_CB.Projection, float4(pos, 1.0f));
    clip.xyz /= clip.w;
    return float3(clip.xy * float2(0.5f, -0.5f) + 0.5f, clip.z);
}

float cosineWeightingIntegral(float min_angle, float max_angle, float normal_angle) {
    return 0.25f * (-cos(2 * max_angle - normal_angle) + cos(normal_angle) + 2 * max_angle * sin(normal_angle) +
                    -cos(2 * min_angle - normal_angle) + cos(normal_angle) + 2 * min_angle * sin(normal_angle));
}

float SpatialDirectionsNoise(uint3 DTid) {
    return (1.0 / 16.0) * (((( DTid.x + DTid.y) & 0x3) << 2) + (DTid.x & 0x3));
}

float SpatialOffsetsNoise(uint3 DTid) {
    return (1.0 / 4.0) * ((DTid.y - DTid.x) & 0x3);
}

float TemporalDirections() { 
    float rotations[] = {60, 300, 180, 240, 120, 0};
    float rotation = rotations[g_CB.frameID % 6] / 360.0f;
    return rotation;
}

float TemporalOffsets() { 
    float offsets[] = {0, 0.5, 0.25, 0.75};
    float offset = offsets[(g_CB.frameID / 6) % 4];
    return offset;
}

[numthreads(16, 16, 1)] 
void CS_GTAO(uint3 DTid : SV_DispatchThreadID) 
{
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;

    const float AO_distance = g_CB.AO_distance;
    const int NUM_DIRECTIONS = 4;
    const int STEPS_PER_DIR = 32;

    float2 texel = g_CB.texel_size;
    float2 uv = (DTid.xy + 0.5f) * texel;

    float centerDepth = Depth.Load(DTid);
    if (centerDepth >= 1.0f || centerDepth <= 0.0f) {
        Output[DTid.xy] = float4(1.0f, 1.0f, 1.0f, 1.0f);
        return;
    }

    float3 normal = Gbuffer.Load(DTid).xyz;
    float3 viewPos = ReconstructViewPosition(uv, centerDepth);
    float3 viewDir = normalize(-viewPos);

    float3 ls_X = float3(1, 0, 0);
    ls_X = normalize(ls_X - viewDir * dot(viewDir, ls_X));  // orthogonalize
    float3 ls_Y = cross(viewDir, ls_X);

    float spatial_directions_noise = frac(SpatialDirectionsNoise(DTid) + TemporalDirections());
    float spatial_offsets_noise = frac(SpatialOffsetsNoise(DTid) + TemporalOffsets());

    float occlusion = 0.0f;
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
    {
        float rot = PI * (float(i) + spatial_directions_noise) / NUM_DIRECTIONS;
        float cos_r = cos(rot);
        float sin_r = sin(rot);
        float3 rayViewDir = cos_r * ls_X + sin_r * ls_Y;
        
        float2 projected_normal = float2(dot(normal, rayViewDir), dot(normal, viewDir));
        float projected_normal_length = length(projected_normal);
        float2 projected_normal_normalized = clamp(projected_normal / projected_normal_length, -1.0f, 1.0f);
        float normal_angle = (-sign(projected_normal_normalized.x)) * acos(projected_normal_normalized.y);
        
        float2 rayUVDir = ViewPositionToUV(viewPos + rayViewDir * 0.01f) - uv;
        float2 rayScreenDir = AO_distance * 100.0f * rayUVDir * g_CB.FrameSize;

        float rayScreenDirLength = length(rayScreenDir);
        float2 rayScreenDirNorm = rayScreenDir / rayScreenDirLength;

        float min_angle;
        float max_angle;

        for (int sign = -1; sign <= 1; sign += 2) {
            float step_size = 0.5f;
            float DepthLod = -1.0f;
            float acc_angle_cos = -1;
            float t_offset = 0.0f;
            for (int t = 0; t < STEPS_PER_DIR; ++t)
            {
                if (t % 4 == 0 && step_size < 32.0f) {
                    step_size *= 2.0f;
                    DepthLod += 1.0f;
                }
                t_offset += step_size;
                float t_offset_with_noise = t_offset + spatial_offsets_noise * step_size;
                if (t_offset_with_noise > rayScreenDirLength) break;
                float2 sampleUV = uv + sign * rayScreenDirNorm * t_offset_with_noise * texel;
                if (any(sampleUV < 0.0f) || any(sampleUV > 1.0f)) break;
                float sampleDepth = Depth.SampleLevel(Sampler, sampleUV, DepthLod);
                float3 samplePos = ReconstructViewPosition(sampleUV, sampleDepth);
                float3 sampleDir = samplePos - viewPos;

                float sampleDirLength = length(sampleDir);

                float Dist_fac = sampleDirLength / AO_distance;
                Dist_fac = pow2(saturate(Dist_fac));
                float angle_cos = dot(sampleDir, viewDir) / sampleDirLength;
                acc_angle_cos = lerp(max(angle_cos, acc_angle_cos), acc_angle_cos, Dist_fac);
            }
            float angle = acos(acc_angle_cos);
            if (sign < 0) {
                max_angle = angle;
            } else {
                min_angle = -angle;
            }
        }
        min_angle = max(min_angle, normal_angle - PI / 2);
        max_angle = min(max_angle, normal_angle + PI / 2);

        occlusion += projected_normal_length * cosineWeightingIntegral(min_angle, max_angle, normal_angle);
    }

    float ao = saturate(occlusion / (float)NUM_DIRECTIONS);
    Output[DTid.xy] = float4(ao, ao, ao, 1.0f);
}