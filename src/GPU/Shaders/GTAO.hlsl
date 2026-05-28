#include "BRDF.hlsl"

struct GTAOCSInput {
    float4x4 projectionToWorld;  // without translation component
    float4x4 viewProjection;
    float4 cameraPosition;
    int2 FrameSize;
    float2 texel_size;
    float thin_object_factor;
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
    float4 viewH = mul(g_CB.projectionToWorld, clip);

    return viewH.xyz / viewH.w;
}

float cosineWeightingIntegral(float min_angle, float max_angle, float normal_angle) {
    return 0.25f * (-cos(2 * max_angle - normal_angle) + cos(normal_angle) + 2 * max_angle * sin(normal_angle) +
                    -cos(2 * min_angle - normal_angle) + cos(normal_angle) + 2 * min_angle * sin(normal_angle));
}

[numthreads(16, 16, 1)] 
void CS_GTAO(
    uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) 
{
uint2 imageSize;
    Output.GetDimensions(imageSize.x, imageSize.y);

    if (DTid.x >= imageSize.x || DTid.y >= imageSize.y) return;

    const float AO_distance = 4.0f;
    const int NUM_DIRECTIONS = 32;
    const int STEPS_PER_DIR = 32;
    const float THIN_OBJ_HEURISTIC = g_CB.thin_object_factor;

    float2 texel = g_CB.texel_size;
    float2 uv = (DTid.xy + 0.5f) * texel;

    float centerDepth = Depth.Load(DTid);
    if (centerDepth >= 1.0f || centerDepth <= 0.0f) { // Guard both depth extremes
        Output[DTid.xy] = float4(1.0f, 1.0f, 1.0f, 1.0f);
        return;
    }

    float3 normal = Gbuffer.Load(DTid).xyz;
    float3 viewPos = ReconstructViewPosition(uv, centerDepth);
    float3 viewDir = normalize(-viewPos);

    float occlusion = 0.0f;
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
    {
        float rot = PI * float(i) / NUM_DIRECTIONS;
        float cos_r = cos(rot);
        float sin_r = sin(rot);
        //const float2x2 rotation_matrix = float2x2(cos_r, -sin_r, sin_r, cos_r);
        float2 sampleDir = float2(cos_r, sin_r);
        float3 slice_point = ReconstructViewPosition(uv + sampleDir * texel, centerDepth);
        float3 sliceDir = slice_point - viewPos;
        sliceDir = normalize(sliceDir - viewDir * dot(viewDir, sliceDir)); // orthogonalize
        
        float2 projected_normal = float2(dot(normal, sliceDir), dot(normal, viewDir));
        float projected_normal_length = length(projected_normal);
        float2 projected_normal_normalized = clamp(projected_normal / projected_normal_length, -1.0f, 1.0f);
        float normal_angle = (-sign(projected_normal_normalized.x)) * acos(projected_normal_normalized.y);

        float min_angle;
        float max_angle;

        for (int sign = -1; sign <= 1; sign += 2) {
            float acc_angle_cos = -1;
            float angle_cos_prev = 0;
            for (int t = 1; t <= STEPS_PER_DIR; ++t)
            {
                float2 sampleUV = uv + sampleDir * texel * sign * t * AO_distance;
                if (any(sampleUV < 0.0f) || any(sampleUV > 1.0f)) break;
                float sampleDepth = Depth.SampleLevel(Sampler, sampleUV, 0);
                float3 samplePos = ReconstructViewPosition(sampleUV, sampleDepth);
                float3 sampleDir = normalize(samplePos - viewPos);

                float angle_cos = dot(sampleDir, viewDir);
                if (angle_cos >= angle_cos_prev) {
                    acc_angle_cos = max(acc_angle_cos, angle_cos);
                } else {
                    acc_angle_cos = lerp(acc_angle_cos, angle_cos, THIN_OBJ_HEURISTIC);
                }
                angle_cos_prev = angle_cos;
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