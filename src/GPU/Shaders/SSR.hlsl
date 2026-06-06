#include "BRDF.hlsl"
#include "Common.hlsl"

struct SSRCSInput {
    float4x4 Projection;
    float inv_proj_00;
    float inv_proj_11;
    uint2 FrameSize;
    float2 texel_size;
    float DepthThreshold;
    float MaxRoughness;
    float GGXBias;  // 1.0 is no bias, < 1.0 reduces tail of distribution
    int frameID;
    int MaxDepthMipLevel;
};

// Gbuff
Texture2D<float4> Gbuffer : register(t0, space0);

// Depth
Texture2D<float> Depth : register(t1, space0);

// Frame
Texture2D<float4> Frame : register(t2, space0);

// Output
RWTexture2D<float4> Output : register(u0, space0);

// Constant Buffer
ConstantBuffer<SSRCSInput> g_CB : register(b0);

// Point sampler
SamplerState PointSampler : register(s0, space0);

// Default Linear sampler
SamplerState LinearSampler : register(s1, space0);

float3 ReconstructViewPosition(float2 uv, float depth) {
    // depth assumed non-linear [0,1] as sampled from Depth texture
    // Build clip-space position and multiply by inverse projection to get view-space position
    float2 ndcXY = uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f);
    // float4 clip = float4(ndcXY, depth, 1.0f);
    // float4 viewH = mul(g_CB.invProjection, clip);
    // return viewH.xyz / viewH.w;

    // Same as above, but optimized for standard perspective projection matrix
    const float inv_proj_00 = g_CB.inv_proj_00;
    const float inv_proj_11 = g_CB.inv_proj_11;
    const float proj_22 = g_CB.Projection[2][2];
    const float proj_23 = g_CB.Projection[2][3];

    const float z_vs = -proj_23 / (proj_22 + depth);
    const float x_vs = -z_vs * ndcXY.x * inv_proj_00;
    const float y_vs = -z_vs * ndcXY.y * inv_proj_11;

    return float3(x_vs, y_vs, z_vs);
}

float3 ViewPositionToUV(float3 pos) {
    //float4 clip = mul(g_CB.Projection, float4(pos, 1.0f));
    //clip.xyz /= clip.w;
    //return float3(clip.xy * float2(0.5f, -0.5f) + 0.5f, clip.z);

    // Same as above, but optimized for standard perspective projection matrix
    const float proj_00 = g_CB.Projection[0][0];
    const float proj_11 = g_CB.Projection[1][1];
    const float proj_22 = g_CB.Projection[2][2];
    const float proj_23 = g_CB.Projection[2][3];
    float inv_z = 1.0f / pos.z;
    float x_ndc = -pos.x * proj_00 * inv_z;
    float y_ndc = -pos.y * proj_11 * inv_z;
    float z_ndc = -proj_22 - proj_23 * inv_z;
    float3 ndc = float3(x_ndc, y_ndc, z_ndc);
    return float3(ndc.xy * float2(0.5f, -0.5f) + 0.5f, ndc.z);
}

float linear_depth(float non_linear_depth) {
    // Assuming standard perspective projection matrix, reconstruct linear depth from non-linear depth
    const float proj_22 = g_CB.Projection[2][2];
    const float proj_23 = g_CB.Projection[2][3];
    return -proj_23 / (proj_22 + non_linear_depth);
}

float precise_ray_depth(float3 sampleUV, float sampleDepth) { 
    return (linear_depth(sampleDepth) - linear_depth(sampleUV.z));
}

float UVEdgeFade(float2 uv) {
    float2 edge_dist = min(uv, 1.0f - uv);
    float edge_fade = smoothstep(0.0f, 0.05f, min(edge_dist.x, edge_dist.y));
    return edge_fade;
}

float ThresholdFade(float value, float threshold, float mult_range, float lin_range) {
    return smoothstep(threshold, threshold * mult_range - lin_range, value);
}

bool IsPotentialHit(float3 sampleUV, float sampleDepth) { return sampleUV.z > sampleDepth; }

bool CheckPotentialHit(float3 sampleDepth, float3 prev_sampleDepth,
    inout float3 sampleUV, float3 prev_sampleUV, float3 l, inout float hit_confidence ) 
{
    const float3 hit_normals = Gbuffer.SampleLevel(PointSampler, sampleUV, 0).xyz;
    const float LoN = dot(l, hit_normals);
    hit_confidence = min(hit_confidence, ThresholdFade(LoN, 0.005f, 0, 0.005f));
    if (hit_confidence == 0.0f) {
        return false;
    }
    float d2 = precise_ray_depth(sampleUV, sampleDepth);
    hit_confidence = min(hit_confidence, ThresholdFade(d2, g_CB.DepthThreshold, 0.9f, 0));
    if (hit_confidence == 0.0f) {
        return false;
    }
    float d1 = -precise_ray_depth(prev_sampleUV, prev_sampleDepth);
    sampleUV = lerp(prev_sampleUV, sampleUV, d1 / (d1 + d2));

    return true;
}

[numthreads(16, 16, 1)] 
void CS_SSR(uint3 DTid : SV_DispatchThreadID) {
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;

    const int STEPS = 512;

    const float2 texel = g_CB.texel_size;
    const float2 uv = (DTid.xy + 0.5f) * texel;

    const float centerDepth = Depth.Load(DTid);
    if (centerDepth >= 1.0f || centerDepth <= 0.0f) {
        Output[DTid.xy] = float4(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    const float4 GbufferData = Gbuffer.Load(DTid);
    const float3 normal = GbufferData.xyz;
    const float roughness = clamp(GbufferData.w, 0.002f, g_CB.MaxRoughness);
    const float linear_roughness = roughness * roughness;
    const float3 pos = ReconstructViewPosition(uv, centerDepth);
    const float3 v = normalize(-pos);

    float3 tangent = normal.x < 0.9f ? float3(1, 0, 0) : float3(0, 1, 0);
    tangent = normalize(tangent - normal * dot(normal, tangent));
    const float3 bitangent = cross(normal, tangent);
    float3x3 TBN = float3x3(tangent, bitangent, normal);

    //-------------------------------------------------------
    bool found_hit = false;
    float hit_confidence = 1.0f;

    uint3 seed = uint3(DTid.xy, g_CB.frameID);
    float3 jitter = pcg3d(seed);
    float2 rand = fibonacci2D(0, 1.0f / 8);
    rand = fmod(rand + jitter.xy, 1.0f);

    // Cut tails of distribution to reduce noise (introduces bias)
    rand.x *= g_CB.GGXBias;

    float3 h = importanceSampleGGX(rand, linear_roughness);
    h = Tangent2World(h, TBN);

    const float3 l = reflect(-v, h);

    const float LoN = saturate(dot(l, normal));
    if (LoN == 0.0f) {
        //Output[DTid.xy] = float4(1.0f, 0.0f, 0.0f, 1.0f); // todo: IMPORTANT, solve this problem
        //return;
    }

    const float3 rayUVDir = ViewPositionToUV(pos + l * 0.01f) - float3(uv, centerDepth);
    const float3 rayScreenDir = rayUVDir * float3(g_CB.FrameSize, 1.0f);

    const float rayScreenDirLength = length(rayScreenDir.xy);
    const float3 rayScreenDirNorm = rayScreenDir / rayScreenDirLength;

    const float LoV = dot(l, v);

    float3 prev_sampleUV = float3(uv, centerDepth);
    float3 sampleUV = prev_sampleUV + rayScreenDirNorm * float3(texel, 1.0f) * jitter.z;
    float sampleDepth = centerDepth;
    float prev_sampleDepth = centerDepth;
    for (int i = 0; i < STEPS; ++i, prev_sampleUV = sampleUV, prev_sampleDepth = sampleDepth) {
        sampleUV += rayScreenDirNorm * float3(texel, 1.0f);
        hit_confidence = UVEdgeFade(sampleUV.xy);
        if (hit_confidence == 0.0f) break;
        sampleDepth = Depth.SampleLevel(PointSampler, sampleUV.xy, 0);
        if (IsPotentialHit(sampleUV, sampleDepth)) {
            found_hit = CheckPotentialHit(sampleDepth, prev_sampleDepth, sampleUV, prev_sampleUV, l, hit_confidence);
            if (found_hit) {
                break;
            }
        }
    }
    if (found_hit) {
        Output[DTid.xy] = float4(Frame.SampleLevel(LinearSampler, sampleUV.xy, 0).rgb, hit_confidence);
    } else {
        Output[DTid.xy] = 0.0f;
    }

}