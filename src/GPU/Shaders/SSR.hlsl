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

bool IsPotentialHit(float3 sampleUV, float sampleDepth) { return sampleUV.z >= sampleDepth; }

bool CheckPotentialHit(inout float3 sampleUV, float3 sampleDepth, float3 rayUVDirNorm, float3 l, inout float hit_confidence)
{
    const float3 hit_normals = Gbuffer.SampleLevel(PointSampler, sampleUV.xy, 0).xyz;
    const float LoN = dot(l, hit_normals);
    hit_confidence = min(hit_confidence, ThresholdFade(LoN, 0.0f, 0, 0.005f));
    if (hit_confidence == 0.0f) {
        return false;
    }
    float d2 = precise_ray_depth(sampleUV, sampleDepth); // x > 0 means under surface for linear depth
    hit_confidence = min(hit_confidence, ThresholdFade(d2, g_CB.DepthThreshold, 0.5f, 0));
    if (hit_confidence == 0.0f) {
        return false;
    }

    float3 prev_UV = sampleUV - rayUVDirNorm;
    float prev_sampleDepth = Depth.SampleLevel(PointSampler, prev_UV, 0);
    float d1 = prev_sampleDepth - prev_UV.z;  // previous step should be > 0 means over surface
    d2 = sampleDepth - sampleUV.z;             //  x < 0 means under surface

    sampleUV = lerp(prev_UV, sampleUV, saturate(abs(d1) / (abs(d1) + abs(d2))));

    return true;
}

uint2 GetMipDimension(uint mipLevel) { return max(1, g_CB.FrameSize >> mipLevel); }

void AdvanceToNextCell(
    inout float3 sampleUV, float3 rayUVDirNorm, float3 inv_rayUVDirNorm, float sampleDepth, float2 texel, inout int mip) {
    uint2 mip_size = GetMipDimension(mip);
    float2 cell_size = 1.0f / mip_size;
    float2 cell_index = floor(sampleUV.xy * mip_size);

    // Calculate distances to the next X/Y cell walls
    float2 cell_boundary = cell_index * cell_size + select(rayUVDirNorm.xy > 0.0f, cell_size, 0.0f);
    float2 t_xy = (cell_boundary - sampleUV.xy) * inv_rayUVDirNorm.xy;

    // Prevent division-by-zero if the ray is perfectly parallel to an axis
    if (rayUVDirNorm.x == 0.0f) t_xy.x = 1e32f;
    if (rayUVDirNorm.y == 0.0f) t_xy.y = 1e32f;

    float t_exit_XY = min(t_xy.x, t_xy.y);

    // Calculate distance to the Z depth plane
    float t_exit_Z = (sampleDepth - sampleUV.z) * inv_rayUVDirNorm.z;
    if (rayUVDirNorm.z <= 0.0f) t_exit_Z = 1e32f;

    // Check if the ray will cross the Z plane BEFORE it leaves the current X/Y cell
    if (t_exit_Z > 0.0f && t_exit_Z < t_exit_XY) {
        // We hit the depth plane INSIDE the current cell..
        mip = max(mip - 1, 0);
        sampleUV += t_exit_Z * rayUVDirNorm;
        sampleUV.z = sampleDepth;
    } else {
        // We safely hit the X/Y cell wall. Push 0.1 pixels past it to enter the next cell.
        mip = min(mip + 1, g_CB.MaxDepthMipLevel);
        sampleUV += (max(t_exit_XY, 0) + 0.1) * rayUVDirNorm;
    }
}

[numthreads(16, 16, 1)] 
void CS_SSR(uint3 DTid : SV_DispatchThreadID) {
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;

    const int STEPS = 128;

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

    const float3 rayUVDirNorm = rayScreenDirNorm * float3(texel, 1.0f);
    const float3 inv_rayUVDirNorm = 1.0f / rayUVDirNorm;

    float3 sampleUV = float3(uv, centerDepth) + rayUVDirNorm * (1.0f + jitter.z);
    float sampleDepth = centerDepth;
    int mip = 0;
    for (int i = 0; i < STEPS; ++i) {
        hit_confidence = UVEdgeFade(sampleUV.xy);
        if (hit_confidence == 0.0f) break;
        sampleDepth = Depth.SampleLevel(PointSampler, sampleUV.xy, mip);
        if (IsPotentialHit(sampleUV, sampleDepth)) {
            if (mip == 0) {
                found_hit = CheckPotentialHit(sampleUV, sampleDepth, rayUVDirNorm, l, hit_confidence);
                if (found_hit) {
                    break;
                }

                // False positive thickness rejection: safely skip past this pixel's boundary
                float2 cell_boundary = floor(sampleUV.xy * g_CB.FrameSize) * texel + select(rayUVDirNorm.xy > 0.0f, texel, 0.0f);
                float2 t_xy = (cell_boundary - sampleUV.xy) * inv_rayUVDirNorm.xy;
                if (rayUVDirNorm.x == 0.0f) t_xy.x = 1e32f;
                if (rayUVDirNorm.y == 0.0f) t_xy.y = 1e32f;

                sampleUV += (min(t_xy.x, t_xy.y) + 0.1) * rayUVDirNorm;

                mip = min(mip + 1, g_CB.MaxDepthMipLevel);
            } else {
                mip -= 1;
            }
        } else {
            AdvanceToNextCell(sampleUV, rayUVDirNorm, inv_rayUVDirNorm, sampleDepth, texel, mip);
        }
        //if (i == STEPS - 1) { // Debug
        //    Output[DTid.xy] = float4(1.0f, 0.0f, 0.0f, 1.0f); // This means infinite loop somewhere
        //    return;
        //}
    }
    if (found_hit) {
        Output[DTid.xy] = float4(Frame.SampleLevel(LinearSampler, sampleUV.xy, 0).rgb, hit_confidence);
    } else {
        Output[DTid.xy] = 0.0f;
    }

}