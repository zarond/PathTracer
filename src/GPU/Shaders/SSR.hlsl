#include "BRDF.hlsl"
#include "Common.hlsl"

struct SSRCSInput {
    float4x4 Projection;
    float inv_proj_00;
    float inv_proj_11;
    float proj_23_reciprocal;
    float padding;
    uint2 FrameSize;
    float2 texel_size;
    float DepthThreshold;
    float MaxRoughness;
    float2 temporal_jitter;
    float GGXBias;  // 1.0 is no bias, < 1.0 reduces tail of distribution
    int MaxDepthMipLevel;
    float MaxFrameMipLevel;
    float PrefilterDistance;
    float FocalPoint;
    int simpleResolve;
};

// Gbuff
Texture2D<float4> Gbuffer : register(t0, space0);

// Depth
Texture2D<float> Depth : register(t1, space0);
    
// Velocity Buffer
Texture2D<float4> Velocity : register(t2, space0);

// Frame
Texture2D<float4> Frame : register(t3, space0);

// SSR buffeer
Texture2D<float4> SSR_buffer : register(t4, space0);

// Output
RWTexture2D<float4> Output : register(u0, space0);

// Output Average Distance
RWTexture2D<float> OutputDistance : register(u1, space0);
    
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
    const float proj_22 = g_CB.Projection._33;
    const float proj_23 = g_CB.Projection._34;

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
    const float proj_00 = g_CB.Projection._11;
    const float proj_11 = g_CB.Projection._22;
    const float proj_22 = g_CB.Projection._33;
    const float proj_23 = g_CB.Projection._34;
    float inv_z = 1.0f / pos.z;
    float x_ndc = -pos.x * proj_00 * inv_z;
    float y_ndc = -pos.y * proj_11 * inv_z;
    float z_ndc = -proj_22 - proj_23 * inv_z;
    float3 ndc = float3(x_ndc, y_ndc, z_ndc);
    return float3(ndc.xy * float2(0.5f, -0.5f) + 0.5f, ndc.z);
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

// Algebraic Projection Scaling (Non-Linear Derivative)
float calculate_depth_threshold(float depth)
{
    const float proj_22 = g_CB.Projection._33;
    const float proj_23_rec = g_CB.proj_23_reciprocal;
    // Non-linear rate of change (dz_raw / dz_view)
    float tmp = (proj_22 + depth);
    float dz_raw = tmp * tmp * (-proj_23_rec);
    // Scaled non-linear threshold
    return g_CB.DepthThreshold * dz_raw;
}
    
bool CheckPotentialHit(float3 sampleUV, float sampleDepth, float3 rayUVDirNorm, float3 l)
{
    const float3 hit_normals = Gbuffer.SampleLevel(PointSampler, sampleUV.xy, 0).xyz;
    const float LoN = dot(l, hit_normals);
    if (LoN > 0.0f) {
        return false;
    }
    float threshold = calculate_depth_threshold(sampleDepth);
    if (sampleUV.z - sampleDepth > threshold) {
        return false; 
    }
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
void CS_SSR_trace(uint3 DTid : SV_DispatchThreadID) {
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
    float3 normal = GbufferData.xyz;
    const float roughness = clamp(GbufferData.w, 0.002f, max(g_CB.MaxRoughness, 0.002f));
    const float linear_roughness = roughness * roughness;
    const float3 pos = ReconstructViewPosition(uv, centerDepth);
    const float3 v = normalize(-pos);
    if (dot(normal, v) < 0) {
        normal = normalize(normal - v * dot(normal, v));
    }

    float3 tangent = normal.x < 0.9f ? float3(1, 0, 0) : float3(0, 1, 0);
    tangent = normalize(tangent - normal * dot(normal, tangent));
    const float3 bitangent = cross(normal, tangent);
    float3x3 TBN = float3x3(tangent, bitangent, normal);

    //-------------------------------------------------------
    bool found_hit = false;

    uint3 seed = uint3(DTid.xy, 0);

    float3 v_in_TBN_space = mul(TBN, v);
    float3 jitter = pcg3d(seed);
    float2 rand = frac(g_CB.temporal_jitter + jitter.xy);
    rand.x *= g_CB.GGXBias; // Cut tails of distribution to reduce noise (introduces bias)
    float3 h = sampleGGXVNDF(v_in_TBN_space, linear_roughness, rand.x, rand.y);
    float inv_pdf = 1.0f / clamp(PDF_of_importanceSampleGGXVNDF(h.z, v_in_TBN_space.z, linear_roughness), 0.0001f, 10000.0f);
    h = Tangent2World(h, TBN);

    const float3 l = reflect(-v, h);

    const float3 rayUVDir = ViewPositionToUV(pos + l * 0.01f) - float3(uv, centerDepth);
    const float3 rayScreenDir = rayUVDir * float3(g_CB.FrameSize, 1.0f);

    const float rayScreenDirLength = length(rayScreenDir.xy);
    const float3 rayScreenDirNorm = rayScreenDir / rayScreenDirLength;

    const float3 rayUVDirNorm = rayScreenDirNorm * float3(texel, 1.0f);
    const float3 inv_rayUVDirNorm = 1.0f / rayUVDirNorm;
        
    float one_pixel_step_size = min(texel.x * abs(inv_rayUVDirNorm.x), texel.y * abs(inv_rayUVDirNorm.y));

    float3 sampleUV = float3(uv + one_pixel_step_size * rayUVDirNorm.xy, centerDepth);
    float sampleDepth = centerDepth;
    int mip = 0;
    for (int i = 0; i < STEPS; ++i) {
        if (any(sampleUV < 0.0f | sampleUV > 1.0f)) break;
        sampleDepth = Depth.SampleLevel(PointSampler, sampleUV.xy, mip);
        if (IsPotentialHit(sampleUV, sampleDepth)) {
            if (mip == 0) {
                found_hit = CheckPotentialHit(sampleUV, sampleDepth, rayUVDirNorm, l);
                if (found_hit) {
                    break;
                }
                // False positive thickness rejection: linear search if ray is begind geometry
                sampleUV += one_pixel_step_size * rayUVDirNorm;
            } else {
                mip -= 1;
            }
        } else {
            AdvanceToNextCell(sampleUV, rayUVDirNorm, inv_rayUVDirNorm, sampleDepth, texel, mip);
        }
    }
    if (!found_hit) {
        sampleUV = l; // save ray direction in VS in case of miss
        inv_pdf = -inv_pdf; // negative inv_pdf indicates miss
    }
    Output[DTid.xy] = float4(sampleUV, inv_pdf); // save UV + NDC Z coords in case of hit
}
    
struct ssr_hit_info {
    float3 pos;         // View-space position of the hit point
    float confidence;   // Confidence level of the hit, 0 - no hit
    float2 uv;          // UV coordinates of the hit point in the previous frame
    float inv_pdf;
    float padding;
};
    
ssr_hit_info decode_SSR_hit_info(float4 SSR_sample) 
{
    ssr_hit_info hit;
    
    bool hit_valid = SSR_sample.w > 0.0f;
    float2 prev_uv = float2(0.0f, 0.0f);
    float3 hit_pos;
    if (hit_valid) {
        float2 current_uv = SSR_sample.xy;
        float depth = SSR_sample.z;
        float2 ndc_xy = current_uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f);
        float2 prev_ndc = ndc_xy - Velocity.SampleLevel(PointSampler, current_uv, 0).rg;
        prev_uv = prev_ndc * float2(0.5f, -0.5f) + 0.5f;
        hit_pos = ReconstructViewPosition(current_uv, depth);    
    } else {
        hit_pos = SSR_sample.xyz; // Use the ray direction in view space for misses
    }
        
    hit.pos = hit_pos;
    hit.uv = prev_uv;
    hit.confidence = hit_valid ? UVEdgeFade(prev_uv) : 0.0f;
    hit.inv_pdf = abs(SSR_sample.w);
        
    return hit;
}
    
float3 prefilteredSample(float ray_distance, float linearRoughness, float depth, float2 uv) {
    // Mipmap LOD calculation (Roughness prefiltering)
    // Footprint expands based on roughness and distance traveled by ray
    float coneRadius = ray_distance * linearRoughness * g_CB.PrefilterDistance;
    coneRadius *= g_CB.FocalPoint / depth;
    float lod = clamp(log2(2.0f * coneRadius + kEpsilon5), 0.0f, g_CB.MaxFrameMipLevel);
    // Sample Frame texture with roughness LOD blur
    return Frame.SampleLevel(LinearSampler, uv, lod).rgb;
}
    
float ndc_depth(float linear_depth) {
    const float proj_22 = g_CB.Projection._33;
    const float proj_23 = g_CB.Projection._34;
    float inv_z = 1.0f / linear_depth;
    float z_ndc = -proj_22 - proj_23 * inv_z;
    return z_ndc;
}

float linear_depth(float non_linear_depth) {
    // Assuming standard perspective projection matrix, reconstruct linear depth from non-linear depth
    const float proj_22 = g_CB.Projection._33;
    const float proj_23 = g_CB.Projection._34;
    return -proj_23 / (proj_22 + non_linear_depth);
}
    
groupshared ssr_hit_info LDS_SSR[20][20]; // 20 = 2 + 16 + 2
    
void PopulateSharedMemory(uint3 DTid, uint3 Gid, uint3 Tid, uint Gidx)
{
    int2 groupOrigin = (int2)Gid.xy * 16 - 2;

    for (uint i = Gidx; i < 20 * 20; i += (16 * 16)) 
    {
        uint ldsY = i / 20;
        uint ldsX = i % 20;

        // Map LDS 2D coordinate to actual global screen pixel coordinate
        int2 sampleCoord = groupOrigin + int2(ldsX, ldsY);

        // Clamp to edge to prevent out-of-bounds sampling at screen borders
        sampleCoord = max(0, min(sampleCoord, g_CB.FrameSize - 1));

        // Store in shared memory
        float4 SSR_sample = SSR_buffer[sampleCoord];
        LDS_SSR[ldsY][ldsX] = decode_SSR_hit_info(SSR_sample);
    }
    GroupMemoryBarrierWithGroupSync();
}

[numthreads(16, 16, 1)] 
void CS_SSR_resolve(uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) {
    if (g_CB.simpleResolve == 0) {
        PopulateSharedMemory(DTid, Gid, Tid, Gidx);
    }
    
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;

    const float4 centerGbuffer = Gbuffer.Load(int3(DTid.xy, 0));
    float3 N = centerGbuffer.rgb; 
    const float centerRoughness = clamp(centerGbuffer.w, 0.002f, max(g_CB.MaxRoughness, 0.002f));
    const float linearRoughness = centerRoughness * centerRoughness;

    const float centerDepth = Depth.Load(int3(DTid.xy, 0));
    if (centerDepth >= 1.0f) { // Background/Skybox check
        Output[DTid.xy] = float4(0.0f, 0.0f, 0.0f, 0.0f);
        OutputDistance[DTid.xy] = 1.0f;
        return;
    }

    const float2 centerUV = (float2(DTid.xy) + 0.5f) * g_CB.texel_size;
    const float3 centerPosVS = ReconstructViewPosition(centerUV, centerDepth);
    
    const float3 V = normalize(-centerPosVS);
    float NdotV = dot(N, V);
        
    if (NdotV < 0) {
        N = normalize(N - V * NdotV);
        NdotV = 0;
    }
        
    if (g_CB.simpleResolve)
    {
        float4 SSR_sample = SSR_buffer.Load(int3(DTid.xy, 0));
        ssr_hit_info hit_info = decode_SSR_hit_info(SSR_sample);
        float3 ray = hit_info.pos - centerPosVS;
        bool valid_hit = hit_info.confidence > 0;
        float ray_distance = valid_hit ? length(ray) : 0.0f;
        float3 frameSample = prefilteredSample(ray_distance, linearRoughness, -centerPosVS.z, hit_info.uv);;
        Output[DTid.xy] = float4(frameSample * hit_info.confidence, hit_info.confidence);
        OutputDistance[DTid.xy] = valid_hit ? saturate(ndc_depth(linear_depth(centerDepth) - ray_distance)) : centerDepth;
        return;
    }
    
    float3 colorAccum = float3(0.0f, 0.0f, 0.0f);
    float totalWeight = 0.0f;
    float hitConfidenceAccum = 0.0f;
    float distAccum = 0.0f;
    uint valid_points = 0;
        
    // 5x5 Spatial Neighborhood Kernel
    for (int i = -2; i <= 2; ++i) {
        for (int j = -2; j <= 2; ++j) {
            int2 neighborCoord = int2(DTid.xy) + int2(i, j);

            // Boundary check
            if (any(neighborCoord < 0 | neighborCoord >= g_CB.FrameSize)) 
                continue;

            // Load neighbor ray hit UVs
            ssr_hit_info hit_info = LDS_SSR[Tid.y + i + 2][Tid.x + j + 2];
            bool hit_valid = hit_info.confidence > 0.0f;
               
            float3 ray = hit_info.pos - hit_valid * centerPosVS;
            float ray_distance = length(ray);
               
            // Light vector L and Half-vector H
            float3 L = normalize(ray);
            float3 H = normalize(V + L);
            float NdotL = saturate(dot(N, L));

            if (NdotL <= 0.0f) continue;

            float NdotH = saturate(dot(N, H));
            float VdotH = saturate(dot(V, H));

            // (BRDF / pdf) * Cos Weighting
            // BRDF_GGX = (D * F * G) / (4 * NdotV * NdotL)
            float weight = D_GGX(NdotH, linearRoughness) * V_Schlick(NdotL, NdotV, centerRoughness) * NdotL * hit_info.inv_pdf;

            if (hit_valid) {
                float3 frameSample = prefilteredSample(ray_distance, linearRoughness, -centerPosVS.z, hit_info.uv);
                colorAccum += frameSample * weight * hit_info.confidence;   
            }

            totalWeight += weight;
            hitConfidenceAccum += hit_info.confidence * weight;
            distAccum += hit_valid ? ray_distance : 0.0f;
            valid_points += hit_valid;
        }
    }

    if (totalWeight > 0) {
        float3 finalColor = colorAccum.rgb / totalWeight;
        float confidence = hitConfidenceAccum / totalWeight;
        float averageDist = valid_points > 0 ? distAccum * abs(V.z) / valid_points : 0;

        Output[DTid.xy] = float4(finalColor, confidence);
        OutputDistance[DTid.xy] = valid_points > 0 ? saturate(ndc_depth(linear_depth(centerDepth) - averageDist)) : centerDepth;
    } else {
        Output[DTid.xy] = float4(0.0f, 0.0f, 0.0f, 0.0f);
        OutputDistance[DTid.xy] = centerDepth;
    }
}