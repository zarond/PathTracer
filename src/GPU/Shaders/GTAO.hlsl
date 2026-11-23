#include "BRDF.hlsl"

struct GTAOCSInput {
    float4x4 Projection;
    float inv_proj_00;
    float inv_proj_11;
    uint2 FrameSize;
    float2 texel_size;
    float AO_distance;
    float inv_AO_distance_sqr;
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
    //float4 clip = float4(ndcXY, depth, 1.0f);
    //float4 viewH = mul(g_CB.invProjection, clip);
    //return viewH.xyz / viewH.w;

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

float cosineWeightingIntegral(float min_angle, float max_angle, float normal_angle) {
    float sin_normal_angle, cos_normal_angle;
    sincos(normal_angle, sin_normal_angle, cos_normal_angle); 
    return 0.25f * (-cos(2 * max_angle - normal_angle) + cos_normal_angle + 2 * max_angle * sin_normal_angle +
                    -cos(2 * min_angle - normal_angle) + cos_normal_angle + 2 * min_angle * sin_normal_angle);
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
    const float inv_AO_distance_sq = g_CB.inv_AO_distance_sqr;
    const int NUM_DIRECTIONS = 4;
    const int STEPS_PER_DIR = 32;

    const float2 texel = g_CB.texel_size;
    const float2 uv = (DTid.xy + 0.5f) * texel;

    // same as ReconstructViewPosition, but code is outside for variable reuse
    const float2 center_ndc = uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f);
    const float3 center_ray = float3(-center_ndc.x * g_CB.inv_proj_00, -center_ndc.y * g_CB.inv_proj_11, 1.0f);

    const float neg_half_proj_00 = -0.5f * g_CB.Projection._11;
    const float half_proj_11 = 0.5f * g_CB.Projection._22;
    const float2 frame_size = g_CB.FrameSize;
    
    const float centerDepth = Depth.Load(DTid);
    if (centerDepth >= 1.0f || centerDepth <= 0.0f) {
        Output[DTid.xy] = float4(1.0f, 1.0f, 1.0f, 1.0f);
        return;
    }

    const float3 normal = normalize(Gbuffer.Load(DTid).xyz);
    
    // same as ReconstructViewPosition, but code is outside for variable reuse
    const float proj_22 = g_CB.Projection._33;
    const float neg_proj_23 = -g_CB.Projection._34;
    const float z_center = neg_proj_23 / (proj_22 + centerDepth);
    const float3 viewPos = z_center * center_ray;
    const float3 viewDir = normalize(center_ray);
    const float inv_z_center = -1.0f / z_center;

    float3 ls_X = float3(1, 0, 0);
    ls_X = normalize(ls_X - viewDir * dot(viewDir, ls_X));  // orthogonalize
    const float3 ls_Y = cross(viewDir, ls_X);

    const float spatial_directions_noise = frac(SpatialDirectionsNoise(DTid) + TemporalDirections());
    const float spatial_offsets_noise = frac(SpatialOffsetsNoise(DTid) + TemporalOffsets());

    float occlusion = 0.0f;
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
    {
        const float rot = PI * (float(i) + spatial_directions_noise) / NUM_DIRECTIONS;
        float sin_rot, cos_rot;
        sincos(rot, sin_rot, cos_rot); 
        const float3 rayViewDir = cos_rot * ls_X + sin_rot * ls_Y;
        
        const float2 projected_normal = float2(dot(normal, rayViewDir), dot(normal, viewDir));
        const float pn_len_sq = dot(projected_normal, projected_normal);
        const float projected_normal_length = sqrt(pn_len_sq);
        const float projected_normal_inv_length = (projected_normal_length < kEpsilon) ? 1.0f : rsqrt(pn_len_sq);
        const float2 projected_normal_normalized = clamp(projected_normal * projected_normal_inv_length, -1.0f, 1.0f);
        const float normal_angle = (-sign(projected_normal_normalized.x)) * acos(projected_normal_normalized.y);
        
        // Screen-space march direction calculation. The screen motion per unit step along rayViewDir is the
        // projection derivative at viewPos for ViewPositionToUV function
        const float2 march_delta = -float2(rayViewDir.x - center_ray.x * rayViewDir.z,
                                           rayViewDir.y - center_ray.y * rayViewDir.z);
        const float2 rayPixelDir = march_delta * frame_size * float2(neg_half_proj_00, half_proj_11);
        const float rayPixelDirLength = length(rayPixelDir);
        const float rayScreenDirLengthLimit = AO_distance * rayPixelDirLength * inv_z_center;
        const float inv_len = 1.0f / rayPixelDirLength;
        const float2 inv_len_texel = texel / rayPixelDirLength;
        const float2 rayScreenDirNorm = rayPixelDir * inv_len_texel;  // also apply texel scale here to avoid extra multiplication in the loop

        // Orthonormal slice frame {rayViewDir, viewDir, slice_normal}. 
        // A sample at offset t lies on its own pixel ray: samplePos == z_s * (center_ray + t * dR), with
        // z_s - view space Z coordinate from sampleDepth and dR the per-unit-offset change of the pixel ray.
        // For any axis A
        // dot(sampleDir, A) = dot(samplePos - viewPos, A)
        //                   = dot(z_s * (center_ray + t * dR) − viewPos, A)
        //                   = z_s * dot( (center_ray + t * dR), A) − dot(viewPos, A)
        //                   = z_s * ( dot(center_ray, A) + t * dot(dR, A) ) − dot(viewPos, A)
        //                   = z_s * (L0 + t * L1) − offset0
        const float3 slice_normal = -sin_rot * ls_X + cos_rot * ls_Y;
        const float3 L0 = float3(dot(center_ray, rayViewDir), dot(center_ray, viewDir), dot(center_ray, slice_normal));
        // viewPos coords in slice frame
        const float3 offset0 = viewPos.z * L0;  // since viewPos == viewPos.z * center_ray
        const float2 dR = march_delta * inv_len;   // pixel-ray change per unit march offset
        const float3 L1 = float3(dot(dR, rayViewDir.xy), dot(dR, viewDir.xy), dot(dR, slice_normal.xy));

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
                const float t_offset_with_noise = t_offset + spatial_offsets_noise * step_size;
                if (t_offset_with_noise > rayScreenDirLengthLimit) break;
                const float2 sampleUV = uv + (sign * t_offset_with_noise) * rayScreenDirNorm;
                if (any(sampleUV < 0.0f | sampleUV > 1.0f)) break;
                const float sampleDepth = Depth.SampleLevel(Sampler, sampleUV, DepthLod);
                
                const float z_s = neg_proj_23 / (proj_22 + sampleDepth);
                // Components of sampleDir in slice frame coordinates
                const float3 comp = z_s * mad(sign * t_offset_with_noise, L1, L0) - offset0;
                
                const float dist_sq = dot(comp, comp); // sampleDirLength^2 in slice frame
                const float Dist_fac = saturate(dist_sq * inv_AO_distance_sq);
                const float angle_cos = comp.y * rsqrt(dist_sq);
                acc_angle_cos = lerp(max(angle_cos, acc_angle_cos), acc_angle_cos, Dist_fac);
            }
            const float angle = acos(acc_angle_cos);
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

    const float ao = saturate(occlusion / (float)NUM_DIRECTIONS);
    Output[DTid.xy] = float4(ao, ao, ao, 1.0f);
}