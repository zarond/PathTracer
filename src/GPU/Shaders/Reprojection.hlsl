struct ReprojectionCSInput {
    float4x4 Reprojection;
    float4x4 Projection_prev;
    float proj_22;
    float proj_23;
    uint2 FrameSize;
    float2 texel_size;
    float depth_threshold; // default: 0.001f
    float new_mix_factor;
    int weak_depth_condition;
    int use_reflection_distance;
    int DebugMode;  // 0 - normal, 1 - show reprojection failure in red
};

// Old Texture
Texture2D<float4> OldTexture : register(t0, space0);

// Old Depth
Texture2D<float> OldDepth : register(t1, space0);

// New Texture
RWTexture2D<float4> NewTexture : register(u0, space0);

// New Depth
Texture2D<float> NewDepth : register(t2, space0);

// SSR buffer with Distance on B channel for SSR parallax
Texture2D<float4> SSR_buffer : register(t3, space0);

// Velocity Buffer
Texture2D<float4> Velocity : register(t4, space0);

// Constant Buffer
ConstantBuffer<ReprojectionCSInput> g_CB : register(b0);

// Point sampler
SamplerState PointSampler : register(s0, space0);

// Default Linear sampler
SamplerState LinearSampler : register(s1, space0);

float ndc_depth(float linear_depth) {
    const float proj_22 = g_CB.proj_22;
    const float proj_23 = g_CB.proj_23;
    float inv_z = 1.0f / linear_depth;
    float z_ndc = -proj_22 - proj_23 * inv_z;
    return z_ndc;
}

float new_linear_depth(float non_linear_depth) {
    // Assuming standard perspective projection matrix, reconstruct linear depth from non-linear depth
    const float proj_22 = g_CB.proj_22;
    const float proj_23 = g_CB.proj_23;
    return -proj_23 / (proj_22 + non_linear_depth);
}

float linear_depth(float non_linear_depth) {
    // Assuming standard perspective projection matrix, reconstruct linear depth from non-linear depth
    const float proj_22 = g_CB.Projection_prev[2][2];
    const float proj_23 = g_CB.Projection_prev[2][3];
    return -proj_23 / (proj_22 + non_linear_depth);
}

float linear_depth(float4 non_linear_depth) {
    const float proj_22 = g_CB.Projection_prev[2][2];
    const float proj_23 = g_CB.Projection_prev[2][3];
    return -proj_23 / (proj_22 + non_linear_depth);
}

[numthreads(16, 16, 1)] 
void CS_Reprojection(uint3 DTid : SV_DispatchThreadID) {
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;

    const float DepthThreshold = g_CB.depth_threshold;  // Threshold for depth comparison

    const float2 texel = g_CB.texel_size;
    const float2 uv = (DTid.xy + 0.5f) * texel;

    const float centerDepth = NewDepth.Load(DTid);
    const float4 NewColor = NewTexture.Load(DTid);

    float4 ndc = float4(uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), centerDepth, 1.0f);
	float3 prevNDC;

    float reflection_distance = 0.0f;
    if (g_CB.use_reflection_distance) {
        reflection_distance = SSR_buffer.Load(DTid).b;
        if (reflection_distance > 0) {
            float pos_z = new_linear_depth(ndc.z);
            pos_z -= reflection_distance;
            ndc.z = ndc_depth(pos_z);
        }
		const float4 prevClip = mul(g_CB.Reprojection, ndc);
		prevNDC = prevClip.xyz / prevClip.w;
	} else {
		prevNDC = ndc.xyz - Velocity.Load(DTid).xyz;
	}

	float prevDepth = linear_depth(prevNDC.z);

    prevDepth += max(reflection_distance, 0);

    const float2 prevUV = prevNDC.xy * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);

    bool UsePrevSample = true;

    if (any(prevUV < 0.0f) || any(prevUV > 1.0f) || reflection_distance < 0) UsePrevSample = false;  // UV out of bounds

    bool strong_condition = (g_CB.weak_depth_condition == 0);
    const float4 gatheredDepth = OldDepth.Gather(PointSampler, prevUV);
    const float pointDepth = OldDepth.SampleLevel(PointSampler, prevUV, 0);

    bool allValid;
    if (strong_condition) {
        const float4 depthDelta = abs(linear_depth(gatheredDepth) - prevDepth);
        const float4 depthValid = step(depthDelta, DepthThreshold);
        allValid = all(depthValid > 0.0f);  // consider reprojection valid only if all 4 gathered depth samples are within threshold
                                            // it helps with ghosting from bilinear sampling of color
    } else {
        allValid = (abs(linear_depth(pointDepth) - prevDepth) < DepthThreshold);   // consider reprojection valid if 
                                                                                   // bilinear-interpolated depth is within threshold
    }

    float4 prevColor = 0.0f;
    if (allValid) {
        prevColor = OldTexture.SampleLevel(LinearSampler, prevUV, 0);
    } else {  // Depth mismatch
        UsePrevSample = false;
    }

    float final_weight = UsePrevSample ? g_CB.new_mix_factor : 1.0f;

    NewTexture[DTid.xy] = lerp(prevColor, NewColor, final_weight);

    if (g_CB.DebugMode && UsePrevSample == false) NewTexture[DTid.xy] = float4(1.0, 0.0, 0.0, 1.0);  // Debug: red for reprojection failure
}