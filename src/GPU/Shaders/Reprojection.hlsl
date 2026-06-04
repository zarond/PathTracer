struct ReprojectionCSInput {
    float4x4 Reprojection;
    float4x4 Projection_prev;
    uint2 FrameSize;
    float2 texel_size;
    float depth_threshold; // default: 0.001f
    float new_mix_factor;
    int weak_depth_condition;
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

// Constant Buffer
ConstantBuffer<ReprojectionCSInput> g_CB : register(b0);

// Point sampler
SamplerState PointSampler : register(s0, space0);

// Default Linear sampler
SamplerState LinearSampler : register(s1, space0);

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

    const float4 ndc = float4(uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), centerDepth, 1.0f);

    const float4 prevClip = mul(g_CB.Reprojection, ndc);

    const float3 prevNDC = prevClip.xyz / prevClip.w;
    const float prevDepth = linear_depth(prevNDC.z);

    const float2 prevUV = prevNDC.xy * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);

    bool UsePrevSample = true;

    if (any(prevUV < 0.0f) || any(prevUV > 1.0f)) UsePrevSample = false;  // UV out of bounds

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