struct ReprojectionCSInput {
    float4x4 Reprojection;
    uint2 FrameSize;
    float2 texel_size;
    float depth_threshold; // default: 0.001f
    float new_mix_factor;
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

    const float2 prevUV = prevNDC.xy * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);

    bool UsePrevSample = true;

    if (any(prevUV < 0.0f) || any(prevUV > 1.0f)) UsePrevSample = false;  // UV out of bounds

    const float4 gatheredDepth = OldDepth.Gather(PointSampler, prevUV);

    const float4 depthDelta = abs(gatheredDepth - prevNDC.z);
    const float4 depthValid = step(depthDelta, DepthThreshold);

    bool allValid = all(depthValid > 0.0f);     // consider reprojection valid only if all 4 gathered depth samples are within threshold
                                                // it helps with ghosting from bilinear sampling of color

    float4 prevColor = 0.0f;
    if (allValid) {
        prevColor = OldTexture.SampleLevel(LinearSampler, prevUV, 0);
    } else {  // Depth mismatch
        UsePrevSample = false;
    }

    float final_weight = UsePrevSample ? g_CB.new_mix_factor : 1.0f;

    NewTexture[DTid.xy] = lerp(prevColor, NewColor, final_weight);

    //if (UsePrevSample == false) NewTexture[DTid.xy] = float4(1.0, 0.0, 0.0, 1.0);  // Debug: red for reprojection failure
}