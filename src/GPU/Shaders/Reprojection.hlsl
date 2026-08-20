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
    int zeroAlphaReject;
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

// Distance Texture
Texture2D<float> DistanceTexture : register(t3, space0);

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

float4 linear_depth(float4 non_linear_depth) {
    const float proj_22 = g_CB.Projection_prev[2][2];
    const float proj_23 = g_CB.Projection_prev[2][3];
    return -proj_23 / (proj_22 + non_linear_depth);
}

float3 reproject_reflection_hit(uint3 DTid, float4 ndc, float depth, out float reflection_distance) {
    float reflection_depth = DistanceTexture.Load(DTid);
    ndc.z = reflection_depth;
    reflection_distance = linear_depth(depth) - linear_depth(reflection_depth);
	const float4 prevClip = mul(g_CB.Reprojection, ndc);
	return prevClip.xyz / prevClip.w;
}

float2 NDC2UV(float2 ndc) {
    return ndc * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);
}

float Luminance(float3 color) {
	return dot(color, float3(0.2126, 0.7152, 0.0722));
}

float MagnitudeSquare(float3 a) {
    return dot(a,a);
}
float MagnitudeSquare(float2 a) {
    return dot(a,a);
}

float GetDissoclusion(float2 prevUV, float depth, bool use_reflection_hit, float reflection_distance) {
    if (any(prevUV < 0.0f) || any(prevUV > 1.0f)) return 0.0f;  // UV out of bounds
    
    float DepthThreshold = g_CB.depth_threshold;  // Threshold for depth comparison
    float prevDepth = linear_depth(depth);
    if (use_reflection_hit)
    {
        prevDepth += reflection_distance;
        DepthThreshold *= 2;
    }
    bool allValid;
    if (g_CB.weak_depth_condition) {
        const float pointDepth = OldDepth.SampleLevel(PointSampler, prevUV, 0);
        allValid = (abs(linear_depth(pointDepth) - prevDepth) < DepthThreshold);   // consider reprojection valid if 
                                                                                   // bilinear-interpolated depth is within threshold
    } else {
        const float4 gatheredDepth = OldDepth.Gather(PointSampler, prevUV);
        const float4 depthDelta = abs(linear_depth(gatheredDepth) - prevDepth);
        const float4 depthValid = step(depthDelta, DepthThreshold);
        allValid = all(depthValid > 0.0f);  // consider reprojection valid only if all 4 gathered depth samples are within threshold
                                            // it helps with ghosting from bilinear sampling of color
    }
    return float(allValid);
}

[numthreads(16, 16, 1)] 
void CS_Reprojection(uint3 DTid : SV_DispatchThreadID) {
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;

    const float2 texel = g_CB.texel_size;
    const float2 uv = (DTid.xy + 0.5f) * texel;

    const float centerDepth = NewDepth.Load(DTid).r;
    const float4 NewColor = NewTexture.Load(DTid);
    const float3 VelocityNDC = Velocity.Load(DTid).xyz;
    
    if (centerDepth >= 1.0f || centerDepth <= 0.0f) {
        NewTexture[DTid.xy] = NewColor;
        return;
    }
    if (g_CB.zeroAlphaReject && NewColor.a == 0 && any(abs(VelocityNDC.xy) > 1e-2f)) {
        NewTexture[DTid.xy] = NewColor;
        return;
    }

    float4 m1 = 0.0;
	//float4 m2 = 0.0;
    [unroll]
	for (int x = -1; x <= 1; x++)
	{
        [unroll]
		for (int y = -1; y <= 1; y++)
		{
			int2 offset = int2(x, y);
			int2 coord = DTid.xy + offset;

			float4 sampleColor = NewTexture.Load(int3(coord, 0));

			m1 += sampleColor;
			//m2 += sampleColor * sampleColor;
		}
	}

	float4 mean = m1 / 9.0;
	//float4 variance = (m2 / 9.0) - (mean * mean);
    //float4 stddev = sqrt(max(variance, 0.0f));

    float4 ndc = float4(uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), centerDepth, 1.0f);
	float3 prevNDC = ndc.xyz - VelocityNDC;
    float3 prevNDC_reflection = prevNDC;
    float reflection_distance = 0.0f;
    if (g_CB.use_reflection_distance) {
		prevNDC_reflection = reproject_reflection_hit(DTid, ndc, centerDepth, reflection_distance);
	}
    float2 prevUV = NDC2UV(prevNDC.xy);
    float2 prevUV_reflection = NDC2UV(prevNDC_reflection.xy);
    
    float4 prevColor = OldTexture.SampleLevel(LinearSampler, prevUV, 0);
    float4 prevColor_reflection = OldTexture.SampleLevel(LinearSampler, prevUV_reflection, 0);
    
    bool use_reflection_hit = false;
    if (g_CB.use_reflection_distance) {
        float colDistance = MagnitudeSquare(prevColor.rgb - mean.rgb);
	    float colDistanceReflection = MagnitudeSquare(prevColor_reflection.rgb - mean.rgb);
        use_reflection_hit = (colDistanceReflection < colDistance);
        prevColor = use_reflection_hit ? prevColor_reflection : prevColor;
        prevUV = use_reflection_hit ? prevUV_reflection : prevUV;
        prevNDC = use_reflection_hit ? prevNDC_reflection : prevNDC;
    }
    
	float disocclusion = GetDissoclusion(prevUV, prevNDC.z, use_reflection_hit, reflection_distance);

    float final_weight = lerp(1.0f, g_CB.new_mix_factor, disocclusion);

    NewTexture[DTid.xy] = lerp(prevColor, NewColor, final_weight);

    if (g_CB.DebugMode && disocclusion == 0.0f) NewTexture[DTid.xy] = float4(1.0, 0.0, 0.0, 1.0);  // Debug: red for reprojection failure
}