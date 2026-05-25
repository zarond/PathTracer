struct KawaseCSInput {
    float2 halftexel;
    int numMips;
};

Texture2D<float4> Input : register(t0, space0);
RWTexture2D<float4> Output : register(u0, space0);

// Clamp sampler
SamplerState Sampler : register(s0, space0);

// Input parameters
ConstantBuffer<KawaseCSInput> InputInfo : register(b0);

[numthreads(16, 16, 1)] 
void CS_KawaseBlurDownsample_5(
    uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) {
    uint2 imageSize;
    Output.GetDimensions(imageSize.x, imageSize.y);

    if (DTid.x >= imageSize.x || DTid.y >= imageSize.y) return;

    const float2 o = InputInfo.halftexel;
    const float2 uv = (DTid.xy + 0.5f) * 2.0f * o;

    float4 color = Input.SampleLevel(Sampler, uv, 0.0f) * 4.0f;

    color += Input.SampleLevel(Sampler, uv - o, 0.0f);
    color += Input.SampleLevel(Sampler, uv + float2(o.x, -o.y), 0.0f);
    color += Input.SampleLevel(Sampler, uv - float2(o.x, -o.y), 0.0f);
    color += Input.SampleLevel(Sampler, uv + o, 0.0f);

    Output[DTid.xy] = color * 0.125f;
}

[numthreads(16, 16, 1)] 
void CS_KawaseBlurDownsample_13(
        uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) {
    uint2 imageSize;
    Output.GetDimensions(imageSize.x, imageSize.y);

    if (DTid.x >= imageSize.x || DTid.y >= imageSize.y) return;

    const float2 o = InputInfo.halftexel;
    const float2 uv = (DTid.xy + 0.5f) * 2.0f * o;

   // --- 13-Tap Downsample (Jimenez 2014) ---

    // 1. Center tap (Weight: 0.125)
    float4 color = Input.SampleLevel(Sampler, uv, 0.0f) * 0.125f;

    // 2. Inner Box - 1 source texel offset (Weight: 1 / 8 = 0.125 each)
    color += Input.SampleLevel(Sampler, uv + float2(-o.x, -o.y), 0.0f) * 0.125f;
    color += Input.SampleLevel(Sampler, uv + float2( o.x, -o.y), 0.0f) * 0.125f;
    color += Input.SampleLevel(Sampler, uv + float2(-o.x,  o.y), 0.0f) * 0.125f;
    color += Input.SampleLevel(Sampler, uv + float2( o.x,  o.y), 0.0f) * 0.125f;

    // 3. Outer Cross - 2 source texels offset (Weight: 1 / 16 = 0.0625 each)
    color += Input.SampleLevel(Sampler, uv + float2(0.0f, -2.0f * o.y), 0.0f) * 0.0625f;
    color += Input.SampleLevel(Sampler, uv + float2(-2.0f * o.x, 0.0f), 0.0f) * 0.0625f;
    color += Input.SampleLevel(Sampler, uv + float2( 2.0f * o.x, 0.0f), 0.0f) * 0.0625f;
    color += Input.SampleLevel(Sampler, uv + float2(0.0f,  2.0f * o.y), 0.0f) * 0.0625f;

    // 4. Outer Corners - 2 source texels offset (Weight: 1 / 32 = 0.03125 each)
    color += Input.SampleLevel(Sampler, uv + float2(-2.0f * o.x, -2.0f * o.y), 0.0f) * 0.03125f;
    color += Input.SampleLevel(Sampler, uv + float2( 2.0f * o.x, -2.0f * o.y), 0.0f) * 0.03125f;
    color += Input.SampleLevel(Sampler, uv + float2(-2.0f * o.x,  2.0f * o.y), 0.0f) * 0.03125f;
    color += Input.SampleLevel(Sampler, uv + float2( 2.0f * o.x,  2.0f * o.y), 0.0f) * 0.03125f;

    Output[DTid.xy] = color;
}

[numthreads(16, 16, 1)] 
void CS_KawaseBlurUpsample(
        uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 Tid : SV_GroupThreadID, uint Gidx : SV_GroupIndex) {
    uint2 imageSize;
    Output.GetDimensions(imageSize.x, imageSize.y);

    if (DTid.x >= imageSize.x || DTid.y >= imageSize.y) return;

    const float2 o = InputInfo.halftexel;
    const float2 uv = (DTid.xy + 0.5f) * o;

    float4 color = 0.0f;

    color += Input.SampleLevel(Sampler, uv - float2(o.x * 2, 0), 0.0f);
    color += Input.SampleLevel(Sampler, uv + float2(o.x * 2, 0), 0.0f);
    color += Input.SampleLevel(Sampler, uv - float2(0, o.y * 2), 0.0f);
    color += Input.SampleLevel(Sampler, uv + float2(0, o.y * 2), 0.0f);

    color += Input.SampleLevel(Sampler, uv - float2(o.x, -o.y), 0.0f) * 2.0f;
    color += Input.SampleLevel(Sampler, uv + o, 0.0f) * 2.0f;
    color += Input.SampleLevel(Sampler, uv - o, 0.0f) * 2.0f;
    color += Input.SampleLevel(Sampler, uv + float2(o.x, -o.y), 0.0f) * 2.0f;

    Output[DTid.xy] = color / 12.0f;
}