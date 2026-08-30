struct TonemappingCSInput {
    uint2 FrameSize;
    int TonemapType;
    float multiplier;
};

static const int Clamp =                0;
static const int Reinhard =             1;
static const int Hable =                2;
static const int ACES_Narkowicz =       3;
static const int ACES_Filmic =          4;
static const int Khronos_PBR_Neutral =  5;

Texture2D<float4> SrcTexture : register(t0);
RWTexture2D<float4> DstTexture : register(u0);

ConstantBuffer<TonemappingCSInput> g_CB : register(b0);

float3 reinhard(float3 x) {
    return x / (1.0 + x);
}

float hable_1d(float x) {
    const float A = 0.15f;
    const float B = 0.5f;
    const float C = 0.1f;
    const float D = 0.2f;
    const float E = 0.02f;
    const float F = 0.3f;

    return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F;
}
float3 hable(float3 x) {
    const float A = 0.15f;
    const float B = 0.5f;
    const float C = 0.1f;
    const float D = 0.2f;
    const float E = 0.02f;
    const float F = 0.3f;

    return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F;
}
float3 hable_tonemap(float3 x) {
    static const float hW = 11.2;
    //static const float whiteScale = 1.0f / hable_1d(hW);
    static const float whiteScale = 1.0f / 0.72513f;
    return hable(x) * whiteScale;
}

float3 tonemap_aces_narkowicz(float3 x) {
    const float a = 2.51f;
    const float b = 0.03f;
    const float c = 2.43f;
    const float d = 0.59f;
    const float e = 0.14f;
    return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
}

float3 aces_rrt_and_odt_fit(float3 v) {
    float3 a = v * (v + 0.0245786) - 0.000090537;
    float3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

float3 tonemap_aces(float3 color) {
    const float3x3 aces_input_transform = {
        {0.59719, 0.35458, 0.04823},
        {0.07600, 0.90834, 0.01566},
        {0.02840, 0.13383, 0.83777}
    };
    const float3x3 aces_output_transform = {
        { 1.60475, -0.53108, -0.07367},
        {-0.10208,  1.10813, -0.00605},
        {-0.00327, -0.07276,  1.07602}
    };
    color = mul(aces_input_transform, color);
    color = aces_rrt_and_odt_fit(color);
    color = mul(aces_output_transform, color);

    return saturate(color);
}

// https://github.com/KhronosGroup/ToneMapping/blob/main/PBR_Neutral/pbrNeutral.glsl
// License: Apache License 2.0
float3 tonemap_pbr_neutral(float3 color) {
    const float startCompression = 0.8 - 0.04;
    const float desaturation = 0.15;

    float x = min(color.r, min(color.g, color.b));
    float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
    color -= offset;

    float peak = max(color.r, max(color.g, color.b));
    if (peak < startCompression) return color;

    const float d = 1. - startCompression;
    float newPeak = 1. - d * d / (peak + d - startCompression);
    color *= newPeak / peak;

    float g = 1. - 1. / (desaturation * (peak - newPeak) + 1.);
    return lerp(color, newPeak * float3(1, 1, 1), g);
}

[numthreads(16, 16, 1)]
void CS_Tonemapping(uint3 DTid : SV_DispatchThreadID)
{
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;
    uint2 coord = DTid.xy;
    float tonemap_const = g_CB.multiplier;
    int tonemapping_type = g_CB.TonemapType;
    float4 color = SrcTexture.Load(int3(coord, 0));
    color.rgb *= tonemap_const;
    
    switch (tonemapping_type) {
        case 1:
            color.rgb = reinhard(color.rgb);
            break;
        case 2:
            color.rgb = hable_tonemap(color.rgb);
            break;
        case 3:
            color.rgb = tonemap_aces_narkowicz(color.rgb);
            break;
        case 4:
            color.rgb = tonemap_aces(color.rgb);
            break;
        case 5:
            color.rgb = tonemap_pbr_neutral(color.rgb);
            break;
        default:
            color.rgb = saturate(color.rgb);
            break;
    }
    
    DstTexture[coord] = color;
}