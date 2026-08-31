struct TonemappingCSInput {
    uint2 FrameSize;
    int TonemapType;
    float multiplier;
    float white_point_constant_inverse;
};

static const int Clamp =                0;
static const int Reinhard =             1;
static const int Hable_original =       2;
static const int Hable_alternative =    3;
static const int ACES_Narkowicz =       4;
static const int ACES_Filmic =          5;
static const int Khronos_PBR_Neutral =  6;
static const int Gran_Turismo_Sport =   7;

Texture2D<float4> SrcTexture : register(t0);
RWTexture2D<float4> DstTexture : register(u0);

ConstantBuffer<TonemappingCSInput> g_CB : register(b0);

float3 reinhard(float3 x) {
    return x * (1.0 + x * g_CB.white_point_constant_inverse) / (1.0 + x);
}

float3 hable_original(float3 x) {
    const float A = 0.22f;
    const float B = 0.3f;
    const float C = 0.1f;
    const float D = 0.2f;
    const float E = 0.01f;
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
    return saturate(hable(x) * g_CB.white_point_constant_inverse);
}
float3 hable_original_tonemap(float3 x) {
    return saturate(hable_original(x) * g_CB.white_point_constant_inverse);
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

// Practical HDR and Wide Color Techniques in Gran Turismo SPORT from SIGGRAPH ASIA 2018
// [ https://www.desmos.com/calculator/gslcdxvipg]
float3 gran_turismo_sport(float3 x) {
	static const float P = 1;      // Maximum brightness
	static const float a = 1;      // Contrast
	static const float m = 0.22;   // Linear section start
	static const float l = 0.4;    // Linear section length
	static const float c = 1.33;   // Black tightness
	static const float b = 0;
	static const float l0 = (P - m)*l / a;
	//static const float L0 = m - m / a;
	//static const float L1 = m + (1 - m) / a;
	float3 L_x = m + a * (x - m);
	float3 T_x = m * pow(x / m, c) + b;
	static const float S0 = m + l0;
	static const float S1 = m + a * l0;
	static const float C2 = a * P / (P - S1);
	float3 S_x = P - (P - S1) * exp(-(C2*(x-S0)/P));
	float3 w0_x = 1.0 - smoothstep(0.0, m, x);
	float3 w2_x = step(m + l0, x);
	float3 w1_x = 1.0 - w0_x - w2_x;
	float3 f_x = T_x * w0_x + L_x * w1_x + S_x * w2_x;
	return f_x;
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
        case Reinhard:
            color.rgb = reinhard(color.rgb);
            break;
        case Hable_original:
            color.rgb = hable_original_tonemap(color.rgb);
            break;
        case Hable_alternative:
            color.rgb = hable_tonemap(color.rgb);
            break;
        case ACES_Narkowicz:
            color.rgb = tonemap_aces_narkowicz(color.rgb);
            break;
        case ACES_Filmic:
            color.rgb = tonemap_aces(color.rgb);
            break;
        case Khronos_PBR_Neutral:
            color.rgb = tonemap_pbr_neutral(color.rgb);
            break;
        case Gran_Turismo_Sport:
            color.rgb = gran_turismo_sport(color.rgb);
            break;
        default:
            color.rgb = saturate(color.rgb);
            break;
    }
    
    DstTexture[coord] = color;
}