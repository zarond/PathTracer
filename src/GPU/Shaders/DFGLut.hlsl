#include "BRDF.hlsl"

// Output texture, accessed as a UAV
RWTexture2D<float4> gOutput : register(u0);

[numthreads(16, 16, 1)] 
void CS_DFG_Lut(uint3 DTid : SV_DispatchThreadID) {
    uint2 destSize;
    gOutput.GetDimensions(destSize.x, destSize.y);

    if (DTid.x >= (uint)destSize.x || DTid.y >= (uint)destSize.y) return;

    float2 uv = (DTid.xy + 0.5f) / destSize;

    const float NoV = uv.x;       // todo: remap to [0, 1] exactly?
    const float roughness = uv.y; // todo: remap to [0, 1] exactly?
    const float linear_roughness = roughness * roughness;

    float3 v = float3(sqrt(saturate(1.0f - NoV * NoV)), 0, NoV);

    float2 result = 0.0f;
    const int N_samples = 512;
    const float inv_N_samples = 1.0f / N_samples;
    for (int i = 0; i < N_samples; ++i) {
        const float2 rand = fibonacci2D(i, inv_N_samples);
        const float3 m = importanceSampleGGX(rand, linear_roughness);
        const float3 l = normalize(reflect(-v, m));  // normalizing for better accuracy

        const float VoM = saturate(dot(v, m));
        const float LoM = saturate(dot(l, m));
        const float LoN = saturate(l.z);
        const float NoM = saturate(m.z);
        if (LoN > 0) {
            const float G = V_SmithGGXCorrelated(NoV, LoN, linear_roughness);
            const float visibility = G * LoN * VoM / NoM;
            const float F = pow5(1.0f - VoM);
            result.x += visibility * (1.0f - F);
            result.y += visibility * F;
        }
    }
    result *= inv_N_samples;

    gOutput[DTid.xy] = float4(result.xy, 0.0, 1.0);
}