#include "BRDF.hlsl"
#include "Common.hlsl"

// Constant Buffer
ConstantBuffer<RasterConstantBuffer> g_rasterCB : register(b0);

// Materials data buffers
StructuredBuffer<Material> Materials : register(t1, space0);

// DFG LUT
Texture2D<float4> DFGLut : register(t0, space1);

// Diffuse LUT
TextureCube<float4> DiffuseLut : register(t1, space1);

// Specular LUT
TextureCube<float4> SpecularLut : register(t2, space1);

// Default sampler
SamplerState Sampler : register(s0, space1);

// DFG sampler
SamplerState DFGSampler : register(s1, space1);

// Per draw call data
ConstantBuffer<RasterPerDrawData> DrawData : register(b1);

struct PSInput {
    float4 ndc_position : SV_POSITION;
    float4 world_position : POSITION;
    float4 normal : NORMAL;
    float4 tangent : TANGENT;
    float4 uv : TEXCOORD;
};

[shader("vertex")] 
PSInput VS_Main(
    float4 position : POSITION, 
    float4 normal : NORMAL, 
    float4 tangent : TANGENT, 
    float4 uv : TEXCOORD) 
{
    PSInput result;

    position.w = 1.0f;
    normal.w = 0.0f;
    position = mul(DrawData.modelMatrix, position);
    float tangent_sign = tangent.w;
    result.world_position = position;
    result.ndc_position = mul(g_rasterCB.viewProjection, position);
    result.normal = mul(DrawData.normalMatrix, normal);
    result.tangent = mul(DrawData.modelMatrix, tangent);
    result.tangent.w = tangent_sign;
    result.uv = uv;

    return result;
}

[shader("pixel")]
float4 PS_Main(PSInput input) : SV_TARGET {
    Material mat = Materials[DrawData.meshID];
    float2 uv = input.uv.xy;
    float4 albedo_color = sample_albedo(mat, uv, Sampler);
    float alpha = albedo_color.w;
    if (alpha < mat.alpha_cutoff)
        discard;  // alpha-test in the same shader for simplicity, but it disables early-z optimisation

    //return albedo_color;

    const float y_envmap_rotation = g_rasterCB.envmapRotation;  // todo: move to global variable matrix
    const float rot_cos = cos(y_envmap_rotation);
    const float rot_sin = sin(y_envmap_rotation);
    const float2x2 envmap_rotation_matrix = float2x2(rot_cos, -rot_sin, rot_sin, rot_cos);

    float3 emissive = sample_emissive(mat, uv, Sampler);

    float3 ORM = sample_occlusion_roughness_metallic(mat, uv, Sampler);
    const float AO = lerp(1.0, ORM.x, g_rasterCB.TexturesAOStrength);  // Ambient Occlusion, do I need it? Should I add it to raytracing as well?
    const float metallic = ORM.z;
    float3 diffuse_color = (1.0f - metallic) * albedo_color.rgb;

    float3 f0 = lerp(mat.dielectric_f0, albedo_color.rgb, metallic);
    const float3 f90 = 1.0f;

    const float roughness = ORM.y;
    const float linear_roughness = roughness * roughness;

    float3 N = input.normal.xyz;
    float3 T = input.tangent.xyz;
    float tangent_sign = input.tangent.w;
    float3 B = cross(N, T) * tangent_sign;

    bool has_normal_map;
    float3 normal_map_color = sample_normals(mat, uv, Sampler, has_normal_map);

    float3x3 TBN = construct_TBN(T, B, N);
    N = has_normal_map ? normal_map_sample_to_world(normal_map_color, TBN) : TBN[2];// new normal after normal mapping

    float3 v = normalize(g_rasterCB.cameraPosition.xyz - input.world_position.xyz);
    float NoV = dot(N, v);
    if (NoV < 0) {
        v = reflect(v, N);
        NoV = abs(NoV);
    }

    float3 l = reflect(-v, N);

    float2 DFG_uv = float2(NoV, roughness);

    float2 DFG = DFGLut.SampleLevel(DFGSampler, DFG_uv, 0).xy;
    float3 Fresnel = f0 * DFG.x + f90 * DFG.y;

    N.xz = mul(envmap_rotation_matrix, N.xz);  // enmap rotation
    float3 diffuseIBL = DiffuseLut.SampleLevel(Sampler, N, 0);
    l.xz = mul(envmap_rotation_matrix, l.xz);  // enmap rotation
    float3 specularIBL = SpecularLut.SampleLevel(Sampler, l, roughness * (g_rasterCB.SpecularLutMips - 1));

    float3 final_light = AO * diffuse_color * diffuseIBL * (1.0f - Fresnel) + Fresnel * specularIBL + emissive;

    return float4(final_light, alpha);
}

// Background pass

struct BG_VS_OUTPUT {
    float4 position : SV_POSITION;
    float3 viewDir : TEXCOORD0;
};

[shader("vertex")] 
BG_VS_OUTPUT VS_Background(uint vID : SV_VERTEXID) {
    BG_VS_OUTPUT output;

    // 1. Generate NDC coordinates procedurally
    // vID 0: (-1, -1), vID 1: (-1, 3), vID 2: (3, -1)
    float2 uv = float2((vID << 1) & 2, vID & 2);
    output.position = float4(uv * 2.0f - 1.0f, 0.0f, 1.0f);

    // 2. Reconstruct World-Space direction
    float4 worldPos = mul(g_rasterCB.projectionToWorld, output.position);

    // Divide by W to get the actual world position,
    output.viewDir = worldPos.xyz / worldPos.w;
    output.position.zw = 1.0f;

    return output;
}

[shader("pixel")]
float4 PS_Background(BG_VS_OUTPUT input) : SV_TARGET {
    float3 dir = input.viewDir;
    
    float y_envmap_rotation = g_rasterCB.envmapRotation;
    float rot_cos = cos(y_envmap_rotation);
    float rot_sin = sin(y_envmap_rotation);
    const float2x2 envmap_rotation_matrix = float2x2(rot_cos, -rot_sin, rot_sin, rot_cos);

    dir.xz = mul(envmap_rotation_matrix, dir.xz);  // Rotate around Y-axis

    float3 SpecularIBL = SpecularLut.SampleLevel(Sampler, dir, 0);
    return float4(SpecularIBL, 1.0);
}
