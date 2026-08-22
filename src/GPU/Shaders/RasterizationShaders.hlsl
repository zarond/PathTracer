#include "BRDF.hlsl"
#include "Common.hlsl"
#include "Material_IDS.hlsl"

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

// Frame
Texture2D<float4> Frame : register(t3, space1);

// Ambient Occlusion
Texture2D<float4> AmbientOcclusionTexture : register(t4, space1);

// Screen-space Reflections
Texture2D<float4> SSRTexture : register(t5, space1);

// Material IDs
Texture2D<unsigned int> MaterialID : register(t6, space1);

// Default sampler
SamplerState Sampler : register(s0, space1);

// Anisotropic sampler
SamplerState ASampler : register(s1, space1);

// DFG sampler
SamplerState DFGSampler : register(s2, space1);

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
    tangent.w = 0.0f; 
    result.world_position = position;
    result.ndc_position = mul(g_rasterCB.viewProjection, position);
    result.normal = mul(DrawData.normalMatrix, normal);
    result.tangent = mul(DrawData.modelMatrix, tangent);
    result.tangent.w = tangent_sign;
    result.uv = uv;

    return result;
}

struct RefractResult {
    float3 pos;
    float3 dir;
    float dist;
};

RefractResult SimulateRefraction(float3 ws_pos, float3 v, float3 N, float ior, float thickness) {  // thin slab approximation
    float3 refract_dir = refract(-v, N, 1.0f / ior);
    //thickness /= min(-dot(refract_dir, N), 0.01f);
    ws_pos += refract_dir * thickness;

    RefractResult r;
    r.pos = ws_pos;
    r.dir = refract_dir;
    r.dist = thickness;
    return r;
}

float4 SampleFrameBicubic(float2 uv, float lod) {
    if (lod < 0.8f) {
        return Frame.SampleLevel(DFGSampler, uv, lod);
    } else {
        return SampleTextureBicubicMip(Frame, DFGSampler, uv, g_rasterCB.FrameSize, lod);
    }
}

float3 sampleBackgroundAndIBL(float3 ws_pos, float3 refract_dir, float t_roughness, float2x2 envmap_rotation_matrix) {
    float4 ndc = mul(g_rasterCB.viewProjection, float4(ws_pos, 1.0f));
    ndc /= ndc.w;
    ndc.y *= -1.0f;
    float2 uv = ndc.xy * 0.5f + 0.5f;
    float lod = t_roughness * (g_rasterCB.RenderFrameMips - 1);
    float4 frame_sample = SampleFrameBicubic(uv, lod);
    float alpha = frame_sample.a;
    alpha *= 1.0f / 0.9f; // remap a little
    if (alpha >= 1.0f) return frame_sample.rgb;
    // else we need IBL as well
    refract_dir.xz = mul(envmap_rotation_matrix, refract_dir.xz);
    float3 specularIBL = SpecularLut.SampleLevel(Sampler, refract_dir, lod).rgb;
    return lerp(specularIBL, frame_sample.rgb, min(alpha, 1.0f));
}

float3 calculateTransmittedLight(float3 ws_pos, float4 ndc_position, float3 v, float3 N, const Material mat, float3 Fresnel, float linear_roughness,
    float transmission, float thickness, float2x2 envmap_rotation_matrix) 
{
    if (transmission == 0.0f) return 0.0f;
    float t_roughness = sqrt(transmission_roughness(linear_roughness, mat.ior));
    if (!mat.hasVolume) {
        float2 uv = ndc_position.xy / g_rasterCB.FrameSize;
        float lod = t_roughness * (g_rasterCB.RenderFrameMips - 1);
        return (1.0f - Fresnel) * SampleFrameBicubic(uv, lod).rgb;
    }
    float3 transmitted_light = 0.0f;
    RefractResult refraction = SimulateRefraction(ws_pos, v, N, mat.ior, thickness); 
    if (any(refraction.dir != 0.0f)) {
        float3 refractedIBL = sampleBackgroundAndIBL(refraction.pos, refraction.dir, t_roughness, envmap_rotation_matrix);
        float3 attenuation = min(exp(-mat.attenuationFactor.rgb * refraction.dist), 1.0f);  // Volume absorption
        transmitted_light = (1.0f - Fresnel) * attenuation * refractedIBL;
    }
    return transmitted_light;
}

[shader("pixel")]
float4 PS_Main(PSInput input) : SV_TARGET {
    Material mat = Materials[DrawData.meshID];
    float2 uv = input.uv.xy;
    float4 albedo_color = sample_albedo_filtered(mat, uv, ASampler);
    float alpha = albedo_color.w;
    if (alpha < mat.alpha_cutoff)
        discard;  // alpha-test in the same shader for simplicity, but it disables early-z optimisation

    if (g_rasterCB.albedoOnlyMode) {
        return albedo_color;
    }

    const float rot_cos = g_rasterCB.envmap_rotation_cos;
    const float rot_sin = g_rasterCB.envmap_rotation_sin;
    const float2x2 envmap_rotation_matrix = float2x2(rot_cos, -rot_sin, rot_sin, rot_cos);

    float3 emissive = sample_emissive_filtered(mat, uv, ASampler);

    bool is_transmissive_material = (mat.transmisionFactor != 0.0f || mat.hasVolume);
    bool AO_SSR_Holdout_mask =
        (MaterialID.Load(int3(input.ndc_position.xy, 0)) == MaterialID::Transmissive) && !is_transmissive_material;

    float3 ORM = sample_occlusion_roughness_metallic_filtered(mat, uv, ASampler);
    float AO = ORM.x;
    if (mat.aoTextureIndex != mat.metallicRoughnessTextureIndex) {
        AO = sample_occlusion_filtered(mat, uv, ASampler);
    }
    AO = lerp(1.0, AO, mat.AOStrength);
    AO = lerp(1.0, AO, g_rasterCB.TexturesAOStrength);
    if (DrawData.UseAOTexture && g_rasterCB.GTAOStrength != 0.0f && !AO_SSR_Holdout_mask) {
        float GTAO = AmbientOcclusionTexture.Load(int3(input.ndc_position.xy, 0)).x;
        GTAO = lerp(1.0, GTAO, g_rasterCB.GTAOStrength);
        AO *= GTAO;
    }
    const float metallic = ORM.z;
    float3 diffuse_color = (1.0f - metallic) * albedo_color.rgb;

    float3 f0 = lerp(mat.dielectric_f0, albedo_color.rgb, metallic);
    const float3 f90 = 1.0f;

    float3 N = input.normal.xyz;
    float3 T = input.tangent.xyz;
    float tangent_sign = input.tangent.w;
    float3 B = cross(N, T) * tangent_sign;

    bool has_normal_map;
    float3 normal_map_color = sample_normals_filtered(mat, uv, ASampler, has_normal_map);

    float3x3 TBN = construct_TBN(T, B, N);
    N = has_normal_map ? normalize(normal_map_sample_to_world(normal_map_color, TBN)) : TBN[2]; // new normal after normal mapping

    float roughness = ORM.y;
    if (g_rasterCB.specular_aa_enabled) {
        roughness = SpecularAntialiasing(roughness, N, g_rasterCB.specular_aa_variance, g_rasterCB.specular_aa_threshold);
    }
    const float linear_roughness = roughness * roughness;

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

    float2 transmission_thickness = sample_transmission_thickness_filtered(mat, uv, ASampler);
    if (mat.thicknessTextureIndex != mat.transmissionTextureIndex) {
        transmission_thickness.g = sample_thickness_filtered(mat, uv, ASampler);
    }
    transmission_thickness.g *= DrawData.modelScale;
    float transmission = transmission_thickness.r;
    float3 transmissionIBL = calculateTransmittedLight(input.world_position, input.ndc_position, v, N, mat, Fresnel,
        linear_roughness, transmission, transmission_thickness.g, envmap_rotation_matrix).rgb;

    float3 DiffuseSampleDir = N;
    DiffuseSampleDir.xz = mul(envmap_rotation_matrix, DiffuseSampleDir.xz);  // enmap rotation
    float3 diffuseIBL = DiffuseLut.SampleLevel(Sampler, DiffuseSampleDir, 0).rgb;
    
    float3 SpecularSampleDir = DominantReflectionVector(l, N, linear_roughness);
    SpecularSampleDir.xz = mul(envmap_rotation_matrix, SpecularSampleDir.xz);  // enmap rotation
    float3 specularIBL = SpecularLut.SampleLevel(Sampler, SpecularSampleDir, roughness * (g_rasterCB.SpecularLutMips - 1)).rgb;

    if (g_rasterCB.SSREnabled && !AO_SSR_Holdout_mask) {
        float4 SSR_sample = SSRTexture.Load(int3(input.ndc_position.xy, 0));
        specularIBL = specularIBL * (1.0 - SSR_sample.a) + SSR_sample.rgb;
    }

    float3 diffuse_light = diffuse_color * lerp(AO * diffuseIBL, transmissionIBL, transmission);
    float3 final_light = lerp(diffuse_light, specularIBL, Fresnel) + emissive;

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

    const float rot_cos = g_rasterCB.envmap_rotation_cos;
    const float rot_sin = g_rasterCB.envmap_rotation_sin;
    const float2x2 envmap_rotation_matrix = float2x2(rot_cos, -rot_sin, rot_sin, rot_cos);

    dir.xz = mul(envmap_rotation_matrix, dir.xz);  // Rotate around Y-axis

    float3 SpecularIBL = SpecularLut.SampleLevel(Sampler, dir, 0).rgb;
    return float4(SpecularIBL, 1.0);
}

struct GBInput {
    float4 ndc_position : SV_POSITION;
	float4 ndc_position_curr : POSITION;
    float4 ndc_position_prev : POSITION1;
    float4 normal : NORMAL;
    float4 tangent : TANGENT;
    float4 uv : TEXCOORD;
};

struct GBOutput {
    float4 normal_roughness : SV_TARGET0;       // normal.xyz, roughness in alpha
    float3 velocity : SV_TARGET1;               // velocity in R16G16B16A16_FLOAT format
    uint id : SV_TARGET2;                       // material ID in R8_UINT format
};

[shader("vertex")] 
GBInput VS_Gbuffer(float4 position : POSITION, float4 normal : NORMAL, float4 tangent : TANGENT, float4 uv : TEXCOORD) {
    GBInput result;

    position.w = 1.0f;
    normal.w = 0.0f;

    float4 current_ws_pos = mul(DrawData.modelMatrix, position);
    float4 previous_ws_pos = mul(DrawData.modelMatrix_prev, position);

    float tangent_sign = tangent.w;
    tangent.w = 0.0f;

    result.ndc_position = mul(g_rasterCB.viewProjection, current_ws_pos);
	result.ndc_position_curr = result.ndc_position;
    result.ndc_position_prev = mul(g_rasterCB.viewProjection_prev, previous_ws_pos);

    result.normal = mul(DrawData.normalMatrix, normal);
    result.tangent = mul(DrawData.modelMatrix, tangent);
    result.normal.w = 0.0f;
    result.tangent.w = 0.0f;
    result.normal = mul(g_rasterCB.viewMatrix, result.normal);
    result.tangent = mul(g_rasterCB.viewMatrix, result.tangent);
    result.tangent.w = tangent_sign;
    result.uv = uv;

    return result;
}

[shader("pixel")] 
GBOutput PS_Gbuffer(GBInput input) : SV_TARGET {
    Material mat = Materials[DrawData.meshID];
    float2 uv = input.uv.xy;
    float alpha = sample_albedo_filtered(mat, uv, ASampler).w;
    if (alpha < mat.alpha_cutoff) discard;  // alpha-test in the same shader for simplicity, but it disables early-z optimisation

    float3 N = input.normal.xyz;
    float3 T = input.tangent.xyz;
    float tangent_sign = input.tangent.w;
    float3 B = cross(N, T) * tangent_sign;

    bool has_normal_map;
    float3 normal_map_color = sample_normals_filtered(mat, uv, ASampler, has_normal_map);

    float3x3 TBN = construct_TBN(T, B, N);
    N = has_normal_map ? normalize(normal_map_sample_to_world(normal_map_color, TBN))
                       : TBN[2];  // new normal after normal mapping

    float roughness = sample_occlusion_roughness_metallic_filtered(mat, uv, ASampler).g;
    if (g_rasterCB.specular_aa_enabled) {
        roughness = SpecularAntialiasing(roughness, N, g_rasterCB.specular_aa_variance, g_rasterCB.specular_aa_threshold);
    }

    bool is_transmissive_material = (mat.transmisionFactor != 0.0f || mat.hasVolume);

    // Compute velocity from NDC space positions
	float3 current_ndc = input.ndc_position_curr.xyz / input.ndc_position_curr.w;
    float3 previous_ndc = input.ndc_position_prev.xyz / input.ndc_position_prev.w;
    float3 velocity = current_ndc - previous_ndc;

    GBOutput result;
    result.normal_roughness = float4(N, roughness);
    result.velocity = velocity;
    result.id = is_transmissive_material ? MaterialID::Transmissive : MaterialID::Opaque;

    return result;
}
