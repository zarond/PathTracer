#include "Common.hlsl"
#include "BRDF.hlsl"

enum LightType { 
    Directional = 0,
    Spot = 1,
    Point = 2,
};
    
static const float MIN_PERCEPTUAL_ROUGHNESS = 0.045;
static const float MIN_ROUGHNESS = 0.002025;
static const float MIN_POINT_LIGHT_DISTANCE_SQUARED = 0.00001;

struct MaterialParams {
    float3 diffuse_color; // diffuse color already with transmission factor applied
    float linear_roughness;
    float3 f0;
    float transmission_roughness;
    float3 transmission_attenuation; // with transmission factor and albedo applied
};

struct LightRay {
    float3 color;
    float3 direction;
};
    
float compute_shadowing(const ShadowMappingInfo shadow_info, const float3 pos, const float3 n, 
    const SamplerState Sampler) 
{
    Texture2D<float4> shadowTex = ResourceDescriptorHeap[shadow_info.shadowMapTextureIndex_type_padding.x];
    int light_type = shadow_info.shadowMapTextureIndex_type_padding.y;
    
    float4 ls_ndc = mul(shadow_info.viewProjection, float4(pos, 1.0f));
        
    bool in_frustum = ls_ndc.w > 0.0f
        && abs(ls_ndc.x) <= ls_ndc.w
        && abs(ls_ndc.y) <= ls_ndc.w
        && ls_ndc.z >= 0.0f
        && ls_ndc.z <= ls_ndc.w;

    if (!in_frustum) return 1.0f;

    ls_ndc /= ls_ndc.w;
    float2 uv = ls_ndc.xy * float2(0.5f, -0.5f) + 0.5f;
    
    float4 shadow_sample = shadowTex.SampleLevel(Sampler, uv, 0);
    float3 vec = pos - shadow_info.origin.xyz;
    float vec_length;
    if (light_type == LightType::Directional) {
        vec_length = dot(vec, shadow_info.direction.xyz);
    } else {
        vec_length = length(vec);
    }
        
    if (light_type == LightType::Point) return 1.0f; // TODO: do cubemap shadows
        
    const float bias = 0.01f; // TODO: adaptive bias, proper VS shadow maps
    if (vec_length > shadow_sample.x + bias) return 0.0f;
    return 1.0f;
}

float3 lambertian_BRDF(float3 diffuseColor) {
    return diffuseColor * inv_PI;
}

float spot_attenuation(float3 light_direction, float3 direction_to_light, float2 inner_cone_angle_outer_cone_angle){
    float b2 = inner_cone_angle_outer_cone_angle.x;
    float b1 = inner_cone_angle_outer_cone_angle.y;
    float d_cos = dot(light_direction, direction_to_light);
    return smoothstep(b1,b2,d_cos);
}

LightRay convert_to_light_ray(const float3 pos, const Light light) {
    LightRay light_ray;
    
    light_ray.color = light.color_intensity.rgb * light.color_intensity.a;
    
    int light_type = int(light.direction_type.w);
    if (light_type == LightType::Directional) {
        light_ray.direction = light.direction_type.xyz;
        return light_ray;
    }
        
    float3 ray_pl = light.position_range.xyz - pos;
    float ray_pl_sq_magnitude = dot(ray_pl,ray_pl);
    float ray_pl_magnitude_inv = rsqrt(ray_pl_sq_magnitude);
        
    light_ray.direction = (ray_pl_sq_magnitude > 0)? ray_pl * ray_pl_magnitude_inv : float3(0.0f, 0.0f, 0.0f);
        
    float falloff = 1.0f / max(ray_pl_sq_magnitude, MIN_POINT_LIGHT_DISTANCE_SQUARED); // square distance fallof
        
    if (light.position_range.w > 0.0f) {    // range falloff
        float factor = ray_pl_sq_magnitude / (light.position_range.w * light.position_range.w);
        factor = saturate(1.0f - factor * factor);
        falloff *= factor; // or factor * factor for smoother falloff?
    }
    if (light_type == LightType::Spot) {    // spotlight falloff
        falloff *= spot_attenuation(light.direction_type.xyz, light_ray.direction, light.inner_outerConeAngleCos.xy);
    }
    
    light_ray.color *= falloff;
    return light_ray;
}
    
float3 directional_lighting(const float3 pos, const float3 n, const float3 v, float3 refracted_v, 
    MaterialParams mat_params, const LightRay light_ray, const bool has_transmission) 
{
    const float3 f90 = 1.0f;
    float3 h = normalize(v + light_ray.direction);
    float VdH = saturate(dot(v, h));
    float VdN = saturate(dot(v, n));
    float LdN = saturate(dot(n, light_ray.direction));
    float NdH = saturate(dot(n, h));
    float3 F = fresnel_schlick(mat_params.f0, f90, VdH);
    
    mat_params.linear_roughness = max(mat_params.linear_roughness, MIN_ROUGHNESS); // change roughness to prevent division by zero
    
    float G = V_SmithGGXCorrelated(VdN, LdN, mat_params.linear_roughness);
    float D = D_GGX(NdH, mat_params.linear_roughness);
    
    float3 diffuse_brdf = (1.0f - F) * lambertian_BRDF(mat_params.diffuse_color);
    float3 specular_brdf = (D * G) * F;
    
    float3 R = (diffuse_brdf + specular_brdf) * LdN * light_ray.color;
    
    if (!has_transmission) {
        return R;
    }
    mat_params.transmission_roughness = max(mat_params.transmission_roughness, MIN_ROUGHNESS);
    
    refracted_v = -reflect(refracted_v, n);
    
    h = normalize(refracted_v + light_ray.direction);
    VdH = saturate(dot(refracted_v, h));
    NdH = saturate(dot(-n, h));
    LdN = saturate(dot(-n, light_ray.direction));
    VdN = saturate(dot(-n, refracted_v));
    
    F = fresnel_schlick(mat_params.f0, f90, VdH);
    G = V_SmithGGXCorrelated(VdN, LdN, mat_params.transmission_roughness);
    D = D_GGX(NdH, mat_params.transmission_roughness);
    
    float3 transmittance_brdf = (D * G) * (1.0 - F) * (1.0 - F) * mat_params.transmission_attenuation;
    R += transmittance_brdf * LdN * light_ray.color;
    
    return R;
}

float3 compute_punctual_light_contribution(const float3 pos, const float3 n, const float3 v, const float3 refracted_v, 
    MaterialParams mat_params, const Light light, const ShadowMappingInfo shadow_info, const SamplerState Sampler) 
{
    LightRay light_ray = convert_to_light_ray(pos, light);
        
    bool has_transmission = any(mat_params.transmission_attenuation > 0.0f) && any(refracted_v != 0.0f);
    if (!has_transmission) {
        light_ray.color = (dot(n, light_ray.direction) > 0) ? light_ray.color : 0.0f; // facing attenuation for opaque materials
    }
    if (all(light_ray.color == 0.0f)) {
        return 0.0f; // early exit if light is not contributing
    }
    float shadowing_factor = compute_shadowing(shadow_info, pos, n, Sampler);
    if (shadowing_factor == 0.0f) { // ??? what about if it has transmission?
        return 0.0f; // early exit if fully in shadow
    }
        
    float3 Radiance = directional_lighting(pos, n, v, refracted_v, mat_params, light_ray, has_transmission);
    return Radiance * shadowing_factor;
}
    