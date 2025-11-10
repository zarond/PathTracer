#include "ray_program.h"
#include "brdf.h"

namespace {
    using namespace glm;
    using namespace app;

    constexpr float GOLDEN_RATIO = 1.618034f;
    constexpr float kEpsilon = 1e-5f;
    
    float fibonacci1D(int i)
    {
        return std::fmod((static_cast<float>(i) + 1.0f) * GOLDEN_RATIO, 1.0f);
    }
    fvec2 fibonacci2D(int i, int nbSamples)
    {
        return fvec2(
            (static_cast<float>(i) + 0.5f) / static_cast<float>(nbSamples),
            fibonacci1D(i)
        );
    }
    fvec3 ImportanceSampleCosDir(fvec2 xi) {
        float cos_theta2 = 1.0f - xi.x;
        float cos_theta = sqrt(cos_theta2);
        float sin_theta = sqrt(xi.x);
        float phi = 2.0f * xi.y * pi<float>();

        float cos_phi = cos(phi);
        float sin_phi = sin(phi);

        return fvec3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
    }
    fvec3 importanceSampleGGX(fvec2 xi, float roughness)
    {
        float a = roughness * roughness;
        float cos_theta2 = (1.0f - xi.x) / (1.0f + (a * a - 1.0f) * xi.x);
        float cos_theta = sqrt(cos_theta2);
        float sin_theta = sqrt(1.0f - cos_theta2);
        float phi = 2.0f * xi.y * pi<float>();

        float cos_phi = cos(phi);
        float sin_phi = sin(phi);
        return fvec3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
    }
    fvec3 fresnel_schlick(fvec3 f0, fvec3 f90, float cos_nv) {
        float x = 1.0f - cos_nv;
        float x2 = x * x;
        float x5 = x2 * x2 * x;
        return mix(f0, f90, x5);
    }
    float V_SmithGGXCorrelated(float NoV, float NoL, float a) {
        // Original formulation of G_SmithGGX Correlated
        // lambda_v = (-1 + sqrt ( alphaG2 * (1 - NdotL2 ) / NdotL2 + 1)) * 0.5 f;
        // lambda_l = (-1 + sqrt ( alphaG2 * (1 - NdotV2 ) / NdotV2 + 1)) * 0.5 f;
        // G_SmithGGXCorrelated = 1 / (1 + lambda_v + lambda_l );
        // V_SmithGGXCorrelated = G_SmithGGXCorrelated / (4.0 f * NdotL * NdotV );
        // This is the optimized version
        float a2 = a * a;
        float GGXV = NoL * sqrt((-NoV * a2 + NoV) * NoV + a2);
        float GGXL = NoV * sqrt((-NoL * a2 + NoL) * NoL + a2);
        return 0.5f / (GGXV + GGXL);
    }
    float pow2(float v) {
        return v * v;
    }
    float f0_dielectric(float ior) {
        return pow2((ior - 1.0f) / (ior + 1.0f));
    }
    fvec3 Tangent2World(fvec3 v, fvec3 T, fvec3 B, fvec3 N) { return T * v.x + B * v.y + N * v.z; }
    fvec3 Tangent2World(fvec3 v, const fmat3x3& TBN) { return TBN * v; }
    
    inline const Mesh& get_mesh_data(const Model& modelref, const ray_triangle_hit_info& hit) {
        return modelref.meshes_[hit.meshIndex];
    }
    inline std::tuple<vertex, vertex, vertex> get_vertex_data(const Mesh& mesh_data, const ray_triangle_hit_info& hit) {
        auto p1 = mesh_data.indices[hit.triangleIndex];
        auto p2 = mesh_data.indices[hit.triangleIndex + 1];
        auto p3 = mesh_data.indices[hit.triangleIndex + 2];
        return std::tuple{ mesh_data.vertices[p1], mesh_data.vertices[p2], mesh_data.vertices[p3] };
    }
    inline vertex interpolate_vertex_data(const vertex& p1, const vertex& p2, const vertex& p3, const ray_triangle_hit_info& hit) {
        auto C = hit.b_coords.C();
        auto os_position = p1.position * hit.b_coords.A +
            p2.position * hit.b_coords.B +
            p3.position * C;
        auto os_normal = p1.normal * hit.b_coords.A +
            p2.normal * hit.b_coords.B +
            p3.normal * C;
        auto os_tangent = p1.tangent * hit.b_coords.A +
            p2.tangent * hit.b_coords.B +
            p3.tangent * C;
        auto uv = p1.uv * hit.b_coords.A +
            p2.uv * hit.b_coords.B +
            p3.uv * C;
        return vertex{ os_position, os_normal, os_tangent, uv };
    }
    inline fvec3 get_geometric_normal(const vertex& p1, const vertex& p2, const vertex& p3) {
        auto geometric_normal = normalize(cross(
            p2.position - p1.position,
            p3.position - p1.position));
        return geometric_normal;
    }
    inline fmat3x3 construct_TBN(const fvec3& tangent, const fvec3& bitangent, const fvec3& normal) {
        auto n = normalize(normal);
        auto t = normalize(tangent - n * dot(n, tangent));
        auto b = normalize(bitangent - n * dot(n, bitangent) - t * dot(t, bitangent));
        return fmat3x3( t, b, n );
    }
    inline fvec3 normal_map_sample_to_world(const fvec3& normal_map_sample, const fmat3x3& TBN) {
        fvec3 n_ts = glm::fma(normal_map_sample, fvec3(2.0f) , fvec3(-1.0f));
        n_ts.y *= -1.0f; // flip Y
        return TBN * n_ts;
    }
}

namespace app {
    RayCasterProgram::RayCasterProgram(const Model& model, const CPUTexture<hdr_pixel>& env) 
        : modelRef(model), envmapRef(env) {}

    fvec4 RayCasterProgram::on_hit(const ray_with_payload& ray_, const ray_triangle_hit_info& hit, std::vector<ray_with_payload>& ray_collection) const
    {
        if (hit.forward_hit() == false) { // on miss
            return sample_environment(ray_.direction, envmapRef);
        }
        const auto& mesh_data = get_mesh_data(modelRef, hit);
        const auto& [p1, p2, p3] = get_vertex_data(mesh_data, hit);
        auto uv = p1.uv * hit.b_coords.A + p2.uv * hit.b_coords.B + p3.uv * hit.b_coords.C();

        auto mat_index = mesh_data.materialIndex;
        auto albedo_color = sample_albedo(modelRef.materials_[mat_index], modelRef.images_, uv);
        return albedo_color;
    }
    // RayCasterProgram

    AOProgram::AOProgram(const Model& model, const CPUTexture<hdr_pixel>& env, const unsigned int ao_samples)
        : modelRef(model), envmapRef(env), aoSamples(ao_samples) {
    }

    std::minstd_rand thread_local AOProgram::gen = std::minstd_rand(std::random_device{}());
    std::uniform_real_distribution<float> thread_local AOProgram::dist = std::uniform_real_distribution<float>(0.0f, 1.0f);

    fvec4 AOProgram::on_hit(const ray_with_payload& ray_, const ray_triangle_hit_info& hit, std::vector<ray_with_payload>& ray_collection) const
    {
        if (hit.forward_hit() == false) { // on miss
            return ray_.payload;
        }
        if (ray_.depth == 0) {
            return fvec4(0.0f);
        }
        const auto& mesh_data = get_mesh_data(modelRef, hit);
        const auto& [p1, p2, p3] = get_vertex_data(mesh_data, hit);
        auto point = interpolate_vertex_data(p1, p2, p3, hit);

        auto tangent_sign = point.tangent.w;
        point.tangent = fvec4(fmat3(modelRef.objects_[hit.objectIndex].ModelMatrix) * fvec3(point.tangent), tangent_sign);
        point.normal = fvec3(fmat3(modelRef.objects_[hit.objectIndex].NormalMatrix) * point.normal);
        auto bitangent = cross(point.normal, fvec3(point.tangent)) * point.tangent.w;

        fmat3x3 TBN = construct_TBN(fvec3(point.tangent), bitangent, point.normal);

        auto mat_index = mesh_data.materialIndex;
        auto normal_map_color = sample_normals(modelRef.materials_[mat_index], modelRef.images_, point.uv);
        if (normal_map_color.w != 0.0f) {
            fvec3 normal_vector = normal_map_sample_to_world(normal_map_color, TBN);
            TBN = construct_TBN(TBN[0], TBN[1], normal_vector); // re-construct TBN with normal from normal map
        }
        
        auto ws_pos = ray_.origin + ray_.direction * hit.distance;

        std::uint8_t new_depth = ray_.depth - 1;

        auto jitter_value_x = dist(gen);
        auto jitter_value_y = dist(gen);
        for (int i = 0; i < aoSamples; ++i) {
            fvec2 rand = fibonacci2D(i, aoSamples);
            rand.x = std::fmod(rand.x + jitter_value_x, 1.0f); // jitter
            rand.y = std::fmod(rand.y + jitter_value_y, 1.0f); // jitter
            auto new_direction = ImportanceSampleCosDir(rand);
            assert(new_direction.z > 0.0f);
            assert(abs(length(new_direction) - 1.0f) < 1e-5f);
            new_direction = Tangent2World(new_direction, TBN);
            auto new_pos = ws_pos + point.normal * 1e-5f; // offset to avoid self-intersection
            ray_with_payload new_ray{ new_pos, normalize(new_direction), fvec4(1.0f/ aoSamples), new_depth, true };
            ray_collection.push_back(new_ray);
        }
        
        return fvec4(0.0f);
    }
    // AOProgram
    
    PBRProgram::PBRProgram(const Model& model, const CPUTexture<hdr_pixel>& env, const unsigned int max_new_rays_)
        : modelRef(model), envmapRef(env), max_new_rays(max_new_rays_) {
    }

    std::minstd_rand thread_local PBRProgram::gen = std::minstd_rand(std::random_device{}());
    std::uniform_real_distribution<float> thread_local PBRProgram::dist = std::uniform_real_distribution<float>(0.0f, 1.0f);

    fvec4 PBRProgram::on_hit(const ray_with_payload& ray_, const ray_triangle_hit_info& hit, std::vector<ray_with_payload>& ray_collection) const
    {
        // Todo: implement full path tracer
        if (hit.forward_hit() == false) { // on miss
            return ray_.payload * sample_environment(ray_.direction, envmapRef);
        }
        const auto& mesh_data = get_mesh_data(modelRef, hit);
        const auto& [p1, p2, p3] = get_vertex_data(mesh_data, hit);
        auto point = interpolate_vertex_data(p1, p2, p3, hit);

        auto tangent_sign = point.tangent.w;
        point.tangent = fvec4(fmat3(modelRef.objects_[hit.objectIndex].ModelMatrix) * fvec3(point.tangent), tangent_sign);
        point.normal = fvec3(fmat3(modelRef.objects_[hit.objectIndex].NormalMatrix) * point.normal);
        auto bitangent = cross(point.normal, fvec3(point.tangent)) * point.tangent.w;

        fmat3x3 TBN = construct_TBN(fvec3(point.tangent), bitangent, point.normal);

        auto mat_index = mesh_data.materialIndex;
        const auto& material = modelRef.materials_[mat_index];
        auto normal_map_color = sample_normals(material, modelRef.images_, point.uv);
        if (normal_map_color.w != 0.0f) {
            fvec3 normal_vector = normal_map_sample_to_world(normal_map_color, TBN);
            TBN = construct_TBN(TBN[0], TBN[1], normal_vector); // re-construct TBN with normal from normal map
        }

        auto emissive = sample_emissive(material, modelRef.images_, point.uv);
        if (ray_.depth == 0) {
            return ray_.payload * emissive;
        }
        auto albedo_color = sample_albedo(material, modelRef.images_, point.uv);
        auto ORM = sample_roughness_metallic(material, modelRef.images_, point.uv);
        auto diffuse_color = (1.0f - ORM.z) * albedo_color;
        //float f0_diel = f0_dielectric(material.ior);
        const float f0_diel = 0.04f;
        auto f0 = mix(fvec3(f0_diel), fvec3(albedo_color), ORM.z);
        const auto f90 = fvec3(1.0f);
        const auto roughness = ORM.y;

        const bool no_diffuse = (diffuse_color.r < 1e-5f) && (diffuse_color.g < 1e-5f) && (diffuse_color.b < 1e-5f);
        const unsigned int diffuse_rays_n = (!no_diffuse) ? 1 + max_new_rays / 2 : 0;
        const unsigned int specular_rays_n = 1 + max_new_rays - diffuse_rays_n;

        auto ws_pos = ray_.origin + ray_.direction * hit.distance;

        std::uint8_t new_depth = ray_.depth - 1;

        auto jitter_value_x = dist(gen);
        auto jitter_value_y = dist(gen);
        for (int i = 0; i < diffuse_rays_n; ++i) {
            fvec2 rand = fibonacci2D(i, diffuse_rays_n);
            rand.x = std::fmod(rand.x + jitter_value_x, 1.0f); // jitter
            rand.y = std::fmod(rand.y + jitter_value_y, 1.0f); // jitter
            auto l = ImportanceSampleCosDir(rand);
            
            const auto v = -ray_.direction;
            const auto h = normalize(v + l);
            float LdH = dot(l, h);

            assert(l.z > 0.0f);
            assert(abs(length(l) - 1.0f) < 1e-5f);
            l = normalize(Tangent2World(l, TBN));
            assert(abs(length(l) - 1.0f) < 1e-5f);
            auto new_pos = ws_pos + point.normal * 1e-5f; // offset to avoid self-intersection
            
            auto F = fvec3(1.0f) - fresnel_schlick(f0, f90, LdH);
            ray_with_payload new_ray{ new_pos, l, fvec4(F, 1.0f) * diffuse_color * ray_.payload * fvec4(1.0f / diffuse_rays_n), new_depth, false };
            ray_collection.push_back(new_ray);
        }
        const auto N = TBN[2];
        for (int i = 0; i < specular_rays_n; ++i) {
            fvec2 rand = fibonacci2D(i, specular_rays_n);
            rand.x = std::fmod(rand.x + jitter_value_x, 1.0f); // jitter
            rand.y = std::fmod(rand.y + jitter_value_y, 1.0f); // jitter
            auto d_N = importanceSampleGGX(rand, roughness);
            assert(d_N.z > 0.0f);
            assert(abs(length(d_N) - 1.0f) < 1e-5f);
            d_N = normalize(Tangent2World(d_N, TBN));
            assert(abs(length(d_N) - 1.0f) < 1e-5f);

            const auto v = -ray_.direction;
            const auto l = reflect( -v, d_N);
            const auto h = normalize(v + l);
            float LdH = clamp(dot(l, h), kEpsilon, 1.0f);
            float LdN = clamp(dot(N, l), kEpsilon, 1.0f);
            float VdN = clamp(dot(N, v), kEpsilon, 1.0f);
            float VdH = clamp(dot(v, h), kEpsilon, 1.0f);
            float NdH = clamp(dot(N, h), kEpsilon, 1.0f);

            auto new_pos = ws_pos + point.normal * 1e-5f; // offset to avoid self-intersection
            auto F = fresnel_schlick(f0, f90, LdH);
            auto G = V_SmithGGXCorrelated(VdN, LdN, roughness);
            //auto brdf = fvec4(F * VdH / (VdN * NdH), 1.0f); // simplified version without G
            auto brdf = fvec4(F * (4.0f * G * LdN * VdH / NdH), 1.0f);
            ray_with_payload new_ray{ new_pos, l, brdf * ray_.payload * fvec4(1.0f / specular_rays_n), new_depth, false };
            ray_collection.push_back(new_ray);
        }

        return ray_.payload * emissive;
    }
    
}