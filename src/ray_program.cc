#include "ray_program.h"
#include "brdf.h"

namespace {
    using namespace glm;
    using namespace app;

    constexpr float kEpsilon = 1e-8f;
    constexpr float kMaxBRDF = 10.0f;
    
    fvec3 ImportanceSampleCosDir(fvec2 xi) {
        // pdf(x) = cos(l, n) / pi
        float cos_theta2 = 1.0f - xi.x;
        float cos_theta = sqrt(cos_theta2);
        float sin_theta = sqrt(xi.x);
        float phi = 2.0f * xi.y * pi<float>();

        float cos_phi = cos(phi);
        float sin_phi = sin(phi);

        return fvec3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
    }
    fvec3 importanceSampleGGX(fvec2 xi, float a)
    {
        // pdf(h) = D(h) * dot(n, h) : before conversion from half-vector to reflection vector
        // pdf(l) = D(h) * dot(n, h) / (4.0 * dot(l, h)) : after conversion to reflection vector
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
    float V_SmithGGXCorrelated(float NoV, float NoL, float a) { // a is alpha_linear_roughness = perceptual_roughness^2
        // Original formulation of G_SmithGGX Correlated
        // lambda_v = (-1 + sqrt ( alphaG2 * (1 - NdotL2 ) / NdotL2 + 1)) * 0.5 f;
        // lambda_l = (-1 + sqrt ( alphaG2 * (1 - NdotV2 ) / NdotV2 + 1)) * 0.5 f;
        // G_SmithGGXCorrelated = 1 / (1 + lambda_v + lambda_l );
        // V_SmithGGXCorrelated = G_SmithGGXCorrelated / (4.0 f * NdotL * NdotV );
        // This is the optimized version
        float a2 = a * a;
        float GGXV = NoL * sqrt((-NoV * a2 + NoV) * NoV + a2);
        float GGXL = NoV * sqrt((-NoL * a2 + NoL) * NoL + a2);
        return 2.0f / (GGXV + GGXL); // should be 0.5f / (GGXV + GGXL);
    }
    float G1(float NdW, float k)
    {
        return 1.0 / (NdW * (1.0 - k) + k);
    }
    // Schlick - Smith visibility term
    // [ http://blog.selfshadow.com/publications/s2013-shading-course/karis/s2013_pbs_epic_notes_v2.pdf ]
    float V_Schlick(float NoL, float NoV, float Roughness) // Roughness is perceptual roughness
    {
        float k = max(Roughness * Roughness * 0.5, 1e-5);
        return G1(NoL, k) * G1(NoV, k); // should be G1*G1 / 4.0;
    }
    fvec3 Tangent2World(fvec3 v, fvec3 T, fvec3 B, fvec3 N) { return T * v.x + B * v.y + N * v.z; }
    fvec3 Tangent2World(fvec3 v, const fmat3x3& TBN) { return TBN * v; }
    
    inline const Object& get_object_data(const Model& modelref, const ray_triangle_hit_info& hit) {
        return modelref.objects_[hit.objectIndex];
    }
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
    inline void convert_position_to_world_space(const Object& object, vertex& point) {
        // Transform position to world space; point is an in_out parameter
        point.position = xyz(object.ModelMatrix * xyz1(point.position));
    }
    inline fvec3 convert_normals_to_world_space(const Object& object, vertex& point) {
        // Transform normal and tangent to world space; point is an in_out parameter; return bitangent
        auto tangent_sign = point.tangent.w;
        point.tangent.w = 0.0f;
        point.tangent = object.ModelMatrix * point.tangent;
        point.normal = xyz(object.NormalMatrix * xyz0(point.normal));
        return cross(point.normal, xyz(point.tangent)) * tangent_sign;
    }
    inline fvec3 get_geometric_normal(const vertex& p1, const vertex& p2, const vertex& p3, 
        const bool double_sided_material = false, const bool backface_hit = false) {
        auto geometric_normal = normalize(cross(
            p2.position - p1.position,
            p3.position - p1.position));
        if (double_sided_material && backface_hit) {
            geometric_normal *= -1.0f;
        }
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
    fmat3x3 handle_TBN_creation(
        const Object& object,
        const fvec4& normal_map_color,
        const vertex& point, const fvec3& bitangent, const fvec3& v, 
        const bool double_sided_material, const bool exiting_volume, const bool backface_hit,
        const vertex& p1, const vertex& p2, const vertex& p3) 
    {
        // todo: potential problem with self-intersection from interpolated normal and geometry normal mismatch in certain cases,
        // unrelated to impossible normal angle (in relation to v)
        fmat3x3 TBN = construct_TBN(xyz(point.tangent), bitangent, point.normal);
        const bool has_normal_map = (normal_map_color.w != 0.0f);
        fvec3 normal_vector = has_normal_map ? normal_map_sample_to_world(normal_map_color, TBN) : TBN[2];
        const bool impossible_normal_angle = (dot((!exiting_volume) ? v : -v, normal_vector) < 0.0);
        if (impossible_normal_angle) {
            normal_vector = get_geometric_normal(p1, p2, p3, double_sided_material, backface_hit); // todo: somehow incorporate normal map into geometric normal?
            normal_vector = xyz(object.NormalMatrix * xyz0(normal_vector));
        }
        if (has_normal_map || impossible_normal_angle) {
            TBN = construct_TBN(TBN[0], TBN[1], normal_vector); // re-construct TBN with normal from normal map
        }
        return TBN;
    }
    float pow2(float v) {
        return v * v;
    }
}

namespace app {
    RayCasterProgram::RayCasterProgram(const Model& model, const CPUTexture<hdr_pixel>& env) 
        : modelRef(model), envmapRef(env) {}

    fvec3 RayCasterProgram::on_hit(const ray_with_payload& ray_, const ray_triangle_hit_info& hit, std::vector<ray_with_payload>& ray_collection) const
    {
        if (hit.forward_hit() == false) { // on miss
            return xyz(sample_environment(ray_.direction, envmapRef));
        }
        const auto& mesh_data = get_mesh_data(modelRef, hit);
        const auto& [p1, p2, p3] = get_vertex_data(mesh_data, hit);
        auto uv = p1.uv * hit.b_coords.A + p2.uv * hit.b_coords.B + p3.uv * hit.b_coords.C();

        auto mat_index = mesh_data.materialIndex;
        const auto& material = modelRef.materials_[mat_index];
        auto albedo_color = sample_albedo(material, modelRef.images_, uv);
        return xyz(albedo_color);
    }
    // RayCasterProgram

    AOProgram::AOProgram(const Model& model, const CPUTexture<hdr_pixel>& env, const unsigned int ao_samples)
        : modelRef(model), envmapRef(env), aoSamples(ao_samples), inv_aoSamples(1.0f / aoSamples) {
    }

    std::minstd_rand thread_local AOProgram::gen = std::minstd_rand(std::random_device{}());
    std::uniform_real_distribution<float> thread_local AOProgram::dist = std::uniform_real_distribution<float>(0.0f, 1.0f);

    fvec3 AOProgram::on_hit(const ray_with_payload& ray_, const ray_triangle_hit_info& hit, std::vector<ray_with_payload>& ray_collection) const
    {
        if (hit.forward_hit() == false) { // on miss
            return ray_.payload;
        }
        if (ray_.depth == 0) {
            return fvec3(0.0f);
        }
        const auto& object = get_object_data(modelRef, hit);
        const auto& mesh_data = get_mesh_data(modelRef, hit);
        const auto& [p1, p2, p3] = get_vertex_data(mesh_data, hit);
        auto point = interpolate_vertex_data(p1, p2, p3, hit);

        auto bitangent = convert_normals_to_world_space(object, point);

        auto mat_index = mesh_data.materialIndex;
        const auto& material = modelRef.materials_[mat_index];

        if (hit.b_coords.backface) {
            // ray hits backside
            if (material.doubleSided) {
                point.normal *= -1.0f;
            }
        }

        fmat3x3 TBN = construct_TBN(xyz(point.tangent), bitangent, point.normal);

        auto normal_map_color = sample_normals(material, modelRef.images_, point.uv);
        if (normal_map_color.w != 0.0f) {
            fvec3 normal_vector = normal_map_sample_to_world(normal_map_color, TBN);
            TBN = construct_TBN(TBN[0], TBN[1], normal_vector); // re-construct TBN with normal from normal map
        }
        
        auto ws_pos = ray_.origin + ray_.direction * hit.distance;
        auto new_pos = ws_pos + point.normal * 1e-5f; // offset to avoid self-intersection

        std::uint8_t new_depth = ray_.depth - 1;

        auto jitter_value_x = dist(gen);
        auto jitter_value_y = dist(gen);
        for (int i = 0; i < aoSamples; ++i) {
            fvec2 rand = fibonacci2D(i, inv_aoSamples); // quasi-random sampling
            rand.x = std::fmod(rand.x + jitter_value_x, 1.0f); // jitter
            rand.y = std::fmod(rand.y + jitter_value_y, 1.0f); // jitter
            auto new_direction = ImportanceSampleCosDir(rand);
            assert(new_direction.z > 0.0f);
            new_direction = Tangent2World(new_direction, TBN);
            assert(abs(length(new_direction) - 1.0f) < 1e-5f);
            ray_with_payload new_ray{ new_pos, new_direction, fvec4(inv_aoSamples), new_depth, true };
            ray_collection.push_back(new_ray);
        }
        
        return fvec3(0.0f);
    }
    // AOProgram
    
    PBRProgram::PBRProgram(const Model& model, const CPUTexture<hdr_pixel>& env)
        : modelRef(model), envmapRef(env) {}

    std::minstd_rand thread_local PBRProgram::gen = std::minstd_rand(std::random_device{}());
    std::uniform_real_distribution<float> thread_local PBRProgram::dist = std::uniform_real_distribution<float>(0.0f, 1.0f);

    fvec3 PBRProgram::on_hit(const ray_with_payload& ray_, const ray_triangle_hit_info& hit, std::vector<ray_with_payload>& ray_collection) const
    {
        if (hit.forward_hit() == false) { // on miss
            return ray_.payload * xyz(sample_environment(ray_.direction, envmapRef));
        }
        const auto& object = get_object_data(modelRef, hit);
        const auto& mesh_data = get_mesh_data(modelRef, hit);
        const auto& [p1, p2, p3] = get_vertex_data(mesh_data, hit);
        auto point = interpolate_vertex_data(p1, p2, p3, hit);

        convert_position_to_world_space(object, point);
        auto bitangent = convert_normals_to_world_space(object, point);

        auto mat_index = mesh_data.materialIndex;
        const auto& material = modelRef.materials_[mat_index];

        bool exiting_volume = false;
        if (hit.b_coords.backface) {
            // ray hits backside
            if (material.doubleSided) {
                point.normal *= -1.0f;
            } else if (material.hasVolume) { // according to glTF spec, volume is only for single-sided materials
                exiting_volume = true;
            }
        }

        auto albedo_color = sample_albedo(material, modelRef.images_, point.uv);
        float alpha = albedo_color.w;
        if (!material.alphaBlending) {
            alpha = (alpha < material.alpha_cutoff) ? 0.0f : 1.0f;
        }
        if (alpha != 1.0f) {
            ray_with_payload new_ray = ray_;
            new_ray.origin = point.position + (exiting_volume ? 1.0f : -1.0f) * point.normal * 1e-5f; // offset to avoid self-intersection
            new_ray.payload *= (1.0f - alpha);
            ray_collection.push_back(new_ray);
        }
        if (alpha == 0.0f) {
            return fvec3{ 0.0f };
        }

        auto emissive = xyz(sample_emissive(material, modelRef.images_, point.uv));
        if (ray_.depth == 0) {
            return ray_.payload * emissive * alpha;
        }

        auto v = -ray_.direction;

        auto normal_map_color = sample_normals(material, modelRef.images_, point.uv);
        fmat3x3 TBN = handle_TBN_creation(
            object,
            normal_map_color,
            point, bitangent, v,
            material.doubleSided, exiting_volume, hit.b_coords.backface,
            p1, p2, p3);

        auto transmission = sample_transmission(material, modelRef.images_, point.uv);
        auto ORM = sample_roughness_metallic(material, modelRef.images_, point.uv);
        auto diffuse_color = (1.0f - ORM.z) * xyz(albedo_color);

        auto f0 = mix(fvec3(material.dielectric_f0), xyz(albedo_color), ORM.z);
        const auto f90 = fvec3(1.0f);
        const auto roughness = ORM.y;
        const auto linear_roughness = roughness * roughness;

        std::uint8_t new_depth = ray_.depth - 1;

        std::array<float, 4> random_values;
        std::generate(random_values.begin(), random_values.end(), [this]() { return dist(gen); });

        const bool sample_diffuse = (diffuse_color * (1.0f - transmission) != fvec3(0.0f));
        if (sample_diffuse) { // diffuse
            fvec2 rand = fvec2{ random_values[0], random_values[1]};

            auto l = ImportanceSampleCosDir(rand);
            assert(l.z > 0.0f);
            l = normalize(Tangent2World(l, TBN)); // normalizing for better accuracy
            assert(abs(length(l) - 1.0f) < 1e-5f);
            
            const auto h = normalize(v + l);
            float LdH = clamp(dot(l, h), 0.0f, 1.0f);
            
            auto F = fvec3(1.0f) - fresnel_schlick(f0, f90, LdH);
            auto new_pos = point.position + point.normal * 1e-5f; // offset to avoid self-intersection
            auto new_payload = F * diffuse_color * (1.0f - transmission) * ray_.payload * alpha;
            ray_with_payload new_ray{ new_pos, l, new_payload, new_depth, false };
            ray_collection.push_back(new_ray);
        }
        {
            //same micro-normal for both specular reflection and transmission
            fvec2 rand = fvec2{ random_values[2], random_values[3] };
            auto m = importanceSampleGGX(rand, linear_roughness);
            assert(m.z > 0.0f);
            m = Tangent2World(m, TBN);
            assert(abs(length(m) - 1.0f) < 1e-5f);

            auto N = TBN[2];
            auto new_pos_offset_dir = point.normal;
            if (exiting_volume) { 
                m *= -1.0f;
                N *= -1.0f;
                new_pos_offset_dir *= -1.0f;
            }

            const float interface_ior = (!exiting_volume) ? 1.0f / material.ior : material.ior;
            auto l = refract(-v, m, interface_ior);

            const float VdN = clamp(dot(N, v), kEpsilon, 1.0f);
            const auto VdM = clamp(dot(v, m), 0.0f, 1.0f);
            const auto LdM = clamp(dot(l, -m), 0.0f, 1.0f);

            auto F = fresnel_schlick(f0, f90, (!exiting_volume) ? VdM : LdM);

            const bool sample_transmission = (diffuse_color * transmission != fvec3(0.0f) && (l != fvec3(0.0f)));
            if (sample_transmission) { // transmission
                assert(abs(length(l) - 1.0f) < 1e-5f);
                auto h = -(interface_ior * v + l); // transmission half-vector; 
                                                   //minus is because normal points into into the medium with the lower index of refraction (e.g., air). (convention)
                h = normalize(h);
                float LdN = clamp(dot(-N, l), 0.0f, 1.0f);
                auto G = V_SmithGGXCorrelated(VdN, LdN, linear_roughness);
                //auto G = V_Schlick(LdN, VdN, roughness);

                float NdM = clamp(dot(N, m), kEpsilon, 1.0f);
                float VdH = clamp(dot((!exiting_volume) ? v : -v, h), 0.0f, 1.0f);

                auto brdf = (fvec3(1.0f) - F) * (G * VdH * LdN / NdM);
                brdf = min(brdf, fvec3(kMaxBRDF)); // clamp to avoid fireflies
                auto new_pos = point.position - new_pos_offset_dir * 1e-5f; // offset to avoid self-intersection
                auto new_payload = diffuse_color * transmission * brdf * ray_.payload * alpha;
                ray_with_payload new_ray{ new_pos, normalize(l), new_payload, new_depth, false }; // normalizing for better accuracy
                ray_collection.push_back(new_ray);
            }
            { // specular reflection
                auto h = m;
                assert(abs(length(h) - 1.0f) < 1e-5f);

                l = normalize(reflect(-v, h)); // normalizing for better accuracy
                float LdH = clamp(dot(l, h), 0.0f, 1.0f);
                float LdN = clamp(dot(N, l), 0.0f, 1.0f);
                float NdH = clamp(dot(N, h), kEpsilon, 1.0f);

                auto G = V_SmithGGXCorrelated(VdN, LdN, linear_roughness);
                //auto G = V_Schlick(LdN, VdN, roughness);
                auto brdf = F * (G * LdN * LdH / NdH);
                brdf = min(brdf, fvec3(kMaxBRDF)); // clamp to avoid fireflies
                auto new_pos = point.position + new_pos_offset_dir * 1e-5f; // offset to avoid self-intersection
                auto new_payload = brdf * ray_.payload * alpha;
                ray_with_payload new_ray{ new_pos, l, new_payload, new_depth, false };
                ray_collection.push_back(new_ray);
            }
        }
        return ray_.payload * emissive * alpha;
    }
    
}