#include "ray_program.h"
#include "brdf.h"

namespace {
    using namespace fastgltf::math;
    using namespace app;

    const float GOLDEN_RATIO = 1.618034;
    
    float fibonacci1D(int i)
    {
        return std::fmod((static_cast<float>(i) + 1.0) * GOLDEN_RATIO, 1.0f);
    }
    fvec2 fibonacci2D(int i, int nbSamples)
    {
        return fvec2(
            (static_cast<float>(i) + 0.5) / static_cast<float>(nbSamples),
            fibonacci1D(i)
        );
    }
    fvec3 ImportanceSampleCosDir(fvec2 xi) {
        float cos_theta2 = 1.0f - xi.x();
        float cos_theta = sqrt(cos_theta2);
        float sin_theta = sqrt(xi.x());
        float phi = 2.0f * xi.y() * pi;

        float cos_phi = cos(phi);
        float sin_phi = sin(phi);

        return fvec3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
    }
    fvec3 Tangent2World(fvec3 v, fvec3 T, fvec3 B, fvec3 N) { return T * v.x() + B * v.y() + N * v.z(); }
    fvec3 Tangent2World(fvec3 v, const fmat3x3& TBN) { return TBN * v; }
    
    inline const Mesh& get_mesh_data(const Model& modelref, const ray_triangle_hit_info& hit) {
        return modelref.meshes_[hit.meshIndex];
    }
    inline std::tuple<const vertex&, const vertex&, const vertex&> get_vertex_data(const Mesh& mesh_data, const ray_triangle_hit_info& hit) {
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

    AOProgram::AOProgram(const Model& model, const CPUTexture<hdr_pixel>& env)
        : modelRef(model), envmapRef(env), gen(std::random_device{}()), dist(0.0f, 1.0f) {
    }
    fvec4 AOProgram::on_hit(const ray_with_payload& ray_, const ray_triangle_hit_info& hit, std::vector<ray_with_payload>& ray_collection) const
    {
        if (hit.forward_hit() == false) { // on miss
            return ray_.payload * fvec4(1.0f);
        }
        if (ray_.depth == 0) {
            return fvec4(0.0f);
        }
        const auto& mesh_data = get_mesh_data(modelRef, hit);
        const auto& [p1, p2, p3] = get_vertex_data(mesh_data, hit);
        auto point = interpolate_vertex_data(p1, p2, p3, hit);

        auto tangent_sign = point.tangent.w();
        point.tangent = fvec3(modelRef.objects_[hit.objectIndex].ModelMatrix * fvec4(point.tangent[0], point.tangent[1], point.tangent[2], 0.0f)); // Todo
        point.tangent[3] = tangent_sign;
        point.normal = fvec3(modelRef.objects_[hit.objectIndex].NormalMatrix * fvec4(point.normal[0], point.normal[1], point.normal[2], 0.0f)); // Todo
        auto bitangent = cross(point.normal, fvec3(point.tangent)) * point.tangent.w();

        fmat3x3 TBN = construct_TBN(fvec3(point.tangent), bitangent, point.normal);
        
        auto ws_pos = ray_.origin + ray_.direction * hit.distance;

        auto new_depth = ray_.depth - 1;

        int N = 32; // Todo: from render settings
        auto jitter_value = dist(gen);
        for (int i = 0; i < N; ++i) {
            fvec2 rand = fibonacci2D(i, N);
            rand[0] = std::fmod(rand[0] + jitter_value, 1.0f); // jitter
            auto new_direction = ImportanceSampleCosDir(rand);
            assert(new_direction.z() > 0.0f);
            assert(abs(length(new_direction) - 1.0f) < 1e-5f);
            new_direction = Tangent2World(new_direction, TBN);
            auto new_pos = ws_pos + point.normal * 1e-5f; // offset to avoid self-intersection
            ray_with_payload new_ray{ new_pos, normalize(new_direction), fvec4(1.0f/ N), new_depth, true };
            ray_collection.push_back(new_ray);
        }
        
        return fvec4(0.0f);
    }
}