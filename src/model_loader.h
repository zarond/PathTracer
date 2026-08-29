#pragma once

#include <cstdint>
#include <fastgltf/types.hpp>
#include <filesystem>
#include <glm/fwd.hpp>
#include <string>
#include <variant>
#include <vector>

#include "cpu_texture.h"

namespace app {

using glm::fmat4x4;
using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

struct vertex {
    fvec3 position;
    fvec3 normal;
    fvec4 tangent;
    fvec2 uv;
};

struct Mesh {                            // primitive in gltf terms
    std::vector<std::uint32_t> indices;  // invariant: size is multiple of 3, values are indices into vertices
    std::vector<vertex> vertices;
    uint32_t materialIndex;
};

struct Object {  // individual flat nodes with a single mesh
    fmat4x4 ModelMatrix;
    fmat4x4 NormalMatrix;
    uint32_t meshIndex;
    uint32_t nodeIndex; // original node index from gltf
};

struct Material {
    fvec4 baseColorFactor{1.0f, 1.0f, 1.0f, 1.0f};
    fvec3 emissiveFactor{0.0f, 0.0f, 0.0f};
    fvec3 attenuationFactor{0.0f, 0.0f, 0.0f};
    float metallicFactor = 1.0f;
    float roughnessFactor = 1.0f;
    int baseColorTextureIndex = -1;  // -1 is no texture
    int metallicRoughnessTextureIndex = -1;
    int normalTextureIndex = -1;
    float ior = 1.5f;
    float dielectric_f0 = 0.04f;
    float transmisionFactor = 0.0f;
    int transmissionTextureIndex = -1;
    int emissiveTextureIndex = -1;
    float emissiveStrength = 1.0f;
    bool doubleSided = false;  // means each side is a surface (thin objects)
    bool hasVolume = false;    // essential for transmission, otherwise treat as thin surface
    bool alphaBlending = false;
    float alphaCutoff = -1.0f;
    float AOStrength = 1.0f;
    int aoTextureIndex = -1;
    int thicknessTextureIndex = -1;
    float thicknessFactor = 0.0f;
    fvec3 attenuationColor{0.0f, 0.0f, 0.0f};
    float attenuationDistance = 0.0f;
    bool useBlenderAttenuation = true;
};

struct Camera {
    fmat4x4 ModelMatrix;
    std::variant<fastgltf::Camera::Perspective, fastgltf::Camera::Orthographic> camera_params;
    uint32_t nodeIndex;  // original node index from gltf
};

struct Node {
    fmat4x4 lsTransformMatrix{};
    uint32_t parent = -1u;
};

struct AnimationChannel {
    uint32_t samplerIndex = -1u;
    uint32_t nodeIndex = -1u;
    fastgltf::AnimationPath path;
};

struct AnimationSampler {
    uint32_t timeIndex;     // logical input
    uint32_t valueIndex;    // logical output
    fastgltf::AnimationInterpolation interpolation = fastgltf::AnimationInterpolation::Linear;
    mutable uint32_t cached_keyframe_index = 0;  // cached index for optimization
};

struct Animation {
    using ScalarData = std::vector<float>;
    using Vec2Data = std::vector<fvec2>;
    using Vec3Data = std::vector<fvec3>;
    using Vec4Data = std::vector<fvec4>;
    using AnimationData = std::variant<ScalarData, Vec2Data, Vec3Data, Vec4Data>;

    Animation(const fastgltf::Animation& gltf_animation, const fastgltf::Asset& asset);

    std::vector<AnimationChannel> channels;
    std::vector<AnimationSampler> samplers;
    std::vector<AnimationData> raw_data;
    std::string name;
    float duration = 0.0f;
};

using NodeTree = std::vector<Node>;

struct Model {
    std::vector<Camera> cameras;
    std::vector<Material> materials;
    std::vector<Mesh> meshes;
    std::vector<Object> objects;
    std::vector<CPUTexture<sdr_pixel>> images;
    std::vector<std::string> materials_names;
    NodeTree nodes_transforms;              // current transforms
    NodeTree nodes_original_transforms;     // "rest" transforms
    std::vector<Animation> animations;

    Model() = default;
    Model(Model&&) noexcept = default;
    Model& operator=(Model&&) noexcept = default;

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;

    void apply_animation(int animation_index, double time, bool looping);
    void update_objects(); // update objects from node_transforms
};

class ModelLoader {
  public:
    bool load_from_file(const std::filesystem::path& path);
    Model construct_model() const;

  private:
    fastgltf::Asset asset_;
};

}  // namespace app
