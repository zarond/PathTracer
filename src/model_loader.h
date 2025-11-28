#pragma once

#include <fastgltf/types.hpp>
#include <filesystem>
#include <glm/glm.hpp>
#include <vector>

#include "cpu_framebuffer.h"

namespace app {

using namespace glm;

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
    float alpha_cutoff = -1.0f;
};

struct Camera {
    fmat4x4 ModelMatrix;
    std::variant<fastgltf::Camera::Perspective, fastgltf::Camera::Orthographic> camera_params;
};

struct Model {
    std::vector<Camera> cameras_;
    std::vector<Material> materials_;
    std::vector<Mesh> meshes_;
    std::vector<Object> objects_;
    std::vector<CPUTexture<sdr_pixel>> images_;
};

class ModelLoader {
  public:
    bool loadFromFile(std::filesystem::path path);
    Model constructModel() const;

  private:
    fastgltf::Asset asset_;
};

}  // namespace app
