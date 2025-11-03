#pragma once

#include <filesystem>
#include <vector>

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

#include <glm/glm.hpp>

#include "cpu_framebuffer.h"

namespace app {

using namespace glm;

struct vertex {
    fvec3 position;
    fvec3 normal;
    fvec4 tangent;
    fvec2 uv;
};

struct Mesh { // primitive in gltf terms
    std::vector<std::uint32_t> indices; // invariant: size is multiple of 3, values are indices into vertices
    std::vector<vertex> vertices;
    uint32_t materialIndex;
};

struct Object { // individual flat nodes with a single mesh
    fmat4x4 ModelMatrix;
    fmat4x4 NormalMatrix;
    uint32_t meshIndex;
};

struct Material {
    fvec4 baseColorFactor{1.0f, 1.0f, 1.0f, 1.0f};
    //float metallicFactor = 1.0f;
    //float roughnessFactor = 1.0f;
    int baseColorTextureIndex = -1; // -1 is no texture
    //int metallicRoughnessTextureIndex = -1;
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


}
