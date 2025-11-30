#include "model_loader.h"

#include <algorithm>
#include <fastgltf/core.hpp>
#include <fastgltf/glm_element_traits.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/types.hpp>
#include <filesystem>
#include <iostream>
#include <utility>

#include "brdf.h"
#include "compute_tangents.h"

namespace app {

bool ModelLoader::loadFromFile(std::filesystem::path path) {
    if (!std::filesystem::exists(path)) {
        std::cout << "Failed to find " << path << '\n';
        return false;
    }

    static constexpr auto supportedExtensions =
        fastgltf::Extensions::KHR_lights_punctual | 
        fastgltf::Extensions::KHR_materials_specular |
        fastgltf::Extensions::KHR_materials_ior | 
        fastgltf::Extensions::KHR_materials_volume |
        fastgltf::Extensions::KHR_materials_transmission | 
        fastgltf::Extensions::KHR_materials_emissive_strength;

    fastgltf::Parser parser(supportedExtensions);

    constexpr auto gltfOptions = fastgltf::Options::DontRequireValidAssetMember |
                                 fastgltf::Options::LoadExternalBuffers | 
                                 fastgltf::Options::LoadExternalImages |
                                 fastgltf::Options::GenerateMeshIndices;

    auto gltfFile = fastgltf::MappedGltfFile::FromPath(path);
    if (!bool(gltfFile)) {
        std::cerr << "Failed to open glTF file: " << fastgltf::getErrorMessage(gltfFile.error()) << '\n';
        return false;
    }

    auto asset_tmp = parser.loadGltf(gltfFile.get(), path.parent_path(), gltfOptions);
    if (asset_tmp.error() != fastgltf::Error::None) {
        std::cerr << "Failed to load glTF: " << fastgltf::getErrorMessage(asset_tmp.error()) << '\n';
        return false;
    }

    asset_ = std::move(asset_tmp.get());

    return true;
}

Model ModelLoader::constructModel() const {
    Model model{};
    model.materials_.reserve(asset_.materials.size() + 1);
    model.meshes_.reserve(asset_.meshes.size());
    model.objects_.reserve(asset_.nodes.size());
    model.images_.reserve(asset_.images.size());

    for (const auto& image : asset_.images) {
        model.images_.emplace_back(image, asset_);
    }

    TangentSpaceHelper tangent_space_helper;

    for (const auto& material : asset_.materials) {
        int baseColor_imageIndex = -1;
        int metallicRoughness_imageIndex = -1;
        int normal_imageIndex = -1;
        int transmission_imageIndex = -1;
        int emissive_imageIndex = -1;
        if (material.pbrData.baseColorTexture.has_value()) {
            auto textureIndex = material.pbrData.baseColorTexture->textureIndex;
            baseColor_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(
                0);  // narrows size_t to int, but imageIndex should be small enough not to overflow
        }
        if (material.pbrData.metallicRoughnessTexture.has_value()) {
            auto textureIndex = material.pbrData.metallicRoughnessTexture->textureIndex;
            metallicRoughness_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(0);
        }
        if (material.normalTexture.has_value()) {
            auto textureIndex = material.normalTexture->textureIndex;
            normal_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(0);
        }
        if (material.transmission && material.transmission->transmissionTexture.has_value()) {
            auto textureIndex = material.transmission->transmissionTexture->textureIndex;
            transmission_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(0);
        }
        if (material.emissiveTexture.has_value()) {
            auto textureIndex = material.emissiveTexture->textureIndex;
            emissive_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(0);
        }
        fvec3 attenuationFactor = fvec3(0.0f);
        if (material.volume) {
            attenuationFactor = fvec3{material.volume->attenuationColor.x(), material.volume->attenuationColor.y(),
                material.volume->attenuationColor.z()};
            // Following Gltf standard of KHR_materials_volume:
            // attenuationFactor = -log(attenuationFactor) / max(1e-5f, material.volume->attenuationDistance);
            // Following Blender implementation instead, prefer it more for better control for saturated colors and low
            // density mediums:
            attenuationFactor = (1.0f - attenuationFactor) / max(1e-5f, material.volume->attenuationDistance);
        }
        Material mat{
            .baseColorFactor = std::bit_cast<fvec4>(material.pbrData.baseColorFactor),
            .emissiveFactor = fvec3{material.emissiveFactor.x(), material.emissiveFactor.y(), material.emissiveFactor.z()},
            .attenuationFactor = attenuationFactor,
            .metallicFactor = material.pbrData.metallicFactor,
            .roughnessFactor = material.pbrData.roughnessFactor,
            .baseColorTextureIndex = baseColor_imageIndex,
            .metallicRoughnessTextureIndex = metallicRoughness_imageIndex,
            .normalTextureIndex = normal_imageIndex,
            .ior = material.ior,
            .dielectric_f0 = f0_dielectric(material.ior),
            .transmisionFactor = material.transmission ? material.transmission->transmissionFactor : 0.0f,
            .transmissionTextureIndex = transmission_imageIndex,
            .emissiveTextureIndex = emissive_imageIndex,
            .emissiveStrength = material.emissiveStrength,
            .doubleSided = material.doubleSided,
            .hasVolume = material.volume ? true : false,
            .alphaBlending = (material.alphaMode == fastgltf::AlphaMode::Blend),
            .alpha_cutoff = (material.alphaMode == fastgltf::AlphaMode::Mask) ? material.alphaCutoff : -1.0f};
        model.materials_.push_back(mat);
    }
    model.materials_.emplace_back();  // default material at last index

    // vector of span-like structures { index in model.meshes_, size } to map gltf primitives to our model meshes
    // one gltf mesh (mesh_ids[i]) can be multiple gltf primitives
    std::vector<std::pair<uint32_t, uint32_t>> mesh_ids{};
    mesh_ids.reserve(asset_.meshes.size());

    uint32_t count = 0;
    for (const auto& gltf_mesh : asset_.meshes) {
        for (const auto& primitive : gltf_mesh.primitives) {
            Mesh mesh;

            auto* positionIt = primitive.findAttribute("POSITION");
            assert(positionIt !=  primitive.attributes.end());  // A mesh primitive is required to hold the POSITION attribute.
            assert(primitive.indicesAccessor.has_value());  // We specify GenerateMeshIndices, so we should always have indices

            auto* normalIt = primitive.findAttribute("NORMAL");
            auto* uvIt = primitive.findAttribute("TEXCOORD_0");
            auto* tangentIt = primitive.findAttribute("TANGENT");
            assert(normalIt != primitive.attributes.end());
            bool has_uv = (uvIt != primitive.attributes.end());

            // Load material index
            mesh.materialIndex = primitive.materialIndex.value_or(model.materials_.size() - 1);  // value or default material

            // Load indices
            {
                auto& indexAccessor = asset_.accessors[primitive.indicesAccessor.value()];
                if (!indexAccessor.bufferViewIndex.has_value()) throw std::runtime_error("Malformed GLTF: No indices.");
                mesh.indices.resize(indexAccessor.count);
                fastgltf::copyFromAccessor<std::uint32_t>(asset_, indexAccessor, mesh.indices.data());
            }

            // Load vertices
            {
                auto& positionAccessor = asset_.accessors[positionIt->accessorIndex];
                auto& normalAccessor = asset_.accessors[normalIt->accessorIndex];
                auto& uvAccessor = asset_.accessors[uvIt->accessorIndex];
                if (!positionAccessor.bufferViewIndex.has_value())
                    throw std::runtime_error("Malformed GLTF: No positions.");
                if (!normalAccessor.bufferViewIndex.has_value())
                    throw std::runtime_error("Malformed GLTF: No normals.");
                if (has_uv && !uvAccessor.bufferViewIndex.has_value())
                    throw std::runtime_error("Malformed GLTF: No uvs (but were promised).");

                mesh.vertices.reserve(positionAccessor.count);
                for (size_t i = 0; i < positionAccessor.count; ++i) {
                    auto position = fastgltf::getAccessorElement<fvec3>(asset_, positionAccessor, i);
                    auto normal = fastgltf::getAccessorElement<fvec3>(asset_, normalAccessor, i);
                    auto uv = (has_uv) ? fastgltf::getAccessorElement<fvec2>(asset_, uvAccessor, i) : fvec2{0.0f};
                    mesh.vertices.emplace_back(position, normal, fvec4{}, uv);
                }
                const auto* tangentAccessor =
                    (tangentIt != primitive.attributes.end()) ? &asset_.accessors[tangentIt->accessorIndex] : nullptr;
                if (tangentAccessor != nullptr) {
                    fastgltf::iterateAccessorWithIndex<fvec4>(asset_, *tangentAccessor,
                        [&](fvec4 tangent, std::size_t idx) { mesh.vertices[idx].tangent = tangent; });
                } else if (has_uv) {
                    tangent_space_helper.compute_tangents(mesh);
                } else {
                    tangent_space_helper.compute_tangents_no_uv(mesh);
                }
            }

            model.meshes_.push_back(std::move(mesh));
        }
        mesh_ids.push_back({count, gltf_mesh.primitives.size()});
        count += gltf_mesh.primitives.size();
    }
    size_t sceneIndex = asset_.defaultScene.value_or(0);
    fastgltf::iterateSceneNodes(asset_, sceneIndex, fastgltf::math::fmat4x4(),
        [&model, &mesh_ids, this](const fastgltf::Node& node, const fastgltf::math::fmat4x4& matrix) {
            if (node.meshIndex.has_value()) {
                auto normalMatrix = transpose(inverse(matrix));
                size_t mesh_index = *node.meshIndex;
                for (uint32_t i = 0; i < mesh_ids[mesh_index].second; ++i) {
                    model.objects_.emplace_back(
                        std::bit_cast<fmat4>(matrix), 
                        std::bit_cast<fmat4>(normalMatrix),
                        mesh_ids[mesh_index].first + i);
                }
            }
            if (node.cameraIndex.has_value()) {
                model.cameras_.emplace_back(std::bit_cast<fmat4>(matrix), asset_.cameras[*node.cameraIndex].camera);
            }
        });

    return model;
}

}  // namespace app
