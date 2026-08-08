#include "model_loader.h"

#include <fastgltf/core.hpp>
#include <fastgltf/glm_element_traits.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/types.hpp>
#include <filesystem>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <glm/gtc/quaternion.hpp>

#include "brdf.h"
#include "compute_tangents.h"

namespace app {

using namespace glm;

bool ModelLoader::load_from_file(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        std::cerr << "Failed to find " << path << '\n';
        return false;
    }

    constexpr auto supportedExtensions =
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

Model ModelLoader::construct_model() const {
    Model model{};
    model.materials.reserve(asset_.materials.size() + 1);
    model.meshes.reserve(asset_.meshes.size());
    model.objects.reserve(asset_.nodes.size());
    model.images.reserve(asset_.images.size());
    model.materials_names.reserve(asset_.materials.size() + 1);

    for (const auto& image : asset_.images) {
        model.images.emplace_back(image, asset_);
    }

    TangentSpaceHelper tangent_space_helper;

    for (const auto& material : asset_.materials) {
        int baseColor_imageIndex = -1;
        int AO_imageIndex = -1;
        int metallicRoughness_imageIndex = -1;
        int normal_imageIndex = -1;
        int transmission_imageIndex = -1;
        int emissive_imageIndex = -1;
        int thickness_imageIndex = -1;
        if (material.pbrData.baseColorTexture.has_value()) {
            auto textureIndex = material.pbrData.baseColorTexture->textureIndex;
            baseColor_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(
                0);  // narrows size_t to int, but imageIndex should be small enough not to overflow
        }
        if (material.occlusionTexture.has_value()) {
            auto textureIndex = material.occlusionTexture->textureIndex;
            AO_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(0);
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
        fvec3 attenuationColor = fvec3(0.0f);
        if (material.volume) {
            attenuationColor = fvec3{material.volume->attenuationColor.x(), material.volume->attenuationColor.y(),
                material.volume->attenuationColor.z()};
            // Following Gltf standard of KHR_materials_volume:
            // attenuationFactor = -log(attenuationColor) / max(1e-5f, material.volume->attenuationDistance);
            // Following Blender implementation instead, prefer it more for better control for saturated colors and low
            // density mediums:
            attenuationFactor = (1.0f - attenuationColor) / max(1e-5f, material.volume->attenuationDistance);
            if (material.volume->thicknessTexture) {
                thickness_imageIndex = material.volume->thicknessTexture->textureIndex;
            }
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
            .alphaCutoff = (material.alphaMode == fastgltf::AlphaMode::Mask) ? material.alphaCutoff : -1.0f, 
            .AOStrength = material.occlusionTexture.has_value() ? material.occlusionTexture->strength : 0.0f,
            .aoTextureIndex = AO_imageIndex,
            .thicknessTextureIndex = thickness_imageIndex,
            .thicknessFactor = material.volume ? material.volume->thicknessFactor : 0.0f,
            .attenuationColor = attenuationColor,
            .attenuationDistance = material.volume ? material.volume->attenuationDistance : 0.0f,
            .useBlenderAttenuation = true,
        };
        model.materials.push_back(mat);
        model.materials_names.emplace_back(material.name);
    }
    model.materials.emplace_back();  // default material at last index
    model.materials_names.emplace_back("Gltf Default Material");

    // vector of span-like structures { index in model.meshes, size } to map gltf primitives to our model meshes
    // one gltf mesh (mesh_ids[i]) can be multiple gltf primitives
    std::vector<std::pair<uint32_t, uint32_t>> mesh_ids{};
    mesh_ids.reserve(asset_.meshes.size());

    uint32_t count = 0;
    for (const auto& gltf_mesh : asset_.meshes) {
        for (const auto& primitive : gltf_mesh.primitives) {
            Mesh mesh;

            auto* positionIt = primitive.findAttribute("POSITION");
            assert(positionIt != primitive.attributes.end());  // A mesh primitive is required to hold the POSITION attribute.
            assert(primitive.indicesAccessor.has_value());     // We specify GenerateMeshIndices, so we should always have indices

            auto* normalIt = primitive.findAttribute("NORMAL");
            auto* uvIt = primitive.findAttribute("TEXCOORD_0");
            auto* tangentIt = primitive.findAttribute("TANGENT");
            assert(normalIt != primitive.attributes.end());
            bool has_uv = (uvIt != primitive.attributes.end());

            // Load material index
            mesh.materialIndex = primitive.materialIndex.value_or(model.materials.size() - 1);  // value or default material

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

            model.meshes.push_back(std::move(mesh));
        }
        mesh_ids.emplace_back(count, gltf_mesh.primitives.size());
        count += gltf_mesh.primitives.size();
    }
    size_t sceneIndex = asset_.defaultScene.value_or(0);
    fastgltf::iterateSceneNodes(asset_, sceneIndex, fastgltf::math::fmat4x4(),
        [&model, &mesh_ids, this](const fastgltf::Node& node, const fastgltf::math::fmat4x4& matrix) {
            uint32_t node_index = &node - asset_.nodes.data();
            if (node.meshIndex.has_value()) {
                auto normalMatrix = transpose(inverse(matrix));
                size_t mesh_index = *node.meshIndex;
                for (uint32_t i = 0; i < mesh_ids[mesh_index].second; ++i) {
                    model.objects.emplace_back(
                        std::bit_cast<fmat4>(matrix), 
                        std::bit_cast<fmat4>(normalMatrix),
                        mesh_ids[mesh_index].first + i, 
                        node_index);
                }
            }
            if (node.cameraIndex.has_value()) {
                model.cameras.emplace_back(std::bit_cast<fmat4>(matrix), asset_.cameras[*node.cameraIndex].camera, node_index);
            }
        });

    model.nodes_transforms.reserve(asset_.nodes.size());
    // collect local transforms
    for (const fastgltf::Node& node : asset_.nodes) {
        fastgltf::math::fmat4x4 localMatrix = fastgltf::getLocalTransformMatrix(node);
        model.nodes_transforms.emplace_back(std::bit_cast<fmat4>(localMatrix), -1u);
    }
    // collect parenting data
    for (size_t i = 0; i < asset_.nodes.size(); ++i) {
        const auto& node = asset_.nodes[i];
        for (auto& child : node.children) {
            model.nodes_transforms[child].parent = i;
        }
    }

    model.animations.reserve(asset_.animations.size());
    for (const auto& animation : asset_.animations) {
        model.animations.emplace_back(animation, asset_);
    }

    return model;
}

void Model::update_objects() {
    const auto travel_upstream = [this](uint32_t node_id) -> fmat4x4 {
        fmat4x4 final_matrix{1.0f};
        for (auto current_node = node_id; current_node != -1u && current_node < nodes_transforms.size();) {
            const auto& node = nodes_transforms[current_node];
            const auto& ls_transform = node.lsTransformMatrix;
            final_matrix = ls_transform * final_matrix;
            current_node = node.parent;
        }
        return final_matrix;
    };
    for (auto& object : objects) {
        auto node_id = object.nodeIndex;
        object.ModelMatrix = travel_upstream(node_id);
        object.NormalMatrix = transpose(inverse(object.ModelMatrix));
    }
    for (auto& camera : cameras) {
        auto node_id = camera.nodeIndex;
        camera.ModelMatrix = travel_upstream(node_id);
    }
}

Animation::Animation(const fastgltf::Animation& gltf_animation, const fastgltf::Asset& asset) {
    // Copy channels from fastgltf::Animation
    channels.reserve(gltf_animation.channels.size());
    for (const auto& gltf_channel : gltf_animation.channels) {
        AnimationChannel channel{
            .samplerIndex = static_cast<uint32_t>(gltf_channel.samplerIndex),
            .nodeIndex = gltf_channel.nodeIndex.has_value() ? static_cast<uint32_t>(*gltf_channel.nodeIndex) : -1u,
            .path = gltf_channel.path
        };
        channels.push_back(std::move(channel));
    }

    // Extract duration from animation time data
    duration = 0.0f;

    // Build a mapping from accessor indices to raw_data indices to handle deduplication
    std::unordered_map<std::size_t, uint32_t> accessor_to_raw_data_index;
    uint32_t raw_index = 0;

    // First pass: extract all unique input accessors (time data)
    for (const auto& gltf_sampler : gltf_animation.samplers) {
        if (accessor_to_raw_data_index.find(gltf_sampler.inputAccessor) == accessor_to_raw_data_index.end()) {
            accessor_to_raw_data_index[gltf_sampler.inputAccessor] = raw_index++;

            const auto& inputAccessor = asset.accessors[gltf_sampler.inputAccessor];
            // Time data should always be scalar float values
            if (inputAccessor.type == fastgltf::AccessorType::Scalar) {
                ScalarData times(inputAccessor.count);
                fastgltf::copyFromAccessor<float>(asset, inputAccessor, times.data());
                duration = glm::max(duration, times.back());
                raw_data.emplace_back(std::move(times));
            }
        }
    }

    // Second pass: extract all output accessors (animation value data)
    for (const auto& gltf_sampler : gltf_animation.samplers) {
        if (accessor_to_raw_data_index.find(gltf_sampler.outputAccessor) == accessor_to_raw_data_index.end()) {
            accessor_to_raw_data_index[gltf_sampler.outputAccessor] = raw_index++;

            const auto& outputAccessor = asset.accessors[gltf_sampler.outputAccessor];
            // Determine the data type based on accessor type
            if (outputAccessor.type == fastgltf::AccessorType::Scalar) {
                ScalarData scalars(outputAccessor.count);
                fastgltf::copyFromAccessor<float>(asset, outputAccessor, scalars.data());
                raw_data.emplace_back(std::move(scalars));
            } else if (outputAccessor.type == fastgltf::AccessorType::Vec2) {
                Vec2Data vec2s(outputAccessor.count);
                fastgltf::copyFromAccessor<fvec2>(asset, outputAccessor, vec2s.data());
                raw_data.emplace_back(std::move(vec2s));
            } else if (outputAccessor.type == fastgltf::AccessorType::Vec3) {
                Vec3Data vec3s(outputAccessor.count);
                fastgltf::copyFromAccessor<fvec3>(asset, outputAccessor, vec3s.data());
                raw_data.emplace_back(std::move(vec3s));
            } else if (outputAccessor.type == fastgltf::AccessorType::Vec4) {
                Vec4Data vec4s(outputAccessor.count);
                fastgltf::copyFromAccessor<fvec4>(asset, outputAccessor, vec4s.data());
                raw_data.emplace_back(std::move(vec4s));
            }
        }
    }

    // Third pass: create samplers with correct indices pointing into raw_data
    samplers.reserve(gltf_animation.samplers.size());
    for (const auto& gltf_sampler : gltf_animation.samplers) {
        AnimationSampler sampler{
            .timeIndex = accessor_to_raw_data_index[gltf_sampler.inputAccessor],
            .valueIndex = accessor_to_raw_data_index[gltf_sampler.outputAccessor],
            .interpolation = gltf_sampler.interpolation
        };
        samplers.push_back(std::move(sampler));
    }

    name = gltf_animation.name;
}

namespace {
    // Helper function to find the keyframe indices for a given time
    // Uses cached index for optimization - assumes sequential animation playback
    // Returns pair of indices (t0, t1) where times[t0] <= time < times[t1]
    // If time is before first keyframe, returns (0, 0)
    // If time is after last keyframe, returns (last, last)
    // cached_index: reference to cached keyframe index for this sampler (updated in-place)
    std::pair<std::size_t, std::size_t> find_keyframe_indices(const std::vector<float>& times, float time, uint32_t& cached_index) {
        if (times.empty()) return {0, 0};
        if (time <= times.front()) {
            cached_index = 0;
            return {0, 0};
        }
        if (time >= times.back()) {
            cached_index = times.size() - 1;
            return {times.size() - 1, times.size() - 1};
        }

        // Start searching from cached index
        std::size_t start_idx = std::min(static_cast<std::size_t>(cached_index), times.size() - 2);

        if (times[start_idx] <= time) {
            // First try forward from cached position (common case: animation playing forward)
            for (std::size_t i = start_idx; i < times.size() - 1; ++i) {
                if (times[i] <= time && time < times[i + 1]) {
                    cached_index = i;
                    return {i, i + 1};
                }
            }
        } else {
            // Else try backward from cached position (animation rewound)
            for (std::size_t i = start_idx; i > 0; --i) {
                if (times[i - 1] <= time && time < times[i]) {
                    cached_index = i - 1;
                    return {i - 1, i};
                }
            }        
        }

        // Fallback
        cached_index = 0;
        return {0, 0};
    }

    template <typename T>
    T CubicInterpolation(float t0, float t1, float interpolation_factor, const T& v0, const T& v1, const T& a1, const T& b0) {
        float t2 = interpolation_factor * interpolation_factor;
        float t3 = t2 * interpolation_factor;

        T result =  (2.0f * t3 - 3.0f * t2 + 1.0f) * v0 +
                    (t3 - 2.0f * t2 + interpolation_factor) * (t1 - t0) * a1 +
                    (-2.0f * t3 + 3.0f * t2) * v1 +
                    (t3 - t2) * (t1 - t0) * b0;
        return result;
    }
}

void Model::apply_animation(int animation_index, double time_d, bool looping) {
    if (animation_index < 0 || animation_index >= static_cast<int>(animations.size())) {
        return;
    }

    const Animation& animation = animations[animation_index];

    if (looping) {
        time_d = std::fmod(time_d, static_cast<double>(animation.duration));
    }

    float time = time_d;

    // Process each animation channel
    for (const auto& channel : animation.channels) {
        if (channel.nodeIndex == -1u || channel.nodeIndex >= nodes_transforms.size()) {
            continue;
        }

        const auto& sampler = animation.samplers[channel.samplerIndex];

        // Get time data
        if (sampler.timeIndex >= animation.raw_data.size()) {
            continue;
        }

        const auto& time_variant = animation.raw_data[sampler.timeIndex];
        const auto* times = std::get_if<Animation::ScalarData>(&time_variant);
        if (!times || times->empty()) {
            continue;
        }

        // Get value data
        if (sampler.valueIndex >= animation.raw_data.size()) {
            continue;
        }

        const auto& value_variant = animation.raw_data[sampler.valueIndex];

        // Find keyframe indices using cached index for optimization
        auto [t0_idx, t1_idx] = find_keyframe_indices(*times, time, sampler.cached_keyframe_index);
        float t0 = (*times)[t0_idx];
        float t1 = (*times)[t1_idx];

        // Calculate interpolation factor
        float interpolation_factor = 0.0f;
        if (t1 > t0) {
            interpolation_factor = (time - t0) / (t1 - t0);
            interpolation_factor = glm::clamp(interpolation_factor, 0.0f, 1.0f);
        }

        // Apply interpolation based on sampler type
        Node& node = nodes_transforms[channel.nodeIndex];

        if (channel.path == fastgltf::AnimationPath::Translation) {
            if (const auto* values = std::get_if<Animation::Vec3Data>(&value_variant)) {
                fvec3 result;
                if (sampler.interpolation == fastgltf::AnimationInterpolation::Step) {
                    result = (*values)[t0_idx];
                } else if (sampler.interpolation == fastgltf::AnimationInterpolation::Linear) {
                    result = glm::mix((*values)[t0_idx], (*values)[t1_idx], interpolation_factor);
                } else if (sampler.interpolation == fastgltf::AnimationInterpolation::CubicSpline) {
                    // CubicSpline: each output has 3 values per keyframe (in-tangent, value, out-tangent)
                    const fvec3& v0 = (*values)[t0_idx * 3 + 1];
                    const fvec3& a1 = (*values)[t0_idx * 3 + 2];
                    const fvec3& b0 = (*values)[t1_idx * 3 + 0];
                    const fvec3& v1 = (*values)[t1_idx * 3 + 1];

                    result = CubicInterpolation(t0, t1, interpolation_factor, v0, v1, a1, b0);
                }

                // Update node translation
                node.lsTransformMatrix[3] = fvec4(result, 1.0f);
            }
        } else if (channel.path == fastgltf::AnimationPath::Rotation) {
            if (const auto* values = std::get_if<Animation::Vec4Data>(&value_variant)) {
                glm::quat result;
                if (sampler.interpolation == fastgltf::AnimationInterpolation::Step) {
                    const fvec4& q = (*values)[t0_idx];
                    result = glm::quat(q.w, q.x, q.y, q.z);
                } else if (sampler.interpolation == fastgltf::AnimationInterpolation::Linear) {
                    // For rotations, use SLERP instead of linear interpolation
                    const fvec4& q0 = (*values)[t0_idx];
                    const fvec4& q1 = (*values)[t1_idx];
                    glm::quat quat0(q0.w, q0.x, q0.y, q0.z);
                    glm::quat quat1(q1.w, q1.x, q1.y, q1.z);
                    result = glm::slerp(quat0, quat1, interpolation_factor);
                } else if (sampler.interpolation == fastgltf::AnimationInterpolation::CubicSpline) {
                    // CubicSpline with quaternions
                    const fvec4& v0 = (*values)[t0_idx * 3 + 1];
                    const fvec4& a1 = (*values)[t0_idx * 3 + 2];
                    const fvec4& b0 = (*values)[t1_idx * 3 + 0];
                    const fvec4& v1 = (*values)[t1_idx * 3 + 1];

                    fvec4 cubic_result = CubicInterpolation(t0, t1, interpolation_factor, v0, v1, a1, b0);
                    cubic_result = normalize(cubic_result);
                    result = glm::quat(cubic_result.w, cubic_result.x, cubic_result.y, cubic_result.z);
                }

                // Update node rotation (convert quaternion to rotation matrix)
                fmat4x4 rotation_matrix = mat4_cast(result);
                node.lsTransformMatrix[0] = rotation_matrix[0];
                node.lsTransformMatrix[1] = rotation_matrix[1];
                node.lsTransformMatrix[2] = rotation_matrix[2];
            }
        } else if (channel.path == fastgltf::AnimationPath::Scale) {
            if (const auto* values = std::get_if<Animation::Vec3Data>(&value_variant)) {
                fvec3 result;
                if (sampler.interpolation == fastgltf::AnimationInterpolation::Step) {
                    result = (*values)[t0_idx];
                } else if (sampler.interpolation == fastgltf::AnimationInterpolation::Linear) {
                    result = glm::mix((*values)[t0_idx], (*values)[t1_idx], interpolation_factor);
                } else if (sampler.interpolation == fastgltf::AnimationInterpolation::CubicSpline) {
                    const fvec3& v0 = (*values)[t0_idx * 3 + 1];
                    const fvec3& a1 = (*values)[t0_idx * 3 + 2];
                    const fvec3& b0 = (*values)[t1_idx * 3 + 0];
                    const fvec3& v1 = (*values)[t1_idx * 3 + 1];

                    result = CubicInterpolation(t0, t1, interpolation_factor, v0, v1, a1, b0);
                }

                // Apply scale to the transformation matrix
                node.lsTransformMatrix[0] = fvec4(glm::normalize(fvec3(node.lsTransformMatrix[0])) * result.x, 0.0f);
                node.lsTransformMatrix[1] = fvec4(glm::normalize(fvec3(node.lsTransformMatrix[1])) * result.y, 0.0f);
                node.lsTransformMatrix[2] = fvec4(glm::normalize(fvec3(node.lsTransformMatrix[2])) * result.z, 0.0f);
            }
        }
    }

    // Update objects after applying animation
    update_objects();
}

}  // namespace app
