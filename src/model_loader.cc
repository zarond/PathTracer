#include <string>
#include <filesystem>
#include <iostream>

#include "model_loader.h"
#include "compute_tangents.h"

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

namespace app {
	bool ModelLoader::loadFromFile(std::filesystem::path path)
{
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

	constexpr auto gltfOptions =
		fastgltf::Options::DontRequireValidAssetMember |
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

Model ModelLoader::constructModel() const
{
	Model model{
		.cameras_ = {},
        .materials_ = {},
		.meshes_ = {},
		.objects_ = {},
		.images_ = {},
	};
    model.materials_.reserve(asset_.materials.size()+1);
    model.meshes_.reserve(asset_.meshes.size());
    model.objects_.reserve(asset_.nodes.size());
	model.images_.reserve(asset_.images.size());

	for (const auto& image : asset_.images) {
		model.images_.emplace_back(image);
	}

	TangentSpaceHelper tangent_space_helper;

	for (const auto& material : asset_.materials) {
		int baseColor_imageIndex = -1;
		if (material.pbrData.baseColorTexture.has_value()) {
            auto textureIndex = material.pbrData.baseColorTexture->textureIndex;
            baseColor_imageIndex = asset_.textures[textureIndex].imageIndex.value_or(0); // narrows size_t to int, but imageIndex should be small enough not to overflow
		}
		model.materials_.emplace_back(
			material.pbrData.baseColorFactor,
			baseColor_imageIndex
        );
	}
	model.materials_.emplace_back(); // default material at last index

	std::vector<uint32_t> mesh_ids{};
    mesh_ids.reserve(asset_.meshes.size());

	uint32_t count = 0;
	for (const auto& gltf_mesh : asset_.meshes) {
		for (const auto& primitive : gltf_mesh.primitives) {
			Mesh mesh;
			
			auto* positionIt = primitive.findAttribute("POSITION");
			assert(positionIt != primitive.attributes.end()); // A mesh primitive is required to hold the POSITION attribute.
			assert(primitive.indicesAccessor.has_value()); // We specify GenerateMeshIndices, so we should always have indices

			auto* normalIt = primitive.findAttribute("NORMAL");
			auto* uvIt = primitive.findAttribute("TEXCOORD_0");
			auto* tangentIt = primitive.findAttribute("TANGENT"); 
			assert(normalIt != primitive.attributes.end());
			assert(uvIt != primitive.attributes.end());

            // Load material index
			mesh.materialIndex = primitive.materialIndex.value_or(model.materials_.size() - 1); // value or default material

			// Load indices
			{
				auto& indexAccessor = asset_.accessors[primitive.indicesAccessor.value()];
				if (!indexAccessor.bufferViewIndex.has_value())
					throw 1; // Todo
				mesh.indices.resize(indexAccessor.count);
                fastgltf::copyFromAccessor<std::uint32_t>(asset_, indexAccessor, mesh.indices.data());
			}

            // Load vertices
			{
				auto& positionAccessor = asset_.accessors[positionIt->accessorIndex];
				auto& normalAccessor = asset_.accessors[normalIt->accessorIndex];
				auto& uvAccessor = asset_.accessors[uvIt->accessorIndex];
				if (!positionAccessor.bufferViewIndex.has_value())
					continue;
				if (!normalAccessor.bufferViewIndex.has_value())
					continue;
				if (!uvAccessor.bufferViewIndex.has_value())
					continue;
                
				mesh.vertices.reserve(positionAccessor.count);
                // Todo: optimize
				for (size_t i = 0; i < positionAccessor.count; ++i) {
					auto position = fastgltf::getAccessorElement<fvec3>(asset_, positionAccessor, i);
					auto normal = fastgltf::getAccessorElement<fvec3>(asset_, normalAccessor, i);
					auto uv = fastgltf::getAccessorElement<fvec2>(asset_, uvAccessor, i);
					mesh.vertices.emplace_back(position, normal, fvec4{}, uv);
                }
				const auto* tangentAccessor = (tangentIt != primitive.attributes.end()) ? &asset_.accessors[tangentIt->accessorIndex] : nullptr;
				if (tangentAccessor != nullptr) {
					fastgltf::iterateAccessorWithIndex<fvec4>(asset_, *tangentAccessor, [&](fvec4 tangent, std::size_t idx) {
						mesh.vertices[idx].tangent = tangent;
					});
				}
				else {
					tangent_space_helper.compute_tangents(mesh);
				}
			}

			model.meshes_.push_back(std::move(mesh));
			++count;
		}
        mesh_ids.push_back(count); // mesh_ids[index of gltf mesh] is the last index (not including) in model.meshes_ for this gltf mesh, because one gltf mesh can be multiple gltf primitives that map to model meshes;
	}
    // Todo: Maybe do all nodes in order, instead of scene nodes only?
	size_t sceneIndex = asset_.defaultScene.value_or(0);
	fastgltf::iterateSceneNodes(asset_, sceneIndex, fmat4x4(),
		[&model, &mesh_ids, this](const fastgltf::Node& node, const fmat4x4& matrix) {
			if (node.meshIndex.has_value()) {
                auto normalMatrix = transpose(inverse(matrix));
                size_t mesh_index = *node.meshIndex; // Todo: refactor for more clear code
				for (size_t i = (mesh_index > 0)? mesh_ids[mesh_index - 1] : 0; i < mesh_ids[mesh_index]; ++i) {
					model.objects_.emplace_back(matrix, normalMatrix, i);
				}
			}
			if (node.cameraIndex.has_value()) {
				model.cameras_.emplace_back(
					matrix,
					asset_.cameras[*node.cameraIndex].camera
                );
			}
		}
	);

	return model;
}

}
