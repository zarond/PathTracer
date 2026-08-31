#include "cpu_texture.h"

#include <glm/glm.hpp>
#include <string>
#include <variant>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image.h"
#include "stb_image_write.h"

namespace app {
std::array<float, 256> SRGB8_TO_LINEAR_LUT = make_srgb_lut();
}

namespace {
using namespace app;

std::vector<hdr_pixel> from_raw_data(const float* data, size_t width, size_t height) {
    std::vector<hdr_pixel> vec(width * height);
    std::memcpy(vec.data(), data, width * height * sizeof(hdr_pixel));
    return vec;
}

std::vector<sdr_pixel> from_raw_data(const unsigned char* data, size_t width, size_t height) {
    std::vector<sdr_pixel> vec(width * height);
    std::memcpy(vec.data(), data, width * height * sizeof(sdr_pixel));
    return vec;
}
}  // namespace

namespace app {
using namespace glm;

// CPUTexture

template <>
CPUTexture<sdr_pixel>::CPUTexture(const fastgltf::Image& image, const fastgltf::Asset& asset_) {
    std::visit(fastgltf::visitor{
                   [](const auto& arg) {},
                   [&](const fastgltf::sources::URI& filePath) {
                       assert(filePath.fileByteOffset == 0);  // We don't support offsets with stbi.
                       assert(filePath.uri.isLocalPath());    // We're only capable of loading local files.

                       const std::string path(filePath.uri.path().begin(), filePath.uri.path().end());  // Thanks C++.
                       unsigned char* data = stbi_load(path.c_str(), &width_, &height_, &channels_, 4);
                       if (data == nullptr) {
                           throw std::runtime_error("Unable to load image: " + path);
                       }
                       data_ = from_raw_data(data, width_, height_);
                       stbi_image_free(data);
                   },
                   [&](const fastgltf::sources::Array& vector) {
                       unsigned char* data = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(vector.bytes.data()),
                           static_cast<int>(vector.bytes.size()), &width_, &height_, &channels_, 4);
                       if (data == nullptr) {
                           throw std::runtime_error("Unable to load image from memory");
                       }
                       data_ = from_raw_data(data, width_, height_);
                       stbi_image_free(data);
                   },
                   [&](const fastgltf::sources::BufferView& view) {
                       const auto& bufferView = asset_.bufferViews[view.bufferViewIndex];
                       const auto& buffer = asset_.buffers[bufferView.bufferIndex];
                       std::visit(fastgltf::visitor{
                                      [&](const fastgltf::sources::Array& vector) {
                                          unsigned char* data = stbi_load_from_memory(
                                              reinterpret_cast<const stbi_uc*>(vector.bytes.data() + bufferView.byteOffset),
                                              static_cast<int>(bufferView.byteLength), &width_, &height_, &channels_, 4);
                                          if (data == nullptr) {
                                              throw std::runtime_error("Unable to load image from memory");
                                          }
                                          data_ = from_raw_data(data, width_, height_);
                                          stbi_image_free(data);
                                      },
                                      [](const auto& arg) {}},
                           buffer.data);
                   },
               },
        image.data);

    assert(width_ > 0 && height_ > 0 && channels_ > 0 && channels_ <= 4 && !data_.empty());
}

template <>
CPUTexture<hdr_pixel>::CPUTexture(const std::filesystem::path& filePath) {
    const std::string path(filePath.string());

    float* data = stbi_loadf(path.c_str(), &width_, &height_, &channels_, 4);
    if (data == nullptr) {
        throw std::runtime_error("Unable to load image: " + path);
    }
    data_ = from_raw_data(data, width_, height_);
    stbi_image_free(data);

    assert(width_ > 0 && height_ > 0 && channels_ > 0 && channels_ <= 4 && !data_.empty());
}

template <>
CPUTexture<sdr_pixel> CPUTexture<sdr_pixel>::create_white_texture() {
    return CPUTexture(sdr_pixel{255, 255, 255, 255});
}
template <>
CPUTexture<sdr_pixel> CPUTexture<sdr_pixel>::create_black_texture() {
    return CPUTexture(sdr_pixel{0, 0, 0, 255});
}
template <>
CPUTexture<hdr_pixel> CPUTexture<hdr_pixel>::create_white_texture() {
    return CPUTexture(hdr_pixel{1.0f});
}
template <>
CPUTexture<hdr_pixel> CPUTexture<hdr_pixel>::create_black_texture() {
    return CPUTexture(hdr_pixel{0.0f});
}

}  // namespace app