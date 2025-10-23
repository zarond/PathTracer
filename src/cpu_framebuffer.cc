#include <string>
#include <variant>
#include <algorithm>

#include "cpu_framebuffer.h"

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image.h"
#include "stb_image_write.h"

namespace {
    using namespace app;
    std::vector<pixel> from_raw_data(const float* data, size_t width, size_t height, size_t channels) {
        std::vector<pixel> vec;
        vec.reserve(width * height);
        // ToDo: optimize
        for (size_t i = 0; i < width * height; ++i) {
            const float* data_p = data + i * 4;
            pixel p(*data_p, *(data_p + 1), *(data_p + 2), *(data_p + 3));
            vec.push_back(p);
        }
        return vec;
    }
}

namespace app {
    using namespace fastgltf::math;
// CPUTexture

CPUTexture::CPUTexture(const fastgltf::Image& image)
{
    std::visit(fastgltf::visitor{
        [](const auto& arg) {},
        [&](const fastgltf::sources::URI& filePath) {
            assert(filePath.fileByteOffset == 0); // We don't support offsets with stbi.
            assert(filePath.uri.isLocalPath()); // We're only capable of loading local files.
            
            const std::string path(filePath.uri.path().begin(), filePath.uri.path().end()); // Thanks C++.
            // ToDo: different gamma
            float* data = stbi_loadf(path.c_str(), &width_, &height_, &channels_, 4);
            data_ = from_raw_data(data, width_, height_, channels_);
            stbi_image_free(data);
        },
        [&](const fastgltf::sources::Array& vector) {
            float* data = stbi_loadf_from_memory(reinterpret_cast<const stbi_uc*>(vector.bytes.data()), static_cast<int>(vector.bytes.size()), &width_, &height_, &channels_, 4);
            data_ = from_raw_data(data, width_, height_, channels_);
            stbi_image_free(data);
        },
        [&](const fastgltf::sources::BufferView& view) {
            // ToDo: Implement loading from buffer view if needed.
            /*
            auto& bufferView = viewer->asset.bufferViews[view.bufferViewIndex];
            auto& buffer = viewer->asset.buffers[bufferView.bufferIndex];
            // Yes, we've already loaded every buffer into some GL buffer. However, with GL it's simpler
            // to just copy the buffer data again for the texture. Besides, this is just an example.
            std::visit(fastgltf::visitor {
                // We only care about VectorWithMime here, because we specify LoadExternalBuffers, meaning
                // all buffers are already loaded into a vector.
                [](auto& arg) {},
                [&](fastgltf::sources::Array& vector) {
                    unsigned char* data = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(vector.bytes.data() + bufferView.byteOffset),
                                                                static_cast<int>(bufferView.byteLength), &width_, &height_, &channels_, 4);

                    stbi_image_free(data);
                }
            }, buffer.data);
            */
        },
        }, 
    image.data);

    assert(width_ > 0 && height_ > 0 && channels_ > 0 && channels_ <= 4 && !data_.empty());
}

CPUTexture::CPUTexture(const std::filesystem::path& filePath) {
    const std::string path(filePath.string());
    // ToDo: different gamma
    float* data = stbi_loadf(path.c_str(), &width_, &height_, &channels_, 4);
    data_ = from_raw_data(data, width_, height_, channels_);
    stbi_image_free(data);

    assert(width_ > 0 && height_ > 0 && channels_ > 0 && channels_ <= 4 && !data_.empty());
}

pixel CPUTexture::sample_nearest(fvec2 uv) const {
    // TODO: wrap coords, better code
    size_t x = static_cast<size_t>(uv.x() * static_cast<float>(width_)) % width_;
    size_t y = static_cast<size_t>(uv.y() * static_cast<float>(height_)) % height_;
    assert(x < width_ && y < height_);
    return data_[y * width_ + x];
}

CPUTexture CPUTexture::create_white_texture()
{
    return CPUTexture();
}

CPUTexture::CPUTexture() : width_(1), height_(1), channels_(4), data_(1, pixel(1.0f)) {}

int CPUTexture::width() const { return width_; }
int CPUTexture::height() const { return height_; }

// CPUFrameBuffer

CPUFrameBuffer::CPUFrameBuffer() : CPUTexture()
{
}

CPUFrameBuffer::CPUFrameBuffer(int width, int height)
{
    width_ = width;
    height_ = height;
    channels_ = 4;
    data_.resize(width_ * height_, pixel{});
}

void CPUFrameBuffer::clear(const pixel clearColor)
{
    data_.assign(width_ * height_, clearColor);
}

pixel& CPUFrameBuffer::at(int x, int y)
{
    return data_[y * width_ + x];
}

const pixel& CPUFrameBuffer::at(int x, int y) const
{
    return data_[y * width_ + x];
}

void CPUFrameBuffer::save_to_file(const std::filesystem::path& filePath) const
{
    // Todo: optimize
    std::vector<float> rawData;
    rawData.reserve(width_ * height_ * 4);
    for (const auto& px : data_) {
        rawData.push_back(px.x());
        rawData.push_back(px.y());
        rawData.push_back(px.z());
        rawData.push_back(px.w());
    }
    stbi_write_hdr(filePath.string().c_str(), width_, height_, 4, rawData.data());
}

}