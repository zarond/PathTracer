#pragma once

#include <filesystem>
#include <vector>
#include <array>

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

namespace app {

using namespace fastgltf::math;

using sdr_pixel = std::array<std::uint8_t, 4>;
using hdr_pixel = fastgltf::math::fvec4;

template<typename T>
concept PixelType = std::is_same_v<T, sdr_pixel> || std::is_same_v<T, hdr_pixel>;

template<PixelType pixel>
class CPUTexture {
public:
    // Idea: use mdspan
    CPUTexture() = default;
    explicit CPUTexture(const fastgltf::Image& image); // used to load sdr texture images
    explicit CPUTexture(const std::filesystem::path& path); // used to load hdr environment images
    CPUTexture(pixel initial_col) : width_(1), height_(1), channels_(4), data_(1, initial_col) {}
    
    int width() const { return width_; }
    int height() const { return height_; }

    pixel sample_nearest(fvec2 uv) const { // Idea: sampler to individual class
        // TODO: wrap coords, better code
        size_t x = static_cast<size_t>(uv.x() * static_cast<float>(width_)) % width_;
        size_t y = static_cast<size_t>(uv.y() * static_cast<float>(height_)) % height_;
        assert(x < width_ && y < height_);
        return data_[y * width_ + x];
    }

    static CPUTexture create_white_texture();
    static CPUTexture create_black_texture();

protected:
    std::vector<pixel> data_;
    int width_ = 1;
    int height_ = 1;
    int channels_ = 4; // Number of channels in the image originally (e.g., 3 for RGB, 4 for RGBA); we always store as RGBA
};

class CPUFrameBuffer: private CPUTexture<hdr_pixel> {
private:
    using CPUTexture::width_;
    using CPUTexture::height_;
    using CPUTexture::channels_;
    using CPUTexture::data_;
public:
    CPUFrameBuffer();
    CPUFrameBuffer(int width, int height);
    
    void clear(const hdr_pixel clearColor = hdr_pixel{0.0,0.0,0.0,1.0});
    hdr_pixel& at(int x, int y);
    const hdr_pixel& at(int x, int y) const;
    
    using CPUTexture::width;
    using CPUTexture::height;
    using CPUTexture::sample_nearest;

    void save_to_file(const std::filesystem::path& filePath) const;
};

}
