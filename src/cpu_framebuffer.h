#pragma once

#include <filesystem>
#include <vector>
#include <array>
#include <cmath>

#include <fastgltf/types.hpp>
#include <glm/glm.hpp>

namespace app {

using namespace glm;

inline float linear_to_srgb(float channel)
{
    if (channel <= 0.0031308f) {
        return 12.92f * channel;
    }
    else {
        return 1.055f * std::pow(channel, 1.0f / 2.4f) - 0.055f;
    }
}
inline float srgb_to_linear(float channel)
{
    if (channel <= 0.04045f) {
        return channel / 12.92f;
    }
    else {
        return std::pow((channel + 0.055f) / 1.055f, 2.4f);
    }
}

using sdr_pixel = std::array<std::uint8_t, 4>;
using hdr_pixel = fvec4;

template<typename T>
concept PixelType = std::is_same_v<T, sdr_pixel> || std::is_same_v<T, hdr_pixel>;

template<PixelType pixel>
inline fvec4 pixel_to_float(pixel sample) {
    return fvec4(
        static_cast<float>(sample[0]) / 255.0f,
        static_cast<float>(sample[1]) / 255.0f,
        static_cast<float>(sample[2]) / 255.0f,
        static_cast<float>(sample[3]) / 255.0f);
};
template<PixelType pixel>
inline fvec4 srgb_pixel_to_float(pixel sample) {
    return fvec4(
        srgb_to_linear(static_cast<float>(sample[0]) / 255.0f),
        srgb_to_linear(static_cast<float>(sample[1]) / 255.0f),
        srgb_to_linear(static_cast<float>(sample[2]) / 255.0f),
        static_cast<float>(sample[3]) / 255.0f);
}

inline sdr_pixel float_pixel_to_srgb8(hdr_pixel sample) {
    sample = clamp(sample, 0.0f, 1.0f);
    return sdr_pixel{   static_cast<uint8_t>(linear_to_srgb(sample.r) * 255 + 0.5f),
                        static_cast<uint8_t>(linear_to_srgb(sample.g) * 255 + 0.5f),
                        static_cast<uint8_t>(linear_to_srgb(sample.b) * 255 + 0.5f),
                        static_cast<uint8_t>(linear_to_srgb(sample.a) * 255 + 0.5f) };
}

template<PixelType pixel>
class CPUTexture {
public:
    // Idea: use mdspan
    CPUTexture() = default;
    explicit CPUTexture(const fastgltf::Image& image, const fastgltf::Asset& asset_); // used to load sdr texture images
    explicit CPUTexture(const std::filesystem::path& path); // used to load hdr environment images
    CPUTexture(pixel initial_col) : width_(1), height_(1), channels_(4), data_(1, initial_col) {}
    
    int width() const { return width_; }
    int height() const { return height_; }

    fvec4 sample_nearest(fvec2 uv, bool srgb_tex = false) const { // Idea: sampler to individual class
        auto xf = std::fmod(uv.x, 1.0f);
        auto yf = std::fmod(uv.y, 1.0f);
        if (xf < 0.0f) xf += 1.0f;
        if (yf < 0.0f) yf += 1.0f;
        size_t x = static_cast<size_t>(xf * width_);
        size_t y = static_cast<size_t>(yf * height_);
        assert(x < width_ && y < height_);
        pixel sample = data_[y * width_ + x];
        if (srgb_tex) {
            return srgb_pixel_to_float(sample);
        }
        else {
            return pixel_to_float(sample);
        }
    }

    fvec4 sample_bilinear(fvec2 uv, bool srgb_tex = false) const {
        auto xf = uv.x * static_cast<float>(width_) - 0.5f;
        auto yf = uv.y * static_cast<float>(height_) - 0.5f;
        float tx = xf - std::floor(xf);
        float ty = yf - std::floor(yf);
        int x0 = static_cast<int>(std::floor(xf));
        int y0 = static_cast<int>(std::floor(yf));
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        x0 = ((x0 % width_) + width_) % width_;
        y0 = ((y0 % height_) + height_) % height_;
        x1 = ((x1 % width_) + width_) % width_;
        y1 = ((y1 % height_) + height_) % height_;
        const pixel c00 = data_[y0 * width_ + x0];
        const pixel c10 = data_[y0 * width_ + x1];
        const pixel c01 = data_[y1 * width_ + x0];
        const pixel c11 = data_[y1 * width_ + x1];
        fvec4 col00, col10, col01, col11;
        if constexpr (std::is_same_v<pixel, sdr_pixel>) {
            col00 = srgb_tex ? srgb_pixel_to_float(c00) : pixel_to_float(c00);
            col10 = srgb_tex ? srgb_pixel_to_float(c10) : pixel_to_float(c10);
            col01 = srgb_tex ? srgb_pixel_to_float(c01) : pixel_to_float(c01);
            col11 = srgb_tex ? srgb_pixel_to_float(c11) : pixel_to_float(c11);
        }
        else if constexpr (std::is_same_v<pixel, hdr_pixel>) {
            col00 = c00;
            col10 = c10;
            col01 = c01;
            col11 = c11;
        }

        fvec4 top = mix(col00, col10, tx);
        fvec4 bottom = mix(col01, col11, tx);
        return mix(top, bottom, ty);
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
