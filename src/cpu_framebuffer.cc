#include "cpu_framebuffer.h"

#include <algorithm>
#include <glm/glm.hpp>
#include <string>
#include <type_traits>
#include <variant>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image.h"
#include "stb_image_write.h"

#ifndef NO_WINDOWS
#include "d3d_context.h"
#endif

namespace app {
    std::array<float, 256> SRGB8_TO_LINEAR_LUT = make_srgb_lut();
}

namespace {
using namespace app;

std::vector<hdr_pixel> from_raw_data(const float* data, size_t width, size_t height, size_t channels) {
    std::vector<hdr_pixel> vec(width * height);
    std::memcpy(vec.data(), data, width * height * sizeof(hdr_pixel));
    return vec;
}

std::vector<sdr_pixel> from_raw_data(const unsigned char* data, size_t width, size_t height, size_t channels) {
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
    std::visit(
        fastgltf::visitor{
            [](const auto& arg) {},
            [&](const fastgltf::sources::URI& filePath) {
                assert(filePath.fileByteOffset == 0);  // We don't support offsets with stbi.
                assert(filePath.uri.isLocalPath());    // We're only capable of loading local files.

                const std::string path(filePath.uri.path().begin(), filePath.uri.path().end());  // Thanks C++.
                unsigned char* data = stbi_load(path.c_str(), &width_, &height_, &channels_, 4);
                if (data == nullptr) {
                    throw std::runtime_error("Unable to load image: " + path);
                }
                data_ = from_raw_data(data, width_, height_, channels_);
                stbi_image_free(data);
            },
            [&](const fastgltf::sources::Array& vector) {
                unsigned char* data = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(vector.bytes.data()),
                    static_cast<int>(vector.bytes.size()), &width_, &height_, &channels_, 4);
                if (data == nullptr) {
                    throw std::runtime_error("Unable to load image from memory");
                }
                data_ = from_raw_data(data, width_, height_, channels_);
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
                        data_ = from_raw_data(data, width_, height_, channels_);
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
    data_ = from_raw_data(data, width_, height_, channels_);
    stbi_image_free(data);

    assert(width_ > 0 && height_ > 0 && channels_ > 0 && channels_ <= 4 && !data_.empty());
}

template <>
CPUTexture<sdr_pixel> CPUTexture<sdr_pixel>::create_white_texture() { return CPUTexture(sdr_pixel{255, 255, 255, 255});}
template <>
CPUTexture<sdr_pixel> CPUTexture<sdr_pixel>::create_black_texture() { return CPUTexture(sdr_pixel{0, 0, 0, 255});}
template <>
CPUTexture<hdr_pixel> CPUTexture<hdr_pixel>::create_white_texture() { return CPUTexture(hdr_pixel{1.0f});}
template <>
CPUTexture<hdr_pixel> CPUTexture<hdr_pixel>::create_black_texture() { return CPUTexture(hdr_pixel{0.0f});}

// CPUFrameBuffer

CPUFrameBuffer::CPUFrameBuffer() : CPUTexture<hdr_pixel>() {}

CPUFrameBuffer::CPUFrameBuffer(int width, int height) {
    width_ = width;
    height_ = height;
    channels_ = 4;
    data_.resize(width_ * height_, hdr_pixel{});
}

#ifndef NO_WINDOWS
CPUFrameBuffer::~CPUFrameBuffer() { release_gpu_resource(); }
#endif

void CPUFrameBuffer::clear(const hdr_pixel clearColor) { data_.assign(width_ * height_, clearColor); }

hdr_pixel& CPUFrameBuffer::at(int x, int y) { return data_[y * width_ + x]; }

const hdr_pixel& CPUFrameBuffer::at(int x, int y) const { return data_[y * width_ + x]; }

void CPUFrameBuffer::save_to_file(const std::filesystem::path& filePath) const {
    int ret = 0;
    if (filePath.extension() == ".png") {
        std::vector<unsigned char> rawData;
        rawData.reserve(width_ * height_ * 4);
        std::for_each(data_.begin(), data_.end(), [&rawData](const auto& pixel) {
            sdr_pixel p = float_pixel_to_srgb8(pixel);
            rawData.emplace_back(p[0]);
            rawData.emplace_back(p[1]);
            rawData.emplace_back(p[2]);
            rawData.emplace_back(p[3]);
        });
        int stride_bytes = width_ * 4;
        ret = stbi_write_png(filePath.string().c_str(), width_, height_, 4, rawData.data(), stride_bytes);
    } else {
        std::vector<float> rawData(width_ * height_ * 4);
        static_assert(sizeof(hdr_pixel) == 4 * sizeof(float));
        std::memcpy(rawData.data(), data_.data(), rawData.size() * sizeof(float));
        ret = stbi_write_hdr(filePath.string().c_str(), width_, height_, 4, rawData.data());
    }
    if (ret == 0) {
        throw std::runtime_error("Failed to save image to " + filePath.string());
    }
}

#ifndef NO_WINDOWS
void CPUFrameBuffer::upload_to_gpu(){
    D3DContext& d3d_ctx = D3DContext::Get();

    if (d3d_ctx.g_fenceLastSignaledValue) {
        d3d_ctx.g_pd3dCopyQueue->Wait(d3d_ctx.g_fence.Get(), d3d_ctx.g_fenceLastSignaledValue);
    }

    // Wait for previous copy to complete
    const UINT64 lastSignaled = d3d_ctx.copy_fenceLastSignaledValue;
    if (d3d_ctx.copy_fence->GetCompletedValue() < lastSignaled) {
        HRESULT hr = d3d_ctx.copy_fence->SetEventOnCompletion(lastSignaled, d3d_ctx.copy_fenceEvent);
        if (SUCCEEDED(hr)) {
            ::WaitForSingleObject(d3d_ctx.copy_fenceEvent, INFINITE);
        }
    }

    d3d_ctx.InitCopyCommandList();

    // Create texture resource
    const D3D12_RESOURCE_DESC tex_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = static_cast<UINT64>(width_),
        .Height = static_cast<UINT>(height_),
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_R32G32B32A32_FLOAT,
        .SampleDesc =  {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
        .Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS // for raytracing
    };

    const D3D12_HEAP_PROPERTIES def_props{
        .Type = D3D12_HEAP_TYPE_DEFAULT,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
    };

    if (pTexture == nullptr) {
        D3DContext::Get().g_pd3dSrvDescHeapAlloc.Alloc(&srv_cpu_handle, &srv_gpu_handle);
        D3DContext::Get().g_pd3dSrvDescHeapAlloc.Alloc(&uav_cpu_handle, &uav_gpu_handle);

        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
            &def_props, D3D12_HEAP_FLAG_NONE, &tex_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&pTexture));

        // Create a shader resource view for the texture
        const D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc{
            .Format = DXGI_FORMAT_R32G32B32A32_FLOAT,
            .ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
            .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
            .Texture2D =
                D3D12_TEX2D_SRV{
                    .MostDetailedMip = 0,
                    .MipLevels = 1,
                },
        };
        d3d_ctx.g_pd3dDevice->CreateShaderResourceView(pTexture, &srvDesc, srv_cpu_handle);

        // Create a unordered access view for the texture
        const D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc{
            .Format = DXGI_FORMAT_R32G32B32A32_FLOAT,
            .ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D,
            .Texture2D =
                D3D12_TEX2D_UAV{
                    .MipSlice = 0,
                    .PlaneSlice = 0,
                },
        };
        d3d_ctx.g_pd3dDevice->CreateUnorderedAccessView(pTexture, nullptr, &uavDesc, uav_cpu_handle);

    }
    
    HRESULT hr;
    if (uploadBuffer == nullptr) {
        // Create a temporary upload resource to move the data in
        uploadPitch =
            (width_ * sizeof(hdr_pixel) + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1u) & ~(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1u);
        uploadSize = height_ * uploadPitch;

        const D3D12_RESOURCE_DESC upload_desc{
            .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
            .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
            .Width = uploadSize,
            .Height = 1,
            .DepthOrArraySize = 1,
            .MipLevels = 1,
            .Format = DXGI_FORMAT_UNKNOWN,
            .SampleDesc = {1, 0},
            .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
            .Flags = D3D12_RESOURCE_FLAG_NONE,
        };
        const D3D12_HEAP_PROPERTIES upload_props{
            .Type = D3D12_HEAP_TYPE_UPLOAD,
            .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
            .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        };

        hr = d3d_ctx.g_pd3dDevice->CreateCommittedResource(
            &upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&uploadBuffer));
        assert(SUCCEEDED(hr));
    }
    // Write pixels into the upload resource
    if (mapped == nullptr) {
        D3D12_RANGE range = {0, uploadSize};
        hr = uploadBuffer->Map(0, nullptr, &mapped);
        assert(SUCCEEDED(hr));
    }
    for (int y = 0; y < height_; y++)
        memcpy((void*)((uintptr_t)mapped + y * uploadPitch), data_.data() + y * width_, width_ * sizeof(hdr_pixel));

    // Copy the upload resource content into the real resource
    const D3D12_TEXTURE_COPY_LOCATION srcLocation = {
        .pResource = uploadBuffer,
        .Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT,
        .PlacedFootprint = D3D12_PLACED_SUBRESOURCE_FOOTPRINT{
            .Footprint = D3D12_SUBRESOURCE_FOOTPRINT{
                .Format = DXGI_FORMAT_R32G32B32A32_FLOAT,
                .Width = static_cast<UINT>(width_),
                .Height = static_cast<UINT>(height_),
                .Depth = 1,
                .RowPitch = uploadPitch,
            }
        }
    };

    const D3D12_TEXTURE_COPY_LOCATION dstLocation = {
        .pResource = pTexture,
        .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX,
        .SubresourceIndex = 0,
    };
    // Copy call
    d3d_ctx.g_pd3dCopyCommandList->CopyTextureRegion(&dstLocation, 0, 0, 0, &srcLocation, nullptr);

    d3d_ctx.DispatchCopyCommandList();

    hr = d3d_ctx.g_pd3dCopyQueue->Signal(d3d_ctx.copy_fence.Get(), ++d3d_ctx.copy_fenceLastSignaledValue);
    assert(SUCCEEDED(hr));
    d3d_ctx.g_pd3dCommandQueue->Wait(d3d_ctx.copy_fence.Get(), d3d_ctx.copy_fenceLastSignaledValue);

    // Transition the texture to be a shader resource for use in shaders
    const D3D12_RESOURCE_BARRIER barrier_to_psr = {
        .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
        .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
        .Transition =
            D3D12_RESOURCE_TRANSITION_BARRIER{
                .pResource = pTexture,
                .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
                .StateBefore = D3D12_RESOURCE_STATE_COPY_DEST,
                .StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
            },
    };
    d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &barrier_to_psr);
}

void CPUFrameBuffer::transition_from_srv_to_copy() {
    if (!pTexture) return;

    D3DContext& d3d_ctx = D3DContext::Get();

    const D3D12_RESOURCE_BARRIER toCopyDest = {
        .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
        .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
        .Transition =
            D3D12_RESOURCE_TRANSITION_BARRIER{
                .pResource = pTexture,
                .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
                .StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                .StateAfter = D3D12_RESOURCE_STATE_COMMON,
            },
    };
    d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &toCopyDest);
}

void CPUFrameBuffer::transition_from_srv_to_uav() {
    if (!pTexture) return;

    D3DContext& d3d_ctx = D3DContext::Get();

    const D3D12_RESOURCE_BARRIER toUAV = {
        .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
        .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
        .Transition =
            D3D12_RESOURCE_TRANSITION_BARRIER{
                .pResource = pTexture,
                .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
                .StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                .StateAfter = D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
            },
    };
    d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &toUAV);
}
void CPUFrameBuffer::transition_from_uav_to_srv() {
    if (!pTexture) return;

    D3DContext& d3d_ctx = D3DContext::Get();

    const D3D12_RESOURCE_BARRIER toSRV = {
        .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
        .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
        .Transition =
            D3D12_RESOURCE_TRANSITION_BARRIER{
                .pResource = pTexture,
                .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
                .StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                .StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
            },
    };
    d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &toSRV);
}

void CPUFrameBuffer::release_gpu_resource() {
    if (pTexture) {
        D3DContext::Get().g_pd3dSrvDescHeapAlloc.Free(srv_cpu_handle, srv_gpu_handle);
        D3DContext::Get().g_pd3dSrvDescHeapAlloc.Free(uav_cpu_handle, uav_gpu_handle);
        pTexture->Release();
        pTexture = nullptr;
    }
    if (mapped) {
        D3D12_RANGE range = {0, uploadSize};
        uploadBuffer->Unmap(0, &range);
        mapped = nullptr;
    }
    if (uploadBuffer) {
        uploadBuffer->Release();
        uploadBuffer = nullptr;
    }
}
#endif  // ifndef NO_WINDOWS
}  // namespace app