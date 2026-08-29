#include "cpu_framebuffer.h"

#include <algorithm>
#include <glm/glm.hpp>
#include <iostream>
#include <span>
#include <string>

#include "stb_image.h"
#include "stb_image_write.h"

#ifdef WINDOWS_SPECIFIC
#include "d3d_context.h"
#include <directx/d3dx12.h>
#endif

namespace app {
using namespace glm;

// CPUFrameBuffer

CPUFrameBuffer::CPUFrameBuffer() : CPUTexture<hdr_pixel>() {}

CPUFrameBuffer::CPUFrameBuffer(int width, int height) {
    width_ = width;
    height_ = height;
    channels_ = 4;
    data_.resize(width_ * height_, hdr_pixel{});
}

#ifdef WINDOWS_SPECIFIC
CPUFrameBuffer::~CPUFrameBuffer() { release_gpu_resource(); }
#endif

void CPUFrameBuffer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    channels_ = 4;
    data_.resize(width_ * height_, hdr_pixel{});
#ifdef WINDOWS_SPECIFIC
    release_gpu_resource();
#endif
}

void CPUFrameBuffer::clear(const hdr_pixel clearColor) { data_.assign(width_ * height_, clearColor); }

hdr_pixel& CPUFrameBuffer::at(int x, int y) { return data_[y * width_ + x]; }

const hdr_pixel& CPUFrameBuffer::at(int x, int y) const { return data_[y * width_ + x]; }

void CPUFrameBuffer::save_to_file(const std::filesystem::path& filePath, bool from_GPU_texture) const {
    std::vector<hdr_pixel> gpu_data;
    std::span<const hdr_pixel> data_source;

#ifdef WINDOWS_SPECIFIC
    if (from_GPU_texture) {
        gpu_data = download_from_gpu();
        if (gpu_data.empty()) {
            std::cout << "Failed to download data from GPU for saving.\n";
            return;
        }
        data_source = gpu_data;
    } else {
        data_source = data_;
    }
#else
    data_source = data_;
#endif
    int ret = 0;
    if (filePath.extension() == ".png") {
        std::vector<unsigned char> rawData;
        rawData.reserve(width_ * height_ * 4);
        std::for_each(data_source.begin(), data_source.end(), [&rawData](const auto& pixel) {
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
        std::memcpy(rawData.data(), data_source.data(), rawData.size() * sizeof(float));
        ret = stbi_write_hdr(filePath.string().c_str(), width_, height_, 4, rawData.data());
    }
    if (ret == 0) {
        throw std::runtime_error("Failed to save image to " + filePath.string());
    }
}

#ifdef WINDOWS_SPECIFIC
void CPUFrameBuffer::create_texture_resource() {
    auto flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV;
    gpuTexture.release_gpu_resource();
    gpuTexture = GPU_texture{static_cast<UINT64>(width_), static_cast<UINT>(height_), 
        flags, DXGI_FORMAT_UNKNOWN, D3D12_RESOURCE_STATE_COMMON, true};
}

void CPUFrameBuffer::upload_to_gpu(){
    D3DContext& d3d_ctx = D3DContext::Get();

    auto pTexture = gpuTexture.get_gpu_resource();
    if (!pTexture) {
        create_texture_resource();
        pTexture = gpuTexture.get_gpu_resource();
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

        hr = d3d_ctx.m_d3dDevice->CreateCommittedResource(
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
        .pResource = uploadBuffer.Get(),
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
        .pResource = pTexture.Get(),
        .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX,
        .SubresourceIndex = 0,
    };

    d3d_ctx.m_CommandList->CopyTextureRegion(&dstLocation, 0, 0, 0, &srcLocation, nullptr);
}

const GPU_texture& CPUFrameBuffer::get_texture_resource() const { return gpuTexture; }

void CPUFrameBuffer::transition_from_copy_to_srv() const {
    const auto pTexture = gpuTexture.get_gpu_resource();
    if (!pTexture) return;

    D3DContext& d3d_ctx = D3DContext::Get();
    const auto barrier_to_psr = CD3DX12_RESOURCE_BARRIER::Transition(
        pTexture.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    d3d_ctx.m_CommandList->ResourceBarrier(1, &barrier_to_psr);
}

void CPUFrameBuffer::transition_from_srv_to_copy() const {
    const auto pTexture = gpuTexture.get_gpu_resource();
    if (!pTexture) return;

    D3DContext& d3d_ctx = D3DContext::Get();
    const auto toCopyDest = CD3DX12_RESOURCE_BARRIER::Transition(
        pTexture.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COMMON);
    d3d_ctx.m_CommandList->ResourceBarrier(1, &toCopyDest);
}

void CPUFrameBuffer::transition_from_srv_to_uav() const {
    const auto pTexture = gpuTexture.get_gpu_resource();
    if (!pTexture) return;

    D3DContext& d3d_ctx = D3DContext::Get();
    const auto toUAV = CD3DX12_RESOURCE_BARRIER::Transition(
        pTexture.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    d3d_ctx.m_DXRCommandList->ResourceBarrier(1, &toUAV);
}
void CPUFrameBuffer::transition_from_uav_to_srv() const {
    const auto pTexture = gpuTexture.get_gpu_resource();
    if (!pTexture) return;

    D3DContext& d3d_ctx = D3DContext::Get();
    const auto toSRV = CD3DX12_RESOURCE_BARRIER::Transition(
        pTexture.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    d3d_ctx.m_DXRCommandList->ResourceBarrier(1, &toSRV);
}

ComPtr<ID3D12Resource> CPUFrameBuffer::get_gpu_resource() const { return gpuTexture.get_gpu_resource(); }

ComPtr<ID3D12Resource> CPUFrameBuffer::get_gpu_upload_resource() const { return uploadBuffer; }

void CPUFrameBuffer::release_gpu_resource() {
    gpuTexture.release_gpu_resource();
    if (uploadBuffer) {
        if (mapped) {
            D3D12_RANGE range = {0, uploadSize};
            uploadBuffer->Unmap(0, &range);
            mapped = nullptr;
        }
        uploadBuffer.Reset();
    }
}

std::vector<hdr_pixel> CPUFrameBuffer::download_from_gpu() const { 
    const auto pTexture = gpuTexture.get_gpu_resource();
    std::vector<hdr_pixel> readback_data;
    if (!pTexture) return readback_data;

    readback_data.resize(data_.size());

    D3DContext& d3d_ctx = D3DContext::Get();

    D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint;
    UINT numRows;
    UINT64 rowSizeInBytes;
    UINT64 totalBytes;

    const D3D12_RESOURCE_DESC tex_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = static_cast<UINT64>(width_),
        .Height = static_cast<UINT>(height_),
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_R32G32B32A32_FLOAT,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
        .Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS
    };
    const D3D12_HEAP_PROPERTIES heap_props{
        .Type = D3D12_HEAP_TYPE_READBACK,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
    };

    d3d_ctx.m_d3dDevice->GetCopyableFootprints(&tex_desc, 0, 1, 0, &footprint, &numRows, &rowSizeInBytes, &totalBytes);

    const D3D12_RESOURCE_DESC upload_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = totalBytes,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ComPtr<ID3D12Resource> readbackBuffer;
    d3d_ctx.m_d3dDevice->CreateCommittedResource(
        &heap_props, D3D12_HEAP_FLAG_NONE, &upload_desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&readbackBuffer));


    const D3D12_TEXTURE_COPY_LOCATION srcLocation = {
        .pResource = pTexture.Get(),
        .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX,
        .SubresourceIndex = 0,
    };

    const D3D12_TEXTURE_COPY_LOCATION dstLocation = {
        .pResource = readbackBuffer.Get(),
        .Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT,
        .PlacedFootprint = footprint,
    };

    transition_from_srv_to_copy();

    d3d_ctx.m_CommandList->CopyTextureRegion(&dstLocation, 0, 0, 0, &srcLocation, nullptr);

    const D3D12_RESOURCE_BARRIER barrier_to_psr = {
        .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
        .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
        .Transition =
            D3D12_RESOURCE_TRANSITION_BARRIER{
                .pResource = pTexture.Get(),
                .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
                .StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE,
                .StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
            },
    };
    d3d_ctx.m_CommandList->ResourceBarrier(1, &barrier_to_psr);

    d3d_ctx.DispatchCommandList();

    d3d_ctx.WaitForPendingOperations();

    auto frameCtx = d3d_ctx.GetCurrentFrameContext();
    d3d_ctx.InitCommandList(*frameCtx->CommandAllocator.Get());

    void* mappedData = nullptr;
    readbackBuffer->Map(0, nullptr, &mappedData);

    for (int y = 0; y < height_; y++)
        memcpy(
            readback_data.data() + y * width_,
            (void*)((uintptr_t)mappedData + y * footprint.Footprint.RowPitch),
            width_ * sizeof(hdr_pixel)
        );

    readbackBuffer->Unmap(0, nullptr);
    return readback_data;
}

#endif  // ifndef NO_WINDOWS
}  // namespace app