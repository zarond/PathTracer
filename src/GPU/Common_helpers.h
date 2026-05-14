#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include <d3dcompiler.h>

#include <iostream>

#include "../d3d_context.h"
#include "helpers/DXSampleHelper.h"

namespace app {
using Microsoft::WRL::ComPtr;

inline std::pair<ComPtr<ID3DBlob>, D3D12_SHADER_BYTECODE> LoadShader(const wchar_t* file_name) {
    ComPtr<ID3DBlob> shaderBlob;
    auto hr = D3DReadFileToBlob(file_name, &shaderBlob);
    if (FAILED(hr)) {
        std::wcout << "Failed to read DXIL library: " << file_name << std::endl;
        throw HrException(hr);
    }
    D3D12_SHADER_BYTECODE dxil = {
        .pShaderBytecode = shaderBlob->GetBufferPointer(),
        .BytecodeLength = shaderBlob->GetBufferSize(),
    };

    return std::make_pair(shaderBlob, dxil);
}

inline void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig) {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;
    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> error;

    ThrowIfFailed(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &error),
        error ? static_cast<wchar_t*>(error->GetBufferPointer()) : nullptr);
    ThrowIfFailed(device->CreateRootSignature(1, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&(*rootSig))));
}

}  // namespace app