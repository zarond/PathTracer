#include "d3d_debug_layer.h"

namespace app {

bool DXDebugLayer::Init() {
#ifdef DX12_ENABLE_DEBUG_LAYER
    // Init D3D12 Debug layer
    if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&m_d3d12Debug)))) {
        m_d3d12Debug->EnableDebugLayer();
        // Init DXGI Debug
        if (SUCCEEDED(DXGIGetDebugInterface1(0, IID_PPV_ARGS(&m_dxgiDebug)))) {
            m_dxgiDebug->EnableLeakTrackingForThread();
            return true;
        }
    }
#endif
    return false;
}

void DXDebugLayer::SetBreakOnSeverity(ID3D12Device& device) {
#ifdef DX12_ENABLE_DEBUG_LAYER
    ID3D12InfoQueue* pInfoQueue = nullptr;
    device.QueryInterface(IID_PPV_ARGS(&pInfoQueue));
    pInfoQueue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_ERROR, true);
    pInfoQueue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_CORRUPTION, true);
    // pInfoQueue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_WARNING, true);
    pInfoQueue->Release();
#endif
}

void DXDebugLayer::Shutdown() {
#ifdef DX12_ENABLE_DEBUG_LAYER
    if (m_dxgiDebug) {
        OutputDebugStringW(L"DXGI Debug Report Live Objects:\n");
        m_dxgiDebug->ReportLiveObjects(
            DXGI_DEBUG_ALL, DXGI_DEBUG_RLO_FLAGS(DXGI_DEBUG_RLO_DETAIL | DXGI_DEBUG_RLO_IGNORE_INTERNAL));
    }
    m_d3d12Debug.Reset();
    m_dxgiDebug.Reset();
#endif
}

}  // namespace app
