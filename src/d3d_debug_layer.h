#pragma once

#include <d3d12.h>
#include <dxgi1_6.h>

#define NOMINMAX
#include <wrl.h>

#ifdef _DEBUG
#define DX12_ENABLE_DEBUG_LAYER
#endif

#ifdef DX12_ENABLE_DEBUG_LAYER
#include <dxgidebug.h>
#endif

namespace app {

using Microsoft::WRL::ComPtr;

class DXDebugLayer {
  public:
    bool Init();
    void Shutdown();
    void SetBreakOnSeverity(ID3D12Device& device);

  private:
#ifdef DX12_ENABLE_DEBUG_LAYER
    ComPtr<ID3D12Debug6> m_d3d12Debug;
    ComPtr<IDXGIDebug1> m_dxgiDebug;
#endif

// singleton
  public:
    DXDebugLayer(const DXDebugLayer&) = delete;
    DXDebugLayer& operator=(const DXDebugLayer&) = delete;

    inline static DXDebugLayer& Get() {
        static DXDebugLayer instance;
        return instance;
    }

  private:
    DXDebugLayer() = default;
};

}
