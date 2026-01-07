#include "ui.h"
#include "d3d_context.h"

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx12.h"

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace app {

bool DXWindow::Init() {
    // Window class
    WNDCLASSEXW wcex{};
    wcex.cbSize = sizeof(wcex);
    // wcex.style = CS_CLASSDC;
    wcex.style = CS_OWNDC;
    wcex.lpfnWndProc = &WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = GetModuleHandleW(nullptr);
    wcex.hIcon = LoadIconW(nullptr, MAKEINTRESOURCEW(IDI_APPLICATION));
    wcex.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(IDC_ARROW));
    wcex.hbrBackground = nullptr;
    wcex.lpszMenuName = nullptr;
    wcex.lpszClassName = L"ImGui Example";
    wcex.hIconSm = LoadIconW(nullptr, MAKEINTRESOURCEW(IDI_APPLICATION));
    wndClass = RegisterClassExW(&wcex);
    if (wndClass == 0) return false;
    // create window
    hwnd = ::CreateWindowExW(WS_EX_OVERLAPPEDWINDOW | WS_EX_APPWINDOW, (LPCWSTR)wndClass, L"Dear ImGui DirectX12 Example",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE, 100, 100, 1280, 720, nullptr, nullptr, wcex.hInstance, nullptr);
    if (hwnd == nullptr) return false;
    // showwindow?
    return true;
}
void DXWindow::ShutDown() {
    if (hwnd) {
        DestroyWindow(hwnd);
        hwnd = nullptr;
    }
    if (wndClass) {
        UnregisterClassW((LPCWSTR)wndClass, GetModuleHandleW(nullptr));
        wndClass = 0;
    }
}
void DXWindow::Update() {
    MSG msg;
    while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
        ::TranslateMessage(&msg);
        ::DispatchMessage(&msg);
    }
}
void DXWindow::SetFullscreen(bool enabled, HWND hWnd) {
    // Update window style
    DWORD style = WS_OVERLAPPEDWINDOW | WS_VISIBLE;
    DWORD ex_style = WS_EX_OVERLAPPEDWINDOW | WS_EX_APPWINDOW;
    if (enabled) {
        style = WS_POPUP | WS_VISIBLE;
        ex_style = WS_EX_APPWINDOW;
    }

    ::SetWindowLongW(hWnd, GWL_STYLE, style);
    ::SetWindowLongW(hWnd, GWL_EXSTYLE, ex_style);

    // Adjust window size
    if (enabled) {
        HMONITOR monitor = ::MonitorFromWindow(hWnd, MONITOR_DEFAULTTONEAREST);
        MONITORINFO monitorInfo{};
        monitorInfo.cbSize = sizeof(monitorInfo);
        if (GetMonitorInfoW(monitor, &monitorInfo)) {
            SetWindowPos(hWnd, nullptr, monitorInfo.rcMonitor.left, monitorInfo.rcMonitor.top,
                monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left,
                monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top, SWP_NOZORDER);
        }
    } else {
        ShowWindow(hWnd, SW_MAXIMIZE);
    }

    is_fullscreen = enabled;
}


// Win32 message handler
// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of
// the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your
// copy of the keyboard data. Generally you may always pass all inputs to dear imgui, and hide them from your application based on
// those two flags.
LRESULT WINAPI DXWindow::WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return true;

    DXWindow& dx_window = DXWindow::Get();

    switch (msg) {
        case WM_KEYDOWN:
            if (wParam == VK_F11) {
                dx_window.SetFullscreen(!dx_window.is_fullscreen, hWnd);
            }
            break;
        case WM_SIZE: 
            {
                D3DContext& d3d_ctx = D3DContext::Get();
                if (d3d_ctx.g_pd3dDevice != nullptr && wParam != SIZE_MINIMIZED) {
                    d3d_ctx.CleanupRenderTarget();
                    DXGI_SWAP_CHAIN_DESC1 desc = {};
                    d3d_ctx.g_pSwapChain->GetDesc1(&desc);
                    HRESULT result = d3d_ctx.g_pSwapChain->ResizeBuffers(
                        APP_NUM_BACK_BUFFERS, (UINT)LOWORD(lParam), (UINT)HIWORD(lParam), desc.Format, desc.Flags);
                    assert(SUCCEEDED(result) && "Failed to resize swapchain.");
                    d3d_ctx.CreateRenderTarget();
                }
            }
            return 0;
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU)  // Disable ALT application menu
                return 0;
            break;
        case WM_CLOSE:
            dx_window.close_window = true;
            return 0;
        case WM_DESTROY:
            ::PostQuitMessage(0);
            dx_window.close_window = true;
            return 0;
    }
    return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}
}
