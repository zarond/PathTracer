#include <span>

#include "ui.h"
#include "d3d_context.h"
#include "cpu_framebuffer.h"

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx12.h"

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Forward declare functions from imgui_impl_dx12.cpp
extern void ImGui_ImplDX12_SetupSamplerLinear(ID3D12GraphicsCommandList* command_list);
extern void ImGui_ImplDX12_SetupSamplerNearest(ID3D12GraphicsCommandList* command_list);

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
    wcex.lpszClassName = L"PathTracer";
    wcex.hIconSm = LoadIconW(nullptr, MAKEINTRESOURCEW(IDI_APPLICATION));
    wndClass = RegisterClassExW(&wcex);
    if (wndClass == 0) return false;
    // create window
    hwnd = ::CreateWindowExW(WS_EX_OVERLAPPEDWINDOW | WS_EX_APPWINDOW, (LPCWSTR)wndClass, L"PathTracer project by @zarond",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE, 100, 100, 1280, 720, nullptr, nullptr, wcex.hInstance, nullptr);
    if (hwnd == nullptr) return false;
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
                    d3d_ctx.ResizeSwapchain(LOWORD(lParam), HIWORD(lParam));
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

std::string OpenFileDialog() {     // todo: modernize with IFileDialog
    const int max_path = 1024;     // MAX_PATH;
    char fileName[max_path] = "";  // todo: long file paths

    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = DXWindow::Get().hwnd;
    ofn.lpstrFilter = 
        "All Files\0*.*\0"
        "Gltf Files (*.gltf;*.glb)\0*.gltf;*.glb\0"
        "HDR Files (*.hdr)\0*.hdr\0";
    ofn.lpstrFile = fileName;
    ofn.nMaxFile = max_path;
    ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;

    if (GetOpenFileNameA(&ofn)) return fileName;

    return "";
}

std::string SaveFileDialog() {     // todo: modernize with IFileDialog
    const int max_path = 1024;     // MAX_PATH;
    char fileName[max_path] = "";  // todo: long file paths

    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = DXWindow::Get().hwnd;
    ofn.lpstrFilter =
        "All Files\0*.*\0"
        "PNG File (*.png)\0*.png\0"
        "HDR File (*.hdr)\0*.hdr\0";
    ofn.lpstrFile = fileName;
    ofn.nMaxFile = max_path;
    ofn.lpstrDefExt = "png";
    ofn.Flags = OFN_EXPLORER | OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

    if (GetSaveFileNameA(&ofn)) return fileName;

    return "";
}

bool SliderUInt(const char* label, unsigned int* v, unsigned int v_min, unsigned int v_max, const char* format, ImGuiSliderFlags flags) {
    return ImGui::SliderScalar(label, ImGuiDataType_U32, v, &v_min, &v_max, format, flags);
}

bool InputUInt(const char* label, unsigned int* v, unsigned int step, unsigned int step_fast, ImGuiInputTextFlags flags) {
    const char* format = (flags & ImGuiInputTextFlags_CharsHexadecimal) ? "%08X" : "%d";
    return ImGui::InputScalar(label, ImGuiDataType_U32, (void*)v, (void*)(step > 0 ? &step : NULL),
        (void*)(step_fast > 0 ? &step_fast : NULL), format, flags);
}

void HelpTooltip(const char* msg) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) ImGui::SetTooltip("%s", msg);
}

// For DX12 backend: Callback to modify current sampler
void ImDrawCallback_ImplDX12_SetSamplerNearest(const ImDrawList* parent_list, const ImDrawCmd* cmd) {
    ImGui_ImplDX12_RenderState* state = (ImGui_ImplDX12_RenderState*)ImGui::GetPlatformIO().Renderer_RenderState;
    ImGui_ImplDX12_SetupSamplerNearest(state->CommandList);
}

void ImDrawCallback_ImplDX12_SetSamplerLinear(const ImDrawList* parent_list, const ImDrawCmd* cmd) {
    ImGui_ImplDX12_RenderState* state = (ImGui_ImplDX12_RenderState*)ImGui::GetPlatformIO().Renderer_RenderState;
    ImGui_ImplDX12_SetupSamplerLinear(state->CommandList);
}

void SetupImGuiStyle() {
    float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(::MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));
    ImGui::StyleColorsDark();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);  // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing
                                      // this requires resetting Style + calling this again)
    style.FontScaleDpi = main_scale;  // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We
                                      // leave both here for documentation purpose)
    // Convert colors to be used in sRGB render target
    auto& colors = style.Colors;
    for (auto& col : colors) {
        col.x = srgb_to_linear(col.x);
        col.y = srgb_to_linear(col.y);
        col.z = srgb_to_linear(col.z);
    }
}

}
