#pragma once

#define NOMINMAX
#include <windows.h>
#include <wrl.h>

namespace app {

struct DXWindow {
    ATOM wndClass = 0;
    HWND hwnd = nullptr;
    bool close_window;
    bool is_fullscreen;

    static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

    bool Init();
    void ShutDown();
    void Update();
    void SetFullscreen(bool enabled, HWND hWnd);

  // Singleton pattern
  public:
    DXWindow(const DXWindow&) = delete;
    DXWindow& operator=(const DXWindow&) = delete;

    inline static DXWindow& Get() {
        static DXWindow instance;
        return instance;
    }

  private:
    DXWindow() = default;
};

}
