#pragma once

#include <string>

#define NOMINMAX
#include <windows.h>

#include "imgui.h"
#include <span>
#include <type_traits>

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

std::string OpenFileDialog();
std::string SaveFileDialog();

bool imgui_combo(const char* label, std::span<const char* const> items, auto& current_index) {
    bool changed = false;

    const auto current_index_size_t = static_cast<std::size_t>(current_index);
    const auto preview_value = current_index_size_t < items.size() ? items[current_index_size_t] : "---";

    if (ImGui::BeginCombo(label, preview_value)) {
        for (size_t index = 0; const auto& item : items) {
            bool is_selected = (current_index_size_t == index);
            if (ImGui::Selectable(item, is_selected)) {
                changed = true;
                current_index = static_cast<std::decay_t<decltype(current_index)>>(index);
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }

            ++index;
        }
        ImGui::EndCombo();
    }

    return changed;
}

bool SliderUInt(const char* label, unsigned int* v, unsigned int v_min, unsigned int v_max, const char* format = (const char*)0,
    ImGuiSliderFlags flags = 0);

bool InputUInt(const char* label, unsigned int* v, unsigned int step = 1, unsigned int step_fast = 100, ImGuiInputTextFlags flags = 0);

void HelpTooltip(const char* msg);

void ImDrawCallback_ImplDX12_SetSamplerNearest(const ImDrawList* parent_list, const ImDrawCmd* cmd);
void ImDrawCallback_ImplDX12_SetSamplerLinear(const ImDrawList* parent_list, const ImDrawCmd* cmd);

void SetupImGuiStyle();
}
