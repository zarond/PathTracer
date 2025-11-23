#include <iostream>
#include <string>
#include <chrono>

#include "model_loader.h"
#include "cpu_framebuffer.h"
#include "arguments.h"
#include "viewer.h"

#include <imgui.h>

int main(int argc, char* argv[]) {
    using namespace app;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ConsoleArgs console_arguments = parse_args(argc, argv, fs::current_path());
    if (console_arguments.exitImmediately) {
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();
    Model model;
    {
        ModelLoader loader{};
        auto success = loader.loadFromFile(console_arguments.modelPath);
        if (!success) {
            std::cerr << "Failed to load model from " << console_arguments.modelPath << '\n';
            return 1;
        }
        model = loader.constructModel();
    }
    CPUTexture<hdr_pixel> environment_texture;
    if (console_arguments.useDefaultEnv) {
        environment_texture = (console_arguments.defaultEnv == DefaultEnvironment::White) ? 
            CPUTexture<hdr_pixel>::create_white_texture() :
            CPUTexture<hdr_pixel>::create_black_texture();
    } else {
        environment_texture = CPUTexture<hdr_pixel>(console_arguments.environmentPath);
    }
    
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "loaded in " << diff.count() << " ms." << '\n';
    
    auto render_settings = RenderSettings{
            .samplesPerPixel = console_arguments.samplesPerPixel,
            .maxRayBounces = console_arguments.maxRayBounces,
            .maxNewRaysPerBounce = console_arguments.maxNewRaysPerBounce,
            .maxTrianglesPerBVHLeaf = console_arguments.maxTrianglesPerBVHLeaf,
            .programMode = console_arguments.programMode,
            .accelStructType = console_arguments.accelStructType
    };

    // Create viewer
    Viewer viewer(std::move(model), std::move(environment_texture), render_settings);
    viewer.resize_window(ivec2(console_arguments.windowWidth, console_arguments.windowHeight));

    start = std::chrono::high_resolution_clock::now();
    viewer.render();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "rendered in " << diff.count() << " ms." << '\n';

    viewer.take_snapshot("snapshot.hdr");

    return 0;
}