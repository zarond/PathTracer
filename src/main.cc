#include <iostream>
#include <string>
#include <chrono>

#include "model_loader.h"
#include "arguments.h"
#include "viewer.h"

#include <imgui.h>

int main(int argc, char* argv[]) {
    using namespace app;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ConsoleArgs console_arguments = parse_args(argc, argv, fs::current_path());
    if (console_arguments.exitImmediately) {
        //return 1; // Ignore for now
    }

    console_arguments.modelPath = "C:/Users/artur/Documents/GitHub/PathTracer/data/Diesel_shoe.gltf"; // For testing only
    //console_arguments.modelPath = "C:/Users/artur/Documents/GitHub/PathTracer/data/boxes_scene/boxes.gltf"; // For testing only
    
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
    // Todo: load environment
    CPUTexture<hdr_pixel> environment_texture = console_arguments.useDefaultEnv ?
        CPUTexture<hdr_pixel>::create_white_texture() :
        CPUTexture<hdr_pixel>(console_arguments.environmentPath);
    
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "loaded in " << diff.count() << " ms." << '\n';
    
    // Create viewer
    Viewer viewer(std::move(model), std::move(environment_texture));

    viewer.set_render_settings(RenderSettings{ .samplesPerPixel = 4, .maxRayBounces = 1 }); // For testing only

    start = std::chrono::high_resolution_clock::now();
    viewer.render();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "rendered in " << diff.count() << " ms." << '\n';

    viewer.take_snapshot("snapshot.hdr");

    return 0;
}