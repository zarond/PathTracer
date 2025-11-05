#pragma once

#include <filesystem>
#include <optional>
#include <variant>

namespace {
    namespace fs = std::filesystem;
}

namespace app {

enum class RayProgramMode {
    RayCaster,
    AmbientOcclusion,

    kNum
};

enum class AccelerationStructureType {
    Naive,

    kNum
};

struct ConsoleArgs {
    fs::path modelPath;
    fs::path environmentPath;
    bool useDefaultEnv = false;

    RayProgramMode programMode = RayProgramMode::RayCaster;
    AccelerationStructureType accelStructType = AccelerationStructureType::Naive;

    unsigned int samplesPerPixel = 1;
    unsigned int maxRayBounces = 0;
    unsigned int maxNewRaysPerBounce = 0;

    unsigned int windowWidth = 800;
    unsigned int windowHeight = 600;

    bool exitImmediately = false;
};

ConsoleArgs parse_args(int argc, char* argv[], const fs::path& pwd);

}