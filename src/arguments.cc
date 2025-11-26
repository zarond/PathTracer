#include "arguments.h"

#include "argparse/argparse.hpp"

namespace {

std::string_view no_arguments_message = "A CPU PathTracer for .gltf files.\n"
                                        "Use -h or --help for usage info.";

template<typename T>
T int_to_enum(int v) {
    if (v < 0 || v >= static_cast<int>(T::kNum)) return static_cast<T>(0);
    return static_cast<T>(v);
}

}

namespace app {

ConsoleArgs parse_args(int argc, char* argv[], const fs::path& pwd) {
    ConsoleArgs args;

    argparse::ArgumentParser program("PathTracer", "1.0", argparse::default_arguments::help, false);

    program.add_description("A CPU PathTracer for .gltf files.\n"
                            "Uses first camera in .gltf file to render image and save it as \"snapshot.hdr\" \n"
                            "Choose between different rendering modes and ray intersection acceleration modes");
    program.add_epilog("It's an educational project, so it supports only a limited set of gltf features.");

    program.add_argument("-f", "--file")
        .help("gltf model file location.")
        .required()
        .nargs(1)
        .default_value("");

    program.add_argument("-e", "--env")
        .help("HDRI environment map file location. Or use \"black\" or \"white\"")
        .required()
        .nargs(1)
        .default_value("white");

    program.add_argument("-o", "--output")
        .help("Output file location. Specify .png to save as SDR image, otherwise saves in HDR format")
        .required()
        .nargs(1)
        .default_value("snapshot.hdr");

    program.add_argument("-p", "--program")
        .help("choose program mode:\n 1: RayCaster\n 2: AmbientOcclusion\n 3: PBR")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(1);

    program.add_argument("-a", "--as")
        .help("choose acceleration structure mode:\n 1: Naive\n 2: BVH")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(2);

    program.add_argument("-s", "--samples")
        .help("number of samples per pixel.")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(1);

    program.add_argument("-b", "--bounces")
        .help("max ray bounces.")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(0);

    program.add_argument("-r", "--newrays")
        .help("max new AO rays (AO mode only).")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(0);

    program.add_argument("--env_rot")
        .help("additional env map rotation around UP axis in degrees.")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(0);

    program.add_argument("-m")
        .help("max trianles in a BVH leaf node.")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(8);

    program.add_argument("--width")
        .help("window width")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(800);

    program.add_argument("--height")
        .help("window height")
        .scan<'i', int>()
        .required()
        .nargs(1)
        .default_value(600);

    program.parse_args(argc, argv);

    args.modelPath = program.get<std::string>("-f");
    args.environmentPath = program.get<std::string>("-e");
    args.outputPath = program.get<std::string>("-o");

    auto programMode = program.get<int>("-p") - 1;
    auto accelStructType = program.get<int>("-a") - 1;

    args.programMode = int_to_enum<RayProgramMode>(programMode);
    args.accelStructType = int_to_enum<AccelerationStructureType>(accelStructType);

    auto samplesPerPixel = program.get<int>("-s");
    auto maxRayBounces = program.get<int>("-b");
    auto maxNewRaysPerBounce = program.get<int>("-r");
    
    auto maxTrianglesPerBVHLeaf = program.get<int>("-m");

    args.samplesPerPixel = (samplesPerPixel > 0)? samplesPerPixel : 1;
    args.maxRayBounces = (maxRayBounces >= 0) ? maxRayBounces : 0;
    args.maxNewRaysPerBounce = (maxNewRaysPerBounce >= 0) ? maxNewRaysPerBounce : 0;
    args.envmapRotation = program.get<int>("--env_rot");

    args.maxTrianglesPerBVHLeaf = (maxTrianglesPerBVHLeaf > 0) ? maxTrianglesPerBVHLeaf : 1;

    auto windowWidth = program.get<int>("--width");
    auto windowHeight = program.get<int>("--height");

    args.windowWidth = (windowWidth > 0) ? windowWidth : 1;
    args.windowHeight = (windowHeight > 0) ? windowHeight : 1;

    if (program.is_used("-h")) {
        args.exitImmediately = true;
    }
    else if (args.modelPath == "") {
        std::cout << no_arguments_message << std::endl;
        args.exitImmediately = true;
    }
    if (args.environmentPath == "" || args.environmentPath == "black" || args.environmentPath == "white") {
        args.useDefaultEnv = true;
        if (args.environmentPath == "black") args.defaultEnv = DefaultEnvironment::Black;
        else if (args.environmentPath == "white") args.defaultEnv = DefaultEnvironment::White;
    }

    return args;
}

}