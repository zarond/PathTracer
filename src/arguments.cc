#include "arguments.h"

#include "argparse/argparse.hpp"

namespace {
    std::string_view no_arguments_message = "A CPU PathTracer for .gltf files.\n"
                                            "Use -h or --help for usage info.";
}

namespace app {

ConsoleArgs parse_args(int argc, char* argv[], const fs::path& pwd) {
    ConsoleArgs args;

    argparse::ArgumentParser program("PathTracer", "1.0", argparse::default_arguments::help, false);

    program.add_argument("-f", "--file")
        .help("gltf model file location.")
        .required()
        .nargs(1)
        .default_value("");

    program.add_argument("-e", "--env")
        .help("HDRI environment map file location.")
        .required()
        .nargs(1)
        .default_value("");

    program.parse_args(argc, argv);

    args.modelPath = program.get<std::string>("-f");
    args.environmentPath = program.get<std::string>("-e");

    if (program.is_used("-h")) {
        args.exitImmediately = true;
    }
    else if (args.modelPath == "") {
        std::cout << no_arguments_message << std::endl;
        args.exitImmediately = true;
    }
    if (args.environmentPath == "") args.useDefaultEnv = true;

    return args;
}

}