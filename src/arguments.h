#pragma once

#include <filesystem>
#include <optional>
#include <variant>

namespace {
    namespace fs = std::filesystem;
}

namespace app {

struct ConsoleArgs {
    fs::path modelPath;
    fs::path environmentPath;
    bool useDefaultEnv = false;
    bool exitImmediately = false;
};

ConsoleArgs parse_args(int argc, char* argv[], const fs::path& pwd);

}