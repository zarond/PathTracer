cmake_minimum_required(VERSION 3.20...4.0)

project(PathTracer VERSION 1.0
                   DESCRIPTION "Educational project of a CPU path tracer"
                   LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# application
set(LIBS_DIR libs)

set(APP_SOURCES
    src/main.cc
    src/model_loader.cc
    src/cpu_framebuffer.cc
    src/arguments.cc
    src/compute_tangents.cc
    src/viewer.cc
    src/ray_program.cc
    src/acceleration_structure.cc
    src/renderer.cc
    src/ui.cc
    src/brdf.cc
)

add_executable(${PROJECT_NAME} ${APP_SOURCES})

# Warning options for compiler
#target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

# doesn't work?
set_target_properties(${PROJECT_NAME} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# libs for application
add_library(imgui STATIC 
    ${LIBS_DIR}/imgui/imgui.cpp
    ${LIBS_DIR}/imgui/imgui_draw.cpp
    ${LIBS_DIR}/imgui/imgui_tables.cpp
    ${LIBS_DIR}/imgui/imgui_widgets.cpp
)
target_include_directories(imgui PUBLIC ${LIBS_DIR}/imgui)

add_library(argparse INTERFACE)
target_include_directories(argparse INTERFACE ${LIBS_DIR}/argparse/include)

add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${LIBS_DIR}/stb)

set(FASTGLTF_COMPILE_AS_CPP20 True)
#set(FASTGLTF_USE_CUSTOM_SMALLVECTOR True)
add_subdirectory(${LIBS_DIR}/fastgltf)

add_library(mikktspace STATIC 
    ${LIBS_DIR}/MikkTSpace/mikktspace.c
)
target_include_directories(mikktspace PUBLIC ${LIBS_DIR}/MikkTSpace)

set(GLM_ENABLE_CXX_20 True)
set(GLM_ENABLE_SIMD_AVX2 "Enable AVX2 optimizations" True)
add_subdirectory(${LIBS_DIR}/glm)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    imgui 
    argparse 
    stb
    fastgltf::fastgltf
    mikktspace
    glm::glm
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    GLM_FORCE_MESSAGES
    GLM_FORCE_INTRINSICS
    GLM_FORCE_ALIGNED_GENTYPES
    GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
)

# tests
set(TEST_SOURCES
    tests/test_main.cc
)
enable_testing()
add_subdirectory("${LIBS_DIR}/googletest")
add_executable(my_tests ${TEST_SOURCES})

target_link_libraries(my_tests
    PRIVATE
    gtest
    gtest_main
)

include(GoogleTest)
gtest_discover_tests(my_tests)