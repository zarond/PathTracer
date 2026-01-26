cmake_minimum_required(VERSION 3.20...4.0)

project(PathTracer VERSION 1.0
                   DESCRIPTION "Educational project of a CPU path tracer"
                   LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# application
option(NO_WINDOWS "remove Windows specific dependencies, that leaves CPU mode only and no GUI" OFF)

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
    src/render_settings.cc
    src/brdf.cc
)
if(NOT NO_WINDOWS)
    list(APPEND APP_SOURCES
        src/ui.cc
        src/d3d_debug_layer.cc
        src/d3d_context.cc
    )
else()
    add_definitions(-DNO_WINDOWS)
endif()

add_executable(${PROJECT_NAME} ${APP_SOURCES})

# Warning options for compiler
#target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

# doesn't work?
set_target_properties(${PROJECT_NAME} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
#target_compile_definitions(${PROJECT_NAME} PRIVATE 
#    UNICODE
#    _UNICODE
#)

# libs for application
if(NOT NO_WINDOWS)
    add_library(imgui STATIC 
        ${LIBS_DIR}/imgui/imgui.cpp
        ${LIBS_DIR}/imgui/imgui_draw.cpp
        ${LIBS_DIR}/imgui/imgui_tables.cpp
        ${LIBS_DIR}/imgui/imgui_widgets.cpp
        ${LIBS_DIR}/imgui/backends/imgui_impl_win32.cpp
        ${LIBS_DIR}/imgui/backends/imgui_impl_dx12.cpp
    )
    target_include_directories(imgui PUBLIC ${LIBS_DIR}/imgui)
endif()

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

if(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(GLM_ENABLE_SIMD_NEON "Enable ARM NEON optimizations" True)
else()
    set(GLM_ENABLE_SIMD_AVX2 "Enable AVX2 optimizations" True)
endif()

add_subdirectory(${LIBS_DIR}/glm)

target_link_libraries(${PROJECT_NAME} PRIVATE
    argparse 
    stb
    fastgltf::fastgltf
    mikktspace
    glm::glm
)
if(NOT NO_WINDOWS)
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        imgui 
        d3d12
        dxgi
    )
endif()

if((CMAKE_BUILD_TYPE STREQUAL "Debug") AND (NOT NO_WINDOWS))
    target_link_libraries(${PROJECT_NAME} PRIVATE dxguid)
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
    #GLM_FORCE_MESSAGES
    GLM_FORCE_INTRINSICS
    #GLM_FORCE_INLINE 
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
gtest_discover_tests(my_tests WORKING_DIRECTORY $<TARGET_FILE_DIR:PathTracer>)

# copy example assets to build dir
add_custom_command(
    TARGET  ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/example/scene.glb
            $<TARGET_FILE_DIR:PathTracer>/scene.glb
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/example/farmland_overcast_1k.hdr
            $<TARGET_FILE_DIR:PathTracer>/farmland_overcast_1k.hdr
    COMMAND ${CMAKE_COMMAND} -E echo "Copying scene file and HDR from /example to $<TARGET_FILE_DIR:PathTracer>"
    COMMENT "Copying scene file and HDR from /example to $<TARGET_FILE_DIR:PathTracer>")
