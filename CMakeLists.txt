cmake_minimum_required(VERSION 3.20...4.0)

project(PathTracer VERSION 1.0
                   DESCRIPTION "Educational project of a CPU path tracer"
                   LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(ON_WINDOWS_DEFAULT OFF)
if(WIN32)
    set(ON_WINDOWS_DEFAULT ON)
endif()

# application
option(ON_WINDOWS "Allow Windows specific dependencies, this enables DirectX raytracing mode and GUI" ${ON_WINDOWS_DEFAULT})
option(ASAN "Turn On Address Sanitizer" OFF)

message(STATUS "${PROJECT_NAME}: ON_WINDOWS option is currently: ${ON_WINDOWS}")
message(STATUS "${PROJECT_NAME}: ASAN option is currently: ${ASAN}")

set(LIBS_DIR libs)

set(SHADER_SOURCES
    RaytracingShaders   RaytracingShaders.hlsl      lib_6_6      
    VS_Main             RasterizationShaders.hlsl   vs_6_6
    PS_Main             RasterizationShaders.hlsl   ps_6_6
    VS_Background       RasterizationShaders.hlsl   vs_6_6
    PS_Background       RasterizationShaders.hlsl   ps_6_6
    CS_DFG_Lut          DFGLut.hlsl                 cs_6_6
    CS_Diffuse_Lut      CubemapsLut.hlsl            cs_6_6
)

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
if(ON_WINDOWS)
    list(APPEND APP_SOURCES
        src/ui.cc
        src/d3d_debug_layer.cc
        src/d3d_context.cc
        src/GPU/GPU_model.cc
        src/GPU/GPU_renderer.cc
        src/GPU/DXR_pipeline.cc
        src/GPU/Raster_pipeline.cc
        src/GPU/DFG_Lut_helper.cc
        src/GPU/Cubemaps_helper.cc
    )
endif()

add_executable(${PROJECT_NAME} ${APP_SOURCES})

if(ON_WINDOWS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WINDOWS_SPECIFIC)
endif()

# compile hlsl shaders to dxil
if(ON_WINDOWS)
    #find_program(DXC dxc REQUIRED) # this just doesn't work for some reason in PowerShell and cmd, works only in Developer PowerShell for VS'
    Set(DXC dxc) # in contrary, this works just fine
    if(NOT DXC)
        message(FATAL_ERROR "DXC compiler 'dxc' not found. Install DXC or add it to PATH.")
    endif()

    # Determine flags based on build type at configure-time
    set(DXC_FLAGS "")

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        # Debug: Full info, no optimizations
        # set(DXC_FLAGS -Zi -Qembed_debug -Od) # For some reason this line causes Debug to output black pixels
    elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
        # RelWithDebInfo: Full info, optimizations enabled
        set(DXC_FLAGS -Zi -Qembed_debug)
    endif()

    # compile shaders
    list(LENGTH SHADER_SOURCES len)
    math(EXPR last "${len} - 1")
    foreach(i RANGE 0 ${last} 3)
        math(EXPR i1 "${i} + 1")
        math(EXPR i2 "${i} + 2")

        list(GET SHADER_SOURCES ${i}  SHADER)
        list(GET SHADER_SOURCES ${i1} SHADER_SOURCE_FILE_NAME)
        list(GET SHADER_SOURCES ${i2} SHADER_STAGE_TYPE)
        # compile into build dir (config-independent)
        set(COMPILED_SHADER ${CMAKE_CURRENT_BINARY_DIR}/${SHADER}.dxil)

        # todo: better yet, compile into object file and link into executable, use extern const in code
        add_custom_command(
            OUTPUT ${COMPILED_SHADER}
            COMMAND ${DXC}
                    -T ${SHADER_STAGE_TYPE}
                    ${DXC_FLAGS}
                    -E ${SHADER}
                    -Fo ${COMPILED_SHADER}
                    ${CMAKE_SOURCE_DIR}/src/GPU/Shaders/${SHADER_SOURCE_FILE_NAME}
            DEPENDS src/GPU/Shaders/${SHADER_SOURCE_FILE_NAME}
            COMMENT "Compiling shader ${SHADER}"
        )
        list(APPEND COMPILED_SHADERS ${COMPILED_SHADER})

        # copy compiled shader next to the executable after the project is built
        # $<TARGET_FILE_DIR:${PROJECT_NAME}> resolves per-config in multi-config generators
        add_custom_command(
            TARGET ${PROJECT_NAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${COMPILED_SHADER}
                    $<TARGET_FILE_DIR:${PROJECT_NAME}>/${SHADER}.dxil
            COMMENT "Copying ${SHADER}.dxil from ${COMPILED_SHADER} to target $<TARGET_FILE_DIR:${PROJECT_NAME}>/${SHADER}.dxil"
            VERBATIM
        )
    endforeach()

    add_custom_target(Shaders ALL DEPENDS ${COMPILED_SHADERS})
    add_dependencies(${PROJECT_NAME} Shaders)
endif()

# Warning options for compiler
#target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

# only works for Visual Studio Generators
set_target_properties(${PROJECT_NAME} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
#target_compile_definitions(${PROJECT_NAME} PRIVATE 
#    UNICODE
#    _UNICODE
#)

# libs for application
if(ON_WINDOWS)
    add_library(imgui STATIC 
        ${LIBS_DIR}/imgui/imgui.cpp
        ${LIBS_DIR}/imgui/imgui_draw.cpp
        ${LIBS_DIR}/imgui/imgui_tables.cpp
        ${LIBS_DIR}/imgui/imgui_widgets.cpp
        ${LIBS_DIR}/imgui/backends/imgui_impl_win32.cpp
        ${LIBS_DIR}/imgui/backends/imgui_impl_dx12.cpp
    )
    target_include_directories(imgui PUBLIC ${LIBS_DIR}/imgui)
    # Directx12 helper headers (and <d3d12.h> is from here too named <directx/d3d12.h>)
    add_subdirectory(${LIBS_DIR}/DirectX-Headers)
    target_link_libraries(${PROJECT_NAME} PRIVATE Microsoft::DirectX-Headers)
endif()

add_library(argparse INTERFACE)
target_include_directories(argparse INTERFACE ${LIBS_DIR}/argparse/include)

add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${LIBS_DIR}/stb)

set(FASTGLTF_COMPILE_AS_CPP20 True)
#set(FASTGLTF_USE_CUSTOM_SMALLVECTOR True)
add_subdirectory(${LIBS_DIR}/fastgltf)

# add ASAN for builds if ASAN option is enabled
if(ASAN)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
        target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
        target_compile_options(fastgltf PRIVATE -fsanitize=address -fno-omit-frame-pointer)
        target_link_options(fastgltf PRIVATE -fsanitize=address)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_options(${PROJECT_NAME} PRIVATE /fsanitize=address)
        target_compile_options(fastgltf PRIVATE /fsanitize=address)
    endif()
endif()

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
if(ON_WINDOWS)
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        imgui 
        d3d12
        dxgi
    )
endif()

if((CMAKE_BUILD_TYPE STREQUAL "Debug") AND ON_WINDOWS)
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
