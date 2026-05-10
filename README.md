# PathTracer

A small path-tracing renderer written in C++ 20 for educational purposes.

You can choose either a CPU Renderer or a DirectX Raytracing (DXR) GPU Renderer in GUI (it is available only from GUI mode).

The program has GUI, but you can also run it in command-line mode to render images without opening a window.

The program requires Windows OS due to usage of Windows API and DirectX.

You can build the program on Linux or MacOS too, but it will disable Windows-specific code - GUI and DXR will be disabled in this case.

You can override CMake option `ON_WINDOWS` to `OFF` if you want to build it on Windows without Windows-specific features.

You can use CMake option `ASAN` to enable Address Sanitizer.

## Usage

Build with CMake and run the `PathTracer` executable.

    git submodule update --init --recursive
    cmake -B ./build -DCMAKE_BUILD_TYPE=Release
    cmake --build ./build --config Release
    cd build
    ctest -C Release --output-on-failure

Use `-h` flag to list options.

By default, without any flags, the program opens a window, allowing loading a Gltf model with a file browser,
but you can also provide command-line arguments.

Provide `-no-gui` flag to disable GUI and run in command-line mode. In this case you must provide at least a model path with `-f`.

Provide a path to a valid GLTF or GLB file with `-f` flag to render the model.

Provide an environment HDR image with `-e` flag or choose `white` or `black` for a default environment map.

If GLTF file contains cameras, the first camera is used, otherwise the model is rendered from the front.

Additional command-line options include: 

 - output image path `-o`
 - ray program mode (simple raycasting, AO only, or full PBR rendering) `-p`
 - acceleration structure type (Naive or Bounding Volume Hierarchies) `-a`
 - resolution `--width` and `--height`
 - number of samples per pixel `-s`
 - maximum number of ray bounces `-b`
 -  and some others...

Try it by running this command in binary (build/Release) folder:

    ./PathTracer -f ./scene.glb -e ./farmland_overcast_1k.hdr -o snapshot.png -s 300 -b 6

This image was rendered in 199 seconds on Intel Core i7-11800H @ 2.30GHz; 

And for DXR GPU rendering mode it took 6.9 seconds on NVIDIA GeForce RTX 3070 Laptop GPU.

![rendered image](example/render.png)

Same scene took 110 seconds in Blender CPU rendering with similar parameters, and 7.25 seconds in Blender GPU OptiX rendering.

Screenshot from GUI mode with AO rendering:

![gui_screen](example/gui_screenshot.png)

Check out this [Video on LinkedIn](https://www.linkedin.com/posts/artur-makoev-85755a2b3_hello-friends-i-would-like-to-show-you-my-activity-7419366989909045249-WOIP) or [this post](https://www.linkedin.com/posts/artur-makoev-85755a2b3_hello-everyone-i-want-to-give-an-update-ugcPost-7447220869531324416-5zeT/) (both are not the latest version of the program) to see how it works in action.

## Features

PBR ray program mode renders images using path tracing with support for:

 - Opaque material = Lambertian diffuse + GGX microsurface with all standard GLTF textures: Albedo, RoughnessMetallic, Normal map
 - Transmissive materials (via `KHR_materials_transmission`) with absorption with several limitations that will be addressed in the future, for example: you can't accurately render nested volumes of transmissive materials that are inside of each other and absorption can have artifacts in case there are other opaque objects inside transmissive volumes. And to properly render transmissive materials refractions you **MUST** specify it with `KHR_materials_volume`. 
 - Emissive materials with `KHR_materials_emissive_strength`
 - `KHR_materials_ior` for IOR manipulation
 - Alpha blending and masking

## Dependencies

This project uses the following libraries:
 - fastgltf
 - glm
 - STB
 - argparse
 - MikkTSpace
 - Dear IMGUI
 - Microsoft DirectX-Headers
 - Google Test

