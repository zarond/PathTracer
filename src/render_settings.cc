#include "render_settings.h"

#include <glm/glm.hpp>

namespace app {

RenderSettings::RenderSettings(const ConsoleArgs& args) :
    samplesPerPixel(args.samplesPerPixel),
    maxRayBounces(args.maxRayBounces),
    maxNewRaysPerBounce(args.maxNewRaysPerBounce),
    maxTrianglesPerBVHLeaf(args.maxTrianglesPerBVHLeaf),
    envmapRotation(glm::radians(static_cast<float>(args.envmapRotation))),
    programMode(args.programMode),
    accelStructType(args.accelStructType) 
{}

}
