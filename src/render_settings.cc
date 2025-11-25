#include "render_settings.h"

#include <glm/gtc/constants.hpp>

namespace app {

RenderSettings::RenderSettings(const ConsoleArgs& args) :
    samplesPerPixel(args.samplesPerPixel),
    maxRayBounces(args.maxRayBounces),
    maxNewRaysPerBounce(args.maxNewRaysPerBounce),
    maxTrianglesPerBVHLeaf(args.maxTrianglesPerBVHLeaf),
    envmapRotation(static_cast<float>(args.envmapRotation)),
    programMode(args.programMode),
    accelStructType(args.accelStructType) 
{
    envmapRotation *= glm::pi<float>() / 180.f;
}

}
