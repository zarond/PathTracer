#pragma once

#include "model_loader.h"
#include "mikktspace.h"

namespace app {

class TangentSpaceHelper {
public:
    TangentSpaceHelper();
    void compute_tangents(Mesh& mesh);

private:
    SMikkTSpaceInterface iface_ = {}; 
    SMikkTSpaceContext context_ = {};
};

}