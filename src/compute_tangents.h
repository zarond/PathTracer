#pragma once

#include "mikktspace.h"
#include "model_loader.h"

namespace app {

class TangentSpaceHelper {
  public:
    TangentSpaceHelper();
    void compute_tangents(Mesh& mesh);
    void compute_tangents_no_uv(Mesh& mesh);

  private:
    SMikkTSpaceInterface iface_ = {};
    SMikkTSpaceContext context_ = {};
};

}  // namespace app
