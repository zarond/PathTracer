#include "Reload_shaders.h"

#include <iostream>
#include <tuple>

#include "Bilateral_filter.h"
#include "Cubemaps_helper.h"
#include "DFG_Lut_helper.h"
#include "DXR_pipeline.h"
#include "GTAO_helper.h"
#include "Kawase_blur_helper.h"
#include "Mipmaps_helper.h"
#include "Raster_pipeline.h"
#include "Reprojection_helper.h"
#include "Silhouette_helper.h"
#include "SSR_helper.h"

namespace app {

template <typename... Ts>
void RunAllTasks() {
    (Ts::Reload(), ...);
}

void ReloadShaders() { 
    RunAllTasks
        <Bilateral_filter, 
        EnvCube_helper,
        DFG_Lut_helper,
        DXR_pipeline,
        GTAO_helper,
        Kawase_blur_helper,
        Mipmaps_helper,
        Raster_pipeline,
        Reprojection_helper,
        Silhouette_helper,
        SSR_helper>(); 
}

}