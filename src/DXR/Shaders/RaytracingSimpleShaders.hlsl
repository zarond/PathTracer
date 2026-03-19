#include "Common.hlsl"
#include "BRDF.hlsl"

// Raytracing output texture, accessed as a UAV
RWTexture2D<float4> gOutput : register(u0);

// Raytracing acceleration structure, accessed as a SRV
RaytracingAccelerationStructure SceneBVH : register(t0);

// Constant Buffer
ConstantBuffer<RayGenConstantBuffer> g_rayGenCB : register(b0);

// Generate a ray in world space for a camera pixel corresponding to an index from the dispatched 2D grid.
inline void GenerateCameraRay(uint2 index, uint2 dims, out float3 origin, out float3 direction) {
    float2 xy = index + 0.5f + g_rayGenCB.subpixel_offset;  // center in the middle of the pixel.
    float2 screenPos = ( xy / dims ) * 2.0f - 1.0f;

    // Invert Y for DirectX-style coordinates.
    screenPos.y = -screenPos.y;

    // Unproject the pixel coordinate into a ray.
    float4 world = mul(g_rayGenCB.projectionToWorld, float4(screenPos, 0, 1));

    world.xyz /= world.w;
    origin = g_rayGenCB.cameraPosition.xyz;
    direction = normalize(world.xyz - origin);
}

typedef BuiltInTriangleIntersectionAttributes Attributes;

[shader("raygeneration")] 
void RayGen() {
    // Initialize the ray payload
    HitInfo payload;
    payload.colorAndDistance = float4(0.0, 0.0, 0.0, 1);

    // Get the location within the dispatched 2D grid of work items
    // (often maps to pixels, so this could represent a pixel coordinate).
    uint2 launchIndex = DispatchRaysIndex();
    uint2 launchDim = DispatchRaysDimensions();
    float2 lerpValues = (float2)launchIndex / (float2)launchDim;

    float3 rayDir;
    float3 origin;

    // use perspective projection from matrix
    GenerateCameraRay(launchIndex, launchDim, origin, rayDir);

    // Trace the ray.
    // Set the ray's extents.
    RayDesc ray;
    ray.Origin = origin;
    ray.Direction = rayDir;
    ray.TMin = 0.001;
    ray.TMax = 10000.0;

    TraceRay(
        SceneBVH,                               // RaytracingAccelerationStructure
        RAY_FLAG_CULL_BACK_FACING_TRIANGLES,    // RayFlags
        ~0,                                     // InstanceInclusionMask
        0,                                      // RayContributionToHitGroupIndex
        1,                                      // MultiplierForGeometryContributionToShaderIndex
        0,                                      // MissShaderIndex
        ray, 
        payload
    );

    gOutput[launchIndex] = float4(payload.colorAndDistance.rgb, 1.f);
}

[shader("closesthit")] void ClosestHit(inout HitInfo payload, Attributes attr) {
    //float3 barycentrics = float3(1 - attr.barycentrics.x - attr.barycentrics.y, attr.barycentrics.x, attr.barycentrics.y);
    //payload.colorAndDistance = float4(1, 1, 0, RayTCurrent());
    float d = RayTCurrent() / 10.0f;
    //float d = 0.0f;

    //float3 worldRayOrigin = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
    //float3 worldNormal = mul(attr.normal, (float3x3)ObjectToWorld3x4());
    //float3 worldTangent = mul(attr.tangent, (float3x3)ObjectToWorld3x4());
    //float3 worldBitangent = cross(worldNormal, worldTangent) * attr.tangent.w;
    //float3x3 TBN = construct_TBN(worldTangent, worldBitangent, worldNormal);

    //for (int i = 0; i < 32; ++i){
    //    float inv_aoSamples = 1.0f / 32.0f;
    //    float2 rand = fibonacci2D(i, inv_aoSamples);

    //    float3 new_direction = ImportanceSampleCosDir(rand);
    //    new_direction = Tangent2World(new_direction, TBN);

    //    RayDesc ray;
    //    ray.Origin = worldRayOrigin;
    //    ray.Direction = new_direction;
    //    ray.TMin = 0.001;
    //    ray.TMax = 10000.0;

    //    TraceRay(
    //        SceneBVH,                             // RaytracingAccelerationStructure
    //        RAY_FLAG_CULL_BACK_FACING_TRIANGLES,  // RayFlags
    //        ~0,                                   // InstanceInclusionMask
    //        0,                                    // RayContributionToHitGroupIndex
    //        1,                                    // MultiplierForGeometryContributionToShaderIndex
    //        0,                                    // MissShaderIndex
    //        ray, 
    //        payload
    //    );

    //    d += payload.colorAndDistance.a;
    //}
    //d /= 32.0f;

    payload.colorAndDistance = float4(d, d, d, d);
}

[shader("miss")] 
void Miss(inout HitInfo payload : SV_RayPayload) {
    payload.colorAndDistance = float4(1.0f, 1.0f, 1.0f, 1.0f);
}
