#include "Common.hlsl"
#include "BRDF.hlsl"

// Raytracing output texture, accessed as a UAV
RWTexture2D<float4> gOutput : register(u0);

// Raytracing acceleration structure, accessed as a SRV
RaytracingAccelerationStructure SceneBVH : register(t0);

// Constant Buffer
ConstantBuffer<RayGenConstantBuffer> g_rayGenCB : register(b0);

// Mesh data buffers
StructuredBuffer<uint> Indices : register(t1, space0);
StructuredBuffer<vertex> Vertices : register(t2, space0);
StructuredBuffer<uint> IndicesOffset : register(t3, space0);
StructuredBuffer<Material> Materials : register(t4, space0);

// Envmap
Texture2D<float4> EnvMap : register(t0, space1);
SamplerState EnvMapSampler : register(s0, space1);
// Default sampler
SamplerState Sampler : register(s1, space1);

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

[shader("raygeneration")] 
void RayGen() {
    // Initialize the ray payload
    HitInfo payload;
    payload.color = float3(0.0, 0.0, 0.0);
    payload.depth = 1;

    // Get the location within the dispatched 2D grid of work items
    // (often maps to pixels, so this could represent a pixel coordinate).
    uint2 launchIndex = DispatchRaysIndex();
    uint2 launchDim = DispatchRaysDimensions();
    float2 lerpValues = (float2)launchIndex / (float2)launchDim;

    float3 rayDir;
    float3 origin;

    // use perspective projection from matrix
    GenerateCameraRay(launchIndex, launchDim, origin, rayDir);

    // Set the ray's extents.
    RayDesc ray;
    ray.Origin = origin;
    ray.Direction = rayDir;
    ray.TMin = 0.001;
    ray.TMax = 10000.0;

    TraceRay(
        SceneBVH,                               // RaytracingAccelerationStructure
        RAY_FLAG_FORCE_OPAQUE,                  // RayFlags
        ~0,                                     // InstanceInclusionMask
        0,                                      // RayContributionToHitGroupIndex
        1,                                      // MultiplierForGeometryContributionToShaderIndex
        0,                                      // MissShaderIndex
        ray, 
        payload
    );

    if (g_rayGenCB.iteration > 1) {
        float3 previousPixel = gOutput[launchIndex].xyz;
        payload.color = lerp(previousPixel, payload.color, g_rayGenCB.invIterationCount);
    }

    gOutput[launchIndex] = float4(payload.color, 1.f);
}

[shader("closesthit")] void ClosestHitAO(inout HitInfo payload, Attributes attr) {
    if (payload.depth <= 0) {
        return;
    }

    const uint mesh_id = InstanceID();
    const uint indices_offset = IndicesOffset[mesh_id];
    const uint base = indices_offset + PrimitiveIndex() * 3;
    const uint3 indices = {Indices[base], Indices[base + 1], Indices[base + 2]};

    float3 vertexNormals[3] = {Vertices[indices[0]].normal.xyz, Vertices[indices[1]].normal.xyz, Vertices[indices[2]].normal.xyz};
    float4 vertexTangent[3] = {Vertices[indices[0]].tangent, Vertices[indices[1]].tangent, Vertices[indices[2]].tangent};

    float3 normal = HitAttribute(vertexNormals, attr);
    float4 tangent = HitAttribute(vertexTangent, attr);

    if (HitKind() == HIT_KIND_TRIANGLE_BACK_FACE) {
        // ray hits backside
        normal *= -1.0f;
    }

    float3 worldRayOrigin = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

    float3x3 ModelMatrix = (float3x3)ObjectToWorld3x4();
    float3x3 NormalMatrixTransposed = (float3x3)WorldToObject3x4();
    float3 worldNormal = mul(normal, NormalMatrixTransposed);
    float3 worldTangent = mul(ModelMatrix, tangent.xyz);
    float3 worldBitangent = cross(worldNormal, worldTangent) * tangent.w;
    float3x3 TBN = construct_TBN(worldTangent, worldBitangent, worldNormal);

    uint2 launchIndex = DispatchRaysIndex();
    uint3 seed = uint3(launchIndex.x, launchIndex.y, g_rayGenCB.frameID);
    float2 jitter = float2(pcg3d16(seed).xy) / float(0xFFFF);

    const int N = g_rayGenCB.maxNewRaysPerBounce;
    const float inv_aoSamples = g_rayGenCB.invMaxNewRaysPerBounce;
    for (int i = 0; i < N; ++i) {
        float2 rand = fibonacci2D(i, inv_aoSamples);
        rand = fmod(rand + jitter, 1.0f);

        float3 new_direction = ImportanceSampleCosDir(rand);
        new_direction = Tangent2World(new_direction, TBN);

        RayDesc ray;
        ray.Origin = worldRayOrigin;
        ray.Direction = new_direction;
        ray.TMin = kEpsilon5;
        ray.TMax = 10000.0;

        HitInfo new_payload;
        new_payload.color = float3(0.0f, 0.0f, 0.0f);
        new_payload.depth = payload.depth - 1;

        TraceRay(
            SceneBVH,                             // RaytracingAccelerationStructure
            RAY_FLAG_FORCE_OPAQUE,                // RayFlags
            ~0,                                   // InstanceInclusionMask
            0,                                    // RayContributionToHitGroupIndex
            1,                                    // MultiplierForGeometryContributionToShaderIndex
            0,                                    // MissShaderIndex
            ray, 
            new_payload
        );

        payload.color += new_payload.color;
    }
    payload.color *= inv_aoSamples;
    payload.depth -= 1;
}

[shader("closesthit")] void ClosestHitRC(inout HitInfo payload, Attributes attr) {
    const uint mesh_id = InstanceID();
    const uint indices_offset = IndicesOffset[mesh_id];
    const uint base = indices_offset + PrimitiveIndex() * 3;
    const uint3 indices = {Indices[base], Indices[base + 1], Indices[base + 2]};

    float2 vertexUV[3] = {Vertices[indices[0]].uv.xy, Vertices[indices[1]].uv.xy, Vertices[indices[2]].uv.xy};

    float2 uv = HitAttribute(vertexUV, attr);

    Material mat = Materials[mesh_id];
    float4 color = mat.baseColorFactor;
    if (mat.baseColorTextureIndex != -1) {
        Texture2D<float4> albedoTex = ResourceDescriptorHeap[mat.baseColorTextureIndex];
        color *= albedoTex.SampleLevel(Sampler, uv, 0);
    }

    //payload.color = worldNormal;
    payload.color = color;
}

[shader("miss")] 
void MissAO(inout HitInfo payload : SV_RayPayload) {
    payload.color = float3(1.0f, 1.0f, 1.0f);
}

[shader("miss")] 
void MissEnvmap(inout HitInfo payload : SV_RayPayload) {
    // compared to Blender, envmap is rotated 180 degrees around Y (Blender's Z) axis, but same as SP
    float y_rotation = g_rayGenCB.envmapRotation;
    float3 dir = WorldRayDirection();
    float2 uv = float2(atan2(-dir.z, -dir.x) + y_rotation, -2.0f * asin(dir.y)) * (1.0f / PI);
    uv = uv * 0.5f + 0.5f;

    payload.color = EnvMap.SampleLevel(EnvMapSampler, uv, 0).rgb;
}