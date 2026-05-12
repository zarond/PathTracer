#include "BRDF.hlsl"
#include "Common.hlsl"

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

[shader("miss")] void MissEnvmap(inout HitInfo payload : SV_RayPayload);

// Generate a ray in world space for a camera pixel corresponding to an index from the dispatched 2D grid.
inline void GenerateCameraRay(uint2 index, uint2 dims, out float3 origin, out float3 direction, int i) {
    float2 xy = index + g_rayGenCB.subpixel_offset;
    if (g_rayGenCB.samplesPerPixel == 1) {
        xy += 0.5f;  // center in the middle of the pixel.
    } else {
        xy += fibonacci2D(i, g_rayGenCB.invSamplesPerPixel);
    }
    float2 screenPos = (xy / dims) * 2.0f - 1.0f;

    // Invert Y for DirectX-style coordinates.
    screenPos.y = -screenPos.y;

    // Unproject the pixel coordinate into a ray.
    float4 world = mul(g_rayGenCB.projectionToWorld, float4(screenPos, 0, 1));

    world.xyz /= world.w;
    origin = g_rayGenCB.cameraPosition.xyz;
    direction = normalize(world.xyz);
}

[shader("raygeneration")] 
void RayGen() {
    // Get the location within the dispatched 2D grid of work items
    // (often maps to pixels, so this could represent a pixel coordinate).
    uint2 launchIndex = DispatchRaysIndex();
    uint2 launchDim = DispatchRaysDimensions();

    float3 accumulatedColor = float3(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < g_rayGenCB.samplesPerPixel; ++i) {
        float3 rayDir;
        float3 origin;
        // use perspective projection from matrix
        GenerateCameraRay(launchIndex, launchDim, origin, rayDir, i);

        // Initialize the ray payload
        HitInfo payload;
        payload.color = float3(0.0, 0.0, 0.0);
        payload.absorption = float3(1.0, 1.0, 1.0);
        payload.depth = g_rayGenCB.maxRayBounces;
        payload.iteration = i;

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
        accumulatedColor += payload.color;
    }
    accumulatedColor *= g_rayGenCB.invSamplesPerPixel;

    if (g_rayGenCB.iteration > 1) {
        float3 previousPixel = gOutput[launchIndex].xyz;
        accumulatedColor = lerp(previousPixel, accumulatedColor, g_rayGenCB.invIterationCount);
    }

    gOutput[launchIndex] = float4(accumulatedColor, 1.f);
}

inline uint3 GetIndices() {
    const uint mesh_id = InstanceID();
    const uint indices_offset = IndicesOffset[mesh_id];
    const uint base = indices_offset + PrimitiveIndex() * 3;
    return uint3(Indices[base], Indices[base + 1], Indices[base + 2]);
}

inline void ContinueTrace(inout HitInfo payload, const float alpha, const float3 old_payload_absorption, const int new_depth) {
    RayDesc ray;
    ray.Origin = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
    ray.Direction = WorldRayDirection();
    ray.TMin = 0.001f;
    ray.TMax = 10000.0f;

    payload.absorption = old_payload_absorption * (1.0f - alpha);
    payload.depth = new_depth;

    if (payload.depth > 0) {
        TraceRay(SceneBVH, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);
    } else {
        MissEnvmap(payload);
    }
}

[shader("closesthit")]
void ClosestHitAO(inout HitInfo payload, Attributes attr) {
    const uint3 indices = GetIndices();

    const uint mesh_id = InstanceID();
    Material material = Materials[mesh_id];

    float3 vertexNormals[3] = {Vertices[indices[0]].normal.xyz, Vertices[indices[1]].normal.xyz, Vertices[indices[2]].normal.xyz};
    float4 vertexTangent[3] = {Vertices[indices[0]].tangent, Vertices[indices[1]].tangent, Vertices[indices[2]].tangent};
    float2 vertexUV[3] = {Vertices[indices[0]].uv.xy, Vertices[indices[1]].uv.xy, Vertices[indices[2]].uv.xy};

    float3 normal = HitAttribute(vertexNormals, attr);
    float4 tangent = HitAttribute(vertexTangent, attr);
    float2 uv = HitAttribute(vertexUV, attr);

    if (HitKind() == HIT_KIND_TRIANGLE_BACK_FACE) {
        if (material.doubleSided) normal *= -1.0f;
    }

    float3 worldRayOrigin = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

    float3x3 ModelMatrix = (float3x3)ObjectToWorld3x4();
    float3x3 NormalMatrixTransposed = (float3x3)WorldToObject3x4();
    float3 worldNormal = mul(normal, NormalMatrixTransposed);
    float3 worldTangent = mul(ModelMatrix, tangent.xyz);
    float3 worldBitangent = cross(worldNormal, worldTangent) * tangent.w;
    float3x3 TBN = construct_TBN(worldTangent, worldBitangent, worldNormal);

    bool has_normal_map;
    float3 normal_map_color = sample_normals(material, uv, Sampler, has_normal_map);
    if (has_normal_map) {
        float3 normal_vector = normal_map_sample_to_world(normal_map_color, TBN);
        TBN = construct_TBN(TBN[0], TBN[1], normal_vector);  // re-construct TBN with normal from normal map
    }

    uint2 launchIndex = DispatchRaysIndex();
    uint3 seed = uint3(launchIndex.x, launchIndex.y, payload.iteration + g_rayGenCB.frameID * g_rayGenCB.samplesPerPixel);
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

        uint flags = RAY_FLAG_CULL_BACK_FACING_TRIANGLES | RAY_FLAG_FORCE_NON_OPAQUE | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER |
                     RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH;

        TraceRay(
            SceneBVH,       // RaytracingAccelerationStructure
            flags,          // RayFlags
            ~0,             // InstanceInclusionMask
            0,              // RayContributionToHitGroupIndex
            1,              // MultiplierForGeometryContributionToShaderIndex
            0,              // MissShaderIndex
            ray, 
            payload
        );
    }
    payload.color *= inv_aoSamples;
}

[shader("closesthit")]
void ClosestHitRC(inout HitInfo payload, Attributes attr) {
    const uint3 indices = GetIndices();

    float2 vertexUV[3] = {Vertices[indices[0]].uv.xy, Vertices[indices[1]].uv.xy, Vertices[indices[2]].uv.xy};

    float2 uv = HitAttribute(vertexUV, attr);

    const uint mesh_id = InstanceID();
    Material mat = Materials[mesh_id];
    float4 color = sample_albedo(mat, uv, Sampler);
    float alpha = color.w;
    if (mat.alphaBlending == 0) {
        alpha = (alpha < mat.alpha_cutoff) ? 0.0f : 1.0f;
    }
    payload.color += payload.absorption * color.rgb * alpha;
    if (alpha >= 1.0f - kEpsilon5) {
        return;
    }
    // Continue Trace
    RayDesc ray;
    ray.Origin = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
    ray.Direction = WorldRayDirection();
    ray.TMin = 0.001f;
    ray.TMax = 10000.0f;

    payload.absorption *= (1.0f - alpha);
    payload.depth -= 1;

    if (payload.depth > 0) {
        TraceRay(SceneBVH, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);
    } else {
        MissEnvmap(payload);
    }
}

[shader("anyhit")] 
void AnyHitRC(inout HitInfo payload, Attributes attr) {
    const uint3 indices = GetIndices();

    float2 vertexUV[3] = {Vertices[indices[0]].uv.xy, Vertices[indices[1]].uv.xy, Vertices[indices[2]].uv.xy};

    float2 uv = HitAttribute(vertexUV, attr);

    const uint mesh_id = InstanceID();
    Material mat = Materials[mesh_id];
    float alpha = mat.baseColorFactor.w;
    if (mat.baseColorTextureIndex != -1) {
        Texture2D<float4> albedoTex = ResourceDescriptorHeap[mat.baseColorTextureIndex];
        alpha *= albedoTex.SampleLevel(Sampler, uv, 0).w;
    }
    if (alpha < mat.alpha_cutoff) {
        IgnoreHit();
    }
}

[shader("closesthit")] 
void ClosestHitPBR(inout HitInfo payload, Attributes attr) {
    const uint3 indices = GetIndices();

    const uint mesh_id = InstanceID();
    Material material = Materials[mesh_id];

    float3 vertexLocalPos[3] = {Vertices[indices[0]].position.xyz, Vertices[indices[1]].position.xyz, Vertices[indices[2]].position.xyz};
    float3 vertexNormals[3] = {Vertices[indices[0]].normal.xyz, Vertices[indices[1]].normal.xyz, Vertices[indices[2]].normal.xyz};
    float4 vertexTangent[3] = {Vertices[indices[0]].tangent, Vertices[indices[1]].tangent, Vertices[indices[2]].tangent};
    float2 vertexUV[3] = {Vertices[indices[0]].uv.xy, Vertices[indices[1]].uv.xy, Vertices[indices[2]].uv.xy};

    float3 position = HitAttribute(vertexLocalPos, attr);
    float3 normal = HitAttribute(vertexNormals, attr);
    float4 tangent = HitAttribute(vertexTangent, attr);
    float2 uv = HitAttribute(vertexUV, attr);

    bool backface_hit = (HitKind() == HIT_KIND_TRIANGLE_BACK_FACE);
    bool exiting_volume = false;
    if (backface_hit) {
        if (material.doubleSided) {
            normal *= -1.0f;
        } else if (material.hasVolume) {
            exiting_volume = true;
        }
    }

    float3 old_payload_absorption = payload.absorption;
    int old_depth = payload.depth;

    float4 albedo_color = sample_albedo(material, uv, Sampler);
    float alpha = albedo_color.w;
    if (material.alphaBlending == 0) {
        alpha = (alpha < material.alpha_cutoff) ? 0.0f : 1.0f;
    }
    if (alpha < 1.0f - kEpsilon5) {
        ContinueTrace(payload, alpha, old_payload_absorption, old_depth - 1);
    }
    if (alpha == 0.0f) {
        return;
    }

    // todo: now program can't apply absorption if ray was reflected of an object that is inside volume mesh
    // only applies absorption to part of path that directly exits (hits backface of) volume
    float3 attenuation =
        exiting_volume ? min(exp(-material.attenuationFactor.rgb * RayTCurrent()), 1.0f) : 1.0f;  // Volume absorption

    float3 emissive = sample_emissive(material, uv, Sampler);
    payload.color += old_payload_absorption * emissive * attenuation * alpha;
    if (old_depth == 0) {
        return;
    }

    float3 v = -WorldRayDirection();

    float3x3 ModelMatrix = (float3x3)ObjectToWorld3x4();
    float3x3 NormalMatrixTransposed = (float3x3)WorldToObject3x4();
    float3 worldNormal = mul(normal, NormalMatrixTransposed);
    float3 worldPosition = mul(ObjectToWorld3x4(), float4(position, 1.0f));
    float3 worldTangent = mul(ModelMatrix, tangent.xyz);
    float3 worldBitangent = cross(worldNormal, worldTangent) * tangent.w;

    bool has_normal_map;
    float3 normal_map_color = sample_normals(material, uv, Sampler, has_normal_map);

    float3x3 TBN = handle_TBN_creation(NormalMatrixTransposed, normal_map_color, has_normal_map, 
        worldTangent, worldBitangent, worldNormal, 
            v, material.doubleSided, exiting_volume, backface_hit, vertexLocalPos[0], vertexLocalPos[1], vertexLocalPos[2]);

    worldNormal = normalize(worldNormal);

    float transmission = sample_transmission(material, uv, Sampler);
    float2 ORM = sample_roughness_metallic(material, uv, Sampler);
    float3 diffuse_color = (1.0f - ORM.y) * albedo_color.rgb;

    float3 f0 = lerp(material.dielectric_f0, albedo_color.rgb, ORM.y);
    const float3 f90 = 1.0f;
    const float roughness = max(ORM.x, 0.002f);  // trying to avoid numerical issues with very low roughness
    const float linear_roughness = roughness * roughness;

    uint2 launchIndex = DispatchRaysIndex();
    uint3 seed1 = uint3(launchIndex.x, launchIndex.y,
        old_depth + g_rayGenCB.maxRayBounces * (payload.iteration + g_rayGenCB.frameID * g_rayGenCB.samplesPerPixel));
    uint3 seed2 = uint3(launchIndex.x, launchIndex.y,
        old_depth + g_rayGenCB.maxRayBounces * (payload.iteration + (g_rayGenCB.frameID + 1) * g_rayGenCB.samplesPerPixel));

    const bool sample_diffuse = any(diffuse_color * (1.0f - transmission) != 0.0f);
    if (sample_diffuse) {  // diffuse
        float2 rand = pcg3d(seed1).xy;

        float3 l = ImportanceSampleCosDir(rand);
        l = normalize(Tangent2World(l, TBN));  // normalizing for better accuracy

        const float3 h = normalize(v + l);
        float LdH = clamp(dot(l, h), 0.0f, 1.0f);

        float3 F = 1.0f - fresnel_schlick(f0, f90, LdH);
        float3 new_pos = worldPosition + worldNormal * kEpsilon5;  // offset to avoid self-intersection
        float3 next_payload_absorption = old_payload_absorption * F * diffuse_color * (1.0f - transmission) * attenuation * alpha;

        RayDesc ray;
        ray.Origin = new_pos;
        ray.Direction = l;
        ray.TMin = 0.0;
        ray.TMax = 10000.0;

        payload.absorption = next_payload_absorption;
        payload.depth = old_depth - 1;

        TraceRay(SceneBVH, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);
    }
    {
        // same micro-normal for both specular reflection and transmission
        float2 rand = pcg3d(seed2).xy;
        float3 m = importanceSampleGGX(rand, linear_roughness);

        m = Tangent2World(m, TBN);

        float3 N = TBN[2];
        float3 new_pos_offset_dir = worldNormal;
        if (exiting_volume) {
            m *= -1.0f;
            N *= -1.0f;
            new_pos_offset_dir *= -1.0f;
        }

        const float interface_ior = (!exiting_volume) ? (1.0f / material.ior) : material.ior;
        float3 l = refract(-v, m, interface_ior);

        const float VdN = clamp(dot(N, v), kEpsilon, 1.0f);
        const float VdM = clamp(dot(v, m), 0.0f, 1.0f);
        const float LdM = clamp(dot(l, -m), 0.0f, 1.0f);

        float3 F = fresnel_schlick(f0, f90, (!exiting_volume) ? VdM : LdM);

        const bool sample_transmission = any(diffuse_color * transmission != 0.0) && any(l != 0.0f);
        if (sample_transmission) {                // transmission
            float3 h = -(interface_ior * v + l);  // transmission half-vector;
                                                  // minus is because normal points into into the medium with the lower
                                                  // index of refraction (e.g., air). (convention)
            h = normalize(h);
            float LdN = clamp(dot(-N, l), 0.0f, 1.0f);
            float G = V_SmithGGXCorrelated(VdN, LdN, linear_roughness);
            // auto G = V_Schlick(LdN, VdN, roughness);

            float NdM = clamp(dot(N, m), kEpsilon, 1.0f);
            float VdH = clamp(dot((!exiting_volume) ? v : -v, h), 0.0f, 1.0f);

            float3 brdf = (1.0f - F) * (G * VdH * LdN / NdM);
            brdf = min(brdf, kMaxBRDF);                                       // clamp to avoid fireflies
            float3 new_pos = worldPosition - new_pos_offset_dir * kEpsilon5;  // offset to avoid self-intersection

            float3 next_payload_absorption = old_payload_absorption * diffuse_color * transmission * attenuation * brdf * alpha;

            RayDesc ray;
            ray.Origin = new_pos;
            ray.Direction = normalize(l);
            ray.TMin = 0.0;
            ray.TMax = 10000.0;

            payload.absorption = next_payload_absorption;
            payload.depth = old_depth - 1;

            TraceRay(SceneBVH, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);
        }
        {  // specular reflection
            float3 h = m;

            l = normalize(reflect(-v, h));  // normalizing for better accuracy
            float LdH = clamp(dot(l, h), 0.0f, 1.0f);
            float LdN = clamp(dot(N, l), 0.0f, 1.0f);
            float NdH = clamp(dot(N, h), kEpsilon, 1.0f);

            float G = V_SmithGGXCorrelated(VdN, LdN, linear_roughness);
            // auto G = V_Schlick(LdN, VdN, roughness);
            float3 brdf = F * (G * LdN * LdH / NdH);
            brdf = min(brdf, kMaxBRDF);                                       // clamp to avoid fireflies
            float3 new_pos = worldPosition + new_pos_offset_dir * kEpsilon5;  // offset to avoid self-intersection
            float3 next_payload_absorption = old_payload_absorption * attenuation * brdf * alpha;

            RayDesc ray;
            ray.Origin = new_pos;
            ray.Direction = l;
            ray.TMin = 0.0;
            ray.TMax = 10000.0;

            payload.absorption = next_payload_absorption;
            payload.depth = old_depth - 1;

            TraceRay(SceneBVH, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);
        }
    }
}

[shader("miss")] 
void MissAO(inout HitInfo payload : SV_RayPayload) {
    payload.color += float3(1.0f, 1.0f, 1.0f);
}

[shader("miss")] 
void MissEnvmap(inout HitInfo payload : SV_RayPayload) {
    // compared to Blender, envmap is rotated 180 degrees around Y (Blender's Z) axis, but same as SP
    float y_rotation = g_rayGenCB.envmapRotation;
    float3 dir = WorldRayDirection();
    float2 uv = float2(atan2(-dir.z, -dir.x) + y_rotation, -2.0f * asin(dir.y)) * (1.0f / PI);
    uv = uv * 0.5f + 0.5f;

    payload.color += payload.absorption * EnvMap.SampleLevel(EnvMapSampler, uv, 0).rgb;
}