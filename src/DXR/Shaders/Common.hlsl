// Hit information, aka ray payload
// Note that the payload should be kept as small as possible,
// and that its size must be declared in the corresponding
// D3D12_RAYTRACING_SHADER_CONFIG pipeline subobjet.
struct HitInfo {
    float3 color;
    int depth;
    float3 absorption;
    int iteration;
};

struct RayGenConstantBuffer {
    float4x4 projectionToWorld;
    float4 cameraPosition;
    float2 subpixel_offset;
    unsigned int frameID;
    int iteration;
    float invIterationCount;
    int samplesPerPixel;
    float invSamplesPerPixel;
    int maxNewRaysPerBounce;
    float invMaxNewRaysPerBounce;
    int maxRayBounces;
    float envmapRotation;
};

struct Material {
    float4 baseColorFactor;
    float4 emissiveFactor;
    float4 attenuationFactor;
    float metallicFactor;
    float roughnessFactor;
    int baseColorTextureIndex;
    int metallicRoughnessTextureIndex;
    int normalTextureIndex;
    float ior;
    float dielectric_f0;
    float transmisionFactor;
    int transmissionTextureIndex;
    int emissiveTextureIndex;
    float emissiveStrength;
    float alpha_cutoff;

    int doubleSided;
    int hasVolume;
    int alphaBlending;
    int padding0;
};

typedef BuiltInTriangleIntersectionAttributes Attributes;

struct vertex {  // I am using float4 because of the memory alignement on the CPU side, I know it's not ideal
    float4 position;
    float4 normal;
    float4 tangent;
    float4 uv;
};

// Retrieve attribute at a hit position interpolated from vertex attributes using the hit's barycentrics.
template <class ValueType>
ValueType HitAttribute(ValueType vertexAttribute[3], Attributes attr) {
    return 
        (1 - attr.barycentrics.x - attr.barycentrics.y) * vertexAttribute[0] + 
        attr.barycentrics.x * vertexAttribute[1] +
        attr.barycentrics.y * vertexAttribute[2];
}

uint3 pcg3d16(uint3 v) {
    v = v * 12829u + 47989u;

    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;

    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;

    v >>= 16u;

    return v;
}

float3 pcg3d(uint3 v) {
    v = v * 1664525u + 1013904223u;

    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;

    v ^= v >> 16u;

    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;

    // Convert to float using the full 32-bit range
    return float3(v) * (1.0 / 4294967295.0);
}

float4 sample_albedo(const Material material, const float2 uv, const SamplerState Sampler) {
    float4 albedo_color = material.baseColorFactor;
    Texture2D<float4> albedoTex = ResourceDescriptorHeap[material.baseColorTextureIndex];
    albedo_color *= albedoTex.SampleLevel(Sampler, uv, 0);
    return albedo_color;
}
float2 sample_roughness_metallic(const Material material, const float2 uv, const SamplerState Sampler) {
    float2 sample = float2(material.roughnessFactor, material.metallicFactor);
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.metallicRoughnessTextureIndex];
    sample *= Tex.SampleLevel(Sampler, uv, 0).gb;
    return sample;
}
float3 sample_normals(const Material material, const float2 uv, const SamplerState Sampler, out bool has_normal_map) {
    float3 sample = float3(0.5f, 0.5f, 1.0f);
    has_normal_map = false;
    if (material.normalTextureIndex != -1) {
        Texture2D<float4> Tex = ResourceDescriptorHeap[material.normalTextureIndex];
        sample = Tex.SampleLevel(Sampler, uv, 0).rgb;
        has_normal_map = true;
    }
    return sample;
}
float3 sample_emissive(const Material material, const float2 uv, const SamplerState Sampler) {
    float3 sample = material.emissiveFactor.rgb * material.emissiveStrength;
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.emissiveTextureIndex];
    sample *= Tex.SampleLevel(Sampler, uv, 0).rgb;
    return sample;
}
float sample_transmission(const Material material, const float2 uv, const SamplerState Sampler) {
    float sample = material.transmisionFactor;
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.transmissionTextureIndex];
    sample *= Tex.SampleLevel(Sampler, uv, 0).r;
    return sample;
}
