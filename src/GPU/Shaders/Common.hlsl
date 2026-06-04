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

struct RasterConstantBuffer {
    float4x4 projectionToWorld;  // without translation component
    float4x4 viewProjection;
    float4x4 viewMatrix;
    float4x4 Projection;
    float4x4 invProjection;
    float4 cameraPosition;
    float2 subpixel_offset;
    int2 FrameSize;
    float envmap_rotation_sin;
    float envmap_rotation_cos;

    unsigned int frameID;
    int albedoOnlyMode;
    int SpecularLutMips;
    int RenderFrameMips;
    float GTAOStrength;
    float TexturesAOStrength;
    int SSREnabled;
    int specular_aa_enabled;
    float specular_aa_variance;
    float specular_aa_threshold;
};

struct RasterPerDrawData {
    float4x4 modelMatrix;
    float4x4 normalMatrix;
    int meshID;
    float modelScale;
    int UseAOTexture;
    float padding;
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

    int aoTextureIndex;
    float AOStrength;

    int thicknessTextureIndex;
    float thicknessFactor;
    float padding;
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

// Functions for Bicubic interpolation
float4 cubic(float v) {
    float4 n = float4(1.0, 2.0, 3.0, 4.0) - v;
    float4 s = n * n * n;
    float x = s.x;
    float y = s.y - 4.0 * s.x;
    float z = s.z - 4.0 * s.y + 6.0 * s.x;
    float w = 6.0 - x - y - z;
    return float4(x, y, z, w) / 6.0;
}

float4 SampleTextureBicubic(Texture2D tex, SamplerState linearSampler, float2 uv, int2 texSize, float lod) {
    // Convert normalized UV to absolute pixel/texel coordinates
    float2 texelPos = uv * texSize - 0.5f;
    float2 f = frac(texelPos);
    float2 texelIdx = floor(texelPos);

    // Calculate sub-pixel weights for X and Y axes
    float4 wx = cubic(f.x);
    float4 wy = cubic(f.y);

    // Compute the optimized 4-tap offset coordinates
    float4 c = texelIdx.xxyy + float2(-0.5f, 1.5f).xyxy;

    float4 s = float4(wx.xz + wx.yw, wy.xz + wy.yw);
    float4 offset = c + float4(wx.yw, wy.yw) / s;

    offset /= texSize.xxyy;

    // Perform the 4 hardware bilinear samples
    float4 sample0 = tex.SampleLevel(linearSampler, offset.xz, lod);
    float4 sample1 = tex.SampleLevel(linearSampler, offset.yz, lod);
    float4 sample2 = tex.SampleLevel(linearSampler, offset.xw, lod);
    float4 sample3 = tex.SampleLevel(linearSampler, offset.yw, lod);

    float sx = s.x / (s.x + s.y);
    float sy = s.z / (s.z + s.w);

    // Final blending based on calculated weights
    return lerp(lerp(sample3, sample2, sx), lerp(sample1, sample0, sx), sy);
}

float4 SampleTextureBicubicMip(Texture2D tex, SamplerState linearSampler, float2 uv, int2 texSize, float lod) {
    float floor_lod = floor(lod);
    float ceil_lod = ceil(lod);
    int2 floor_texSize = max(1, texSize >> (int)floor_lod);
    int2 ceil_texSize = max(1, texSize >> (int)ceil_lod);
    float4 floor_sample = SampleTextureBicubic(tex, linearSampler, uv, floor_texSize, floor_lod);
    float4 ceil_sample = SampleTextureBicubic(tex, linearSampler, uv, ceil_texSize, ceil_lod);

    return lerp(floor_sample, ceil_sample, frac(lod));
}

// without mips
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
float3 sample_occlusion_roughness_metallic(const Material material, const float2 uv, const SamplerState Sampler) {
    float3 sample = float3(1.0, material.roughnessFactor, material.metallicFactor);
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.metallicRoughnessTextureIndex];
    sample *= Tex.SampleLevel(Sampler, uv, 0).rgb;
    return sample;
}
float sample_occlusion(const Material material, const float2 uv, const SamplerState Sampler) {
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.aoTextureIndex];
    return Tex.SampleLevel(Sampler, uv, 0).r;
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

// with mips
float4 sample_albedo_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    float4 albedo_color = material.baseColorFactor;
    Texture2D<float4> albedoTex = ResourceDescriptorHeap[material.baseColorTextureIndex];
    albedo_color *= albedoTex.Sample(Sampler, uv);
    return albedo_color;
}
float2 sample_roughness_metallic_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    float2 sample = float2(material.roughnessFactor, material.metallicFactor);
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.metallicRoughnessTextureIndex];
    sample *= Tex.Sample(Sampler, uv).gb;
    return sample;
}
float3 sample_occlusion_roughness_metallic_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    float3 sample = float3(1.0, material.roughnessFactor, material.metallicFactor);
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.metallicRoughnessTextureIndex];
    sample *= Tex.Sample(Sampler, uv).rgb;
    return sample;
}
float sample_occlusion_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.aoTextureIndex];
    return Tex.Sample(Sampler, uv).r;
}
float3 sample_normals_filtered(const Material material, const float2 uv, const SamplerState Sampler, out bool has_normal_map) {
    float3 sample = float3(0.5f, 0.5f, 1.0f);
    has_normal_map = false;
    if (material.normalTextureIndex != -1) {
        Texture2D<float4> Tex = ResourceDescriptorHeap[material.normalTextureIndex];
        sample = Tex.Sample(Sampler, uv).rgb;
        has_normal_map = true;
    }
    return sample;
}
float3 sample_emissive_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    float3 sample = material.emissiveFactor.rgb * material.emissiveStrength;
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.emissiveTextureIndex];
    sample *= Tex.Sample(Sampler, uv).rgb;
    return sample;
}
float sample_thickness_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    float sample = material.thicknessFactor;
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.thicknessTextureIndex];
    sample *= Tex.Sample(Sampler, uv).g;
    return sample;
}
float2 sample_transmission_thickness_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    float2 sample = float2(material.transmisionFactor, material.thicknessFactor);
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.transmissionTextureIndex];
    sample *= Tex.Sample(Sampler, uv).rg;
    return sample;
}
float sample_transmission_filtered(const Material material, const float2 uv, const SamplerState Sampler) {
    float sample = material.transmisionFactor;
    Texture2D<float4> Tex = ResourceDescriptorHeap[material.transmissionTextureIndex];
    sample *= Tex.Sample(Sampler, uv).r;
    return sample;
}
