// Hit information, aka ray payload
// Note that the payload should be kept as small as possible,
// and that its size must be declared in the corresponding
// D3D12_RAYTRACING_SHADER_CONFIG pipeline subobjet.
struct HitInfo
{
    float3 color;
    int depth;
};

struct RayGenConstantBuffer {
    float4x4 projectionToWorld;
    float4 cameraPosition;
    float2 subpixel_offset;
};

typedef BuiltInTriangleIntersectionAttributes Attributes;

struct vertex { // I am using float4 because of the memory alignement on the CPU side, I know it's not ideal
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
