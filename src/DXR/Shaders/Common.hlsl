// Hit information, aka ray payload
// This sample only carries a shading color and hit distance.
// Note that the payload should be kept as small as possible,
// and that its size must be declared in the corresponding
// D3D12_RAYTRACING_SHADER_CONFIG pipeline subobjet.
struct HitInfo
{
  float4 colorAndDistance;
};

// Attributes output by the raytracing when hitting a surface,
// here the barycentric coordinates
//struct Attributes
//{
//  float2 bary;
//};
// or?
//typedef BuiltInTriangleIntersectionAttributes Attributes;

struct RayGenConstantBuffer {
    float4x4 projectionToWorld;
    float4 cameraPosition;
    float2 subpixel_offset;
};
