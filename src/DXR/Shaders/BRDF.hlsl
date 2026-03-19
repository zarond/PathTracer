static const float PI = 3.14159265359f;
static const float GOLDEN_RATIO = 1.618034f;

float3 ImportanceSampleCosDir(float2 xi) {
    // pdf(x) = cos(l, n) / pi
    float cos_theta2 = 1.0f - xi.x;
    float cos_theta = sqrt(cos_theta2);
    float sin_theta = sqrt(xi.x);
    float phi = 2.0f * xi.y * PI;

    float cos_phi = cos(phi);
    float sin_phi = sin(phi);

    return float3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
}

float fibonacci1D(int i) { return fmod((float(i) + 1.0f) * GOLDEN_RATIO, 1.0f); }
float fibonacci1D(float i) { return fmod((i + 1.0f) * GOLDEN_RATIO, 1.0f); }
float2 fibonacci2D(int i, float inv_nbSamples) {
    float i_f = float(i);
    return float2((i_f + 0.5f) * inv_nbSamples, fibonacci1D(i_f));
}

float3x3 construct_TBN(const float3 tangent, const float3 bitangent, const float3 normal) {
    float3 n = normalize(normal);
    float3 t = normalize(tangent - n * dot(n, tangent));
    float3 b = normalize(bitangent - n * dot(n, bitangent) - t * dot(t, bitangent));
    return float3x3(t, b, n);
}

float3 Tangent2World(float3 v, const float3x3 TBN) { return mul(TBN, v); }