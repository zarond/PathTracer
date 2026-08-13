static const float PI = 3.14159265359f;
static const float GOLDEN_RATIO = 1.618034f;
static const float kEpsilon5 = 1e-5f;
static const float kEpsilon = 1e-8f;
static const float kMaxBRDF = 10.0f;

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
float3 importanceSampleGGX(float2 xi, float a) {
    // pdf(h) = D(h) * dot(n, h) : before conversion from half-vector to reflection vector
    // pdf(l) = D(h) * dot(n, h) / (4.0 * dot(l, h)) : after conversion to reflection vector
    // pdf(l) = D(m) * dot(n, m) * dot(l, h) / (eta * dot(v, h) + dot(l, h))^2 : after conversion to refraction vector (l);
    // m - microsurface normal, h - transmission half vector, l - refraction vector, eta - index of refraction from incoming to
    // outgoing medium
    float cos_theta2 = clamp((1.0f - xi.x) / (1.0f + (a * a - 1.0f) * xi.x), 0.0f, 1.0f);
    float cos_theta = sqrt(cos_theta2);
    float sin_theta = sqrt(1.0f - cos_theta2);
    float phi = 2.0f * xi.y * PI;

    float cos_phi = cos(phi);
    float sin_phi = sin(phi);
    return float3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
}
float D_GGX(float NoH, float linear_roughness) {
    float a = NoH * linear_roughness;
    float k = linear_roughness / (1.0f - NoH * NoH + a * a);
    return k * k * (1.0f / PI);
}
float PDF_of_importanceSampleGGX(float NoH, float LoH, float a) { 
    return D_GGX(NoH, a) * NoH / (4.0f * LoH); 
}
float3 fresnel_schlick(float3 f0, float3 f90, float cos_nv) {
    float x = 1.0f - cos_nv;
    float x2 = x * x;
    float x5 = x2 * x2 * x;
    return lerp(f0, f90, x5);
}
float V_SmithGGXCorrelated(float NoV, float NoL, float a) {  // a is alpha_linear_roughness = perceptual_roughness^2
    // Original formulation of G_SmithGGX Correlated
    // lambda_v = (-1 + sqrt ( alphaG2 * (1 - NdotL2 ) / NdotL2 + 1)) * 0.5 f;
    // lambda_l = (-1 + sqrt ( alphaG2 * (1 - NdotV2 ) / NdotV2 + 1)) * 0.5 f;
    // G_SmithGGXCorrelated = 1 / (1 + lambda_v + lambda_l );
    // V_SmithGGXCorrelated = G_SmithGGXCorrelated / (4.0 f * NdotL * NdotV );
    // This is the optimized version
    float a2 = a * a;
    float GGXV = NoL * sqrt((-NoV * a2 + NoV) * NoV + a2);
    float GGXL = NoV * sqrt((-NoL * a2 + NoL) * NoL + a2);
    return 2.0f / (GGXV + GGXL);  // should be 0.5f / (GGXV + GGXL);
}
float G1(float NdW, float k) { return 1.0f / (NdW * (1.0f - k) + k); }
// Schlick - Smith visibility term
// [ http://blog.selfshadow.com/publications/s2013-shading-course/karis/s2013_pbs_epic_notes_v2.pdf ]
float V_Schlick(float NoL, float NoV, float Roughness)  // Roughness is perceptual roughness
{
    float k = max(Roughness * Roughness * 0.5f, kEpsilon5);
    return G1(NoL, k) * G1(NoV, k);  // should be G1*G1 / 4.0;
}
float pow2(float x) { return x * x; }
float pow5(float x) {
    float x2 = x * x;
    return x2 * x2 * x;
}
float f0_dielectric(float transmitted_ior, float incident_ior) {
    return pow2((transmitted_ior - incident_ior) / (transmitted_ior + incident_ior));
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
    return float3x3(t, b, n);  // transposed TBN matrix, tangent is the first row, bitangent - second row, normal - third
}

float3 Tangent2World(float3 v, const float3x3 TBN) {
    return mul(v, TBN);
}  // swap mul(M, v) order because TBN matrix is transposed

float3 get_geometric_normal(const float3 p1, const float3 p2, const float3 p3, 
    const bool double_sided_material = false, const bool backface_hit = false) 
{
    float3 geometric_normal = normalize(cross(p2 - p1, p3 - p1));
    if (double_sided_material && backface_hit) {
        geometric_normal *= -1.0f;
    }
    return geometric_normal;
}
float3 normal_map_sample_to_world(const float3 normal_map_sample, const float3x3 TBN) {
    float3 normal_vector = normal_map_sample * 2.0f - 1.0f;
    return Tangent2World(normal_vector, TBN);
}

float3x3 handle_TBN_creation(const float3x3 NormalMatrixTransposed, const float3 normal_map_color, bool has_normal_map,
    const float3 w_tangent, const float3 w_bitangent, const float3 w_normal,
    const float3 view, const bool double_sided_material, const bool exiting_volume, const bool backface_hit, 
    const float3 p1, const float3 p2, const float3 p3) 
{
    // todo: potential problem with self-intersection from interpolated normal and geometry normal mismatch in certain
    // cases, unrelated to impossible normal angle (in relation to v)
    float3x3 TBN = construct_TBN(w_tangent, w_bitangent, w_normal);
    float3 normal_vector = has_normal_map ? normal_map_sample_to_world(normal_map_color, TBN) : TBN[2];
    const bool impossible_normal_angle = (dot((!exiting_volume) ? view : -view, normal_vector) < 0.0);
    if (impossible_normal_angle) {
        normal_vector = get_geometric_normal(
            p1, p2, p3, double_sided_material, backface_hit);  // todo: somehow incorporate normal map into geometric normal?
        normal_vector = mul(normal_vector, NormalMatrixTransposed);
    }
    if (has_normal_map || impossible_normal_angle) {
        TBN = construct_TBN(TBN[0], TBN[1], normal_vector);  // re-construct TBN with normal from normal map
    }
    return TBN;
}

// [Moving Frostbite to Physically Based Rendering 3.0]
// getSpecularDominantDir page 69
float3 DominantReflectionVector(float3 l, float3 n, float linear_roughness) {
    float factor = saturate(1.0f - linear_roughness);
    factor *= (sqrt(factor) + linear_roughness);
    return lerp(n, l, factor);  // return vector is not unit length
}
// Rasterization Approximation
float transmission_roughness(float linear_roughness, float ior) { // returns linear roughness
    return saturate(linear_roughness * abs(1.0f - 1.0f / ior));
}

// [http://www.jp.square-enix.com/tech/library/pdf/ImprovedGeometricSpecularAA.pdf]
float SpecularAntialiasing(float roughness, float3 n, float SIGMA2, float KAPPA) {
    float3 du = ddx(n);
    float3 dv = ddy(n);
    float variance = SIGMA2 * (dot(du, du) + dot(dv, dv));
    float kernel_roughness2 = min(2.0 * variance, KAPPA);
    float square_roughness = saturate(roughness * roughness + kernel_roughness2);
    return sqrt(square_roughness);
}

float GGX_brdf(float NoH, float LoH, float VoN, float LoN, float a) { 
    float V = V_SmithGGXCorrelated(VoN, LoN, a);
    float D = D_GGX(NoH, a);
    return D * V;
}

// [ Sampling the GGX Distribution of Visible Normals - Eric Heitz]
// Input Ve: view direction
// Input alpha: roughness parameters
// Input U1, U2: uniform random numbers
// Output Ne: normal sampled with PDF D_Ve(Ne) = G1(Ve) * max(0, dot(Ve, Ne)) * D(Ne) / Ve.z
float3 sampleGGXVNDF(float3 Ve, float alpha, float U1, float U2)
{
    // Section 3.2: transforming the view direction to the hemisphere configuration
    float3 Vh = normalize(float3(alpha * Ve.x, alpha * Ve.y, Ve.z));
    // Section 4.1: orthonormal basis (with special case if cross product is zero)
    float lensq = Vh.x * Vh.x + Vh.y * Vh.y;
    float3 T1 = lensq > 0 ? float3(-Vh.y, Vh.x, 0) * (1.0 / sqrt(lensq)) : float3(1, 0, 0);
    float3 T2 = cross(Vh, T1);
    // Section 4.2: parameterization of the projected area
    float r = sqrt(U1);
    float phi = 2.0 * PI * U2;
    float t1 = r * cos(phi);
    float t2 = r * sin(phi);
    float s = 0.5 * (1.0 + Vh.z);
    t2 = (1.0 - s) * sqrt(1.0 - t1 * t1) + s * t2;
    // Section 4.3: reprojection onto hemisphere
    float3 Nh = t1 * T1 + t2 * T2 + sqrt(max(0.0, 1.0 - t1 * t1 - t2 * t2)) * Vh;
    // Section 3.4: transforming the normal back to the ellipsoid configuration
    float3 Ne = normalize(float3(alpha * Nh.x, alpha * Nh.y, max(0.0, Nh.z)));
    return Ne;
}
