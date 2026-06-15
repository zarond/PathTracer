// If I was worried about performance I could also use
// faster approximations for srgb conversion from here:
//[https://chilliant.blogspot.com/2012/08/srgb-approximations-for-hlsl.html]

float srgb_to_linear(float x) { return x <= 0.04045 ? x * 0.0773993808 : pow((x + 0.055) / 1.055, 2.4); }

float3 srgb_to_linear(float3 c) {
    return float3(srgb_to_linear(c.r), srgb_to_linear(c.g), srgb_to_linear(c.b));
}

float4 srgb_to_linear(float4 c) {
    return float4(srgb_to_linear(c.r), srgb_to_linear(c.g), srgb_to_linear(c.b), c.a);
}

float linear_to_srgb(float x) { return x <= 0.0031308 ? 12.92 * x : 1.055 * pow(x, 0.416666667) - 0.055; }

float3 linear_to_srgb(float3 c) {
    return float3(linear_to_srgb(c.r), linear_to_srgb(c.g), linear_to_srgb(c.b));
}

float4 linear_to_srgb(float4 c) {
    return float4(linear_to_srgb(c.r), linear_to_srgb(c.g), linear_to_srgb(c.b), c.a);
}