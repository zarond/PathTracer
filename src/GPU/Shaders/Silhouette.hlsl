// Depth
Texture2D<float> Depth : register(t0, space0);

// Output
RWTexture2D<float4> Texture : register(u0, space0);

[numthreads(16, 16, 1)] 
void CS_Silhouette(uint3 DTid : SV_DispatchThreadID) 
{
    uint2 imageSize;
    Texture.GetDimensions(imageSize.x, imageSize.y);

    if (DTid.x >= imageSize.x || DTid.y >= imageSize.y) return;

    float depth = Depth.Load(DTid);
    bool is_foreground = (depth != 1.0f);
    Texture[DTid.xy].a = is_foreground;
}