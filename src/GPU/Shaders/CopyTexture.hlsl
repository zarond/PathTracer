struct CopyCSInput {
    uint2 FrameSize;
};

Texture2D<float4> SrcTexture : register(t0);
RWTexture2D<float4> DstTexture : register(u0);

ConstantBuffer<CopyCSInput> g_CB : register(b0);

[numthreads(16, 16, 1)]
void CS_Copy(uint3 DTid : SV_DispatchThreadID)
{
    if (DTid.x >= g_CB.FrameSize.x || DTid.y >= g_CB.FrameSize.y) return;
    uint2 coord = DTid.xy;
    float4 val = SrcTexture.Load(int3(coord, 0));
    DstTexture[coord] = val;
}