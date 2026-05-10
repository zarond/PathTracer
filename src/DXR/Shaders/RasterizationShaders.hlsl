struct PSInput {
    float4 position : SV_POSITION;
    float4 color : COLOR;
};

[shader("vertex")]
//export
PSInput VSMain(float4 position : POSITION, float4 color : COLOR) {
    PSInput result;

    result.position = position;
    result.color = color;

    return result;
}

[shader("pixel")]
//export
float4 PSMain(PSInput input) : SV_TARGET { return input.color; }
