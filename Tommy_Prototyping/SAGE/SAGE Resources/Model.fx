uniform extern float4x4 g_world_view_projection;
uniform extern texture  g_texture;
uniform extern float4   g_ambient_light;

sampler texture_sampler = sampler_state
{
    Texture = <g_texture>;
    
    Filter = MIN_MAG_MIP_LINEAR;
    
	AddressU  = MIRROR;
    AddressV  = MIRROR;
};

struct VertexOutput
{
    float4 homogenous_position    : POSITION0;
    float2 texture_coordinates    : TEXCOORD0;
};

VertexOutput ObjectToProjectionSpace(float3 world_position : POSITION0, float3 normal : NORMAL0, float2 texture_coordinates : TEXCOORD0)
{
    VertexOutput v_out = (VertexOutput)0;
    
    v_out.texture_coordinates = texture_coordinates;
    
    // Transform position from world space to view space.
    v_out.homogenous_position = mul(float4(world_position, 1.0f), g_world_view_projection);
    
    return v_out;
}

float4 ApplyTexture(float2 texture_coordinate : TEXCOORD0, float3 world_position : TEXCOORD2) : COLOR
{
    float4  texture_color = tex2D(texture_sampler, texture_coordinate);
    
    return texture_color;
    //return float4(1.0f, 1.0f, 1.0f, 1.0f);
}

technique ModelRender
{
    pass P0
    {
        vertexShader = compile vs_3_0 ObjectToProjectionSpace();
        pixelShader = compile ps_3_0 ApplyTexture();
        AlphaTestEnable = true;
        AlphaFunc = GreaterEqual;
        AlphaRef = 220;
    }
}