#version 430 core
//#extension GL_ARB_explicit_uniform_location : enable

layout (binding = 0) uniform sampler2D gTex;

layout (location = 0) out vec3 gColorOut; 

in vec2 gUV;
 
void main()
{
	gColorOut = texture(gTex, gUV).rgb; // vec3(0,0,1); texture(gTex, gUV).rgb;
}