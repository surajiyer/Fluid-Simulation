#version 330 core
#extension GL_ARB_explicit_uniform_location : enable

#define specEXP_multiplier 0.01f

layout (location = 0) out vec4 gColorOut; 

in vec4 gColor;
 
void main()
{
	gColorOut = gColor;
}