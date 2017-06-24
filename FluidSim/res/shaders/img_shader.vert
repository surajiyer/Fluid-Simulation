#version 430 core
//#extension GL_ARB_explicit_uniform_location : enable

layout (location = 0) in vec2 in_fsq;

out vec2 gUV;

void main(){
	gl_Position = vec4(in_fsq, 0.0, 1);
	gUV = (in_fsq + 1) * 0.5;
}