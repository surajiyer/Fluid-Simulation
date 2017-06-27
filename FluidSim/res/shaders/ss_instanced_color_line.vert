#version 330 core
#extension GL_ARB_explicit_uniform_location : enable

// vertex input
layout (location = 0) in vec2 in_position_ss;
layout (location = 1) in vec4 in_color;

// global vars
out vec4 gColor;

void main(){
	gColor = in_color;
	gl_Position = vec4((in_position_ss - 0.5) * 2, 0, 1);
}
