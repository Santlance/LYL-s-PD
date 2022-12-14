#version 430 core

out vec4 FragColor;

uniform int color;

void main()
{
	if(color == 0)
		FragColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
	else if(color == 1)
		FragColor = vec4(0.0f, 0.0f, 1.0f, 1.0f);
	else if(color == 2)
		FragColor = vec4(0.0f, 1.0f, 0.0f, 1.0f);
}
