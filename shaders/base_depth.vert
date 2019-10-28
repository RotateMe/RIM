#version 450

layout(location = 0) in vec3 Position;

uniform mat4 mvpMatrix;

void main()
{
	vec4 ndcPos = mvpMatrix * vec4(Position, 1.0);
	gl_Position = ndcPos;
}