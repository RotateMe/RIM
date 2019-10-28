#version 450 core
layout( location = 0 ) in vec3 Position;

in int gl_VertexID;

uniform mat4 mvpMatrix;

out fData
{
	flat int id;
} Out;

void main()
{	
	Out.id = gl_VertexID;
	gl_Position =  mvpMatrix * vec4( Position, 1.0 );
}
