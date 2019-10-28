#version 450

uniform float scale = 1.0;

layout( location = 0 ) in vec3 Position;

out vData
{
	vec3  InstancePos;
	float PointSize;
	flat int Instance;
} Out;

uniform mat4  mvp;
uniform mat4  mMatrix;
uniform mat4  vMatrix;
uniform mat4  pMatrix;
uniform ivec2 outputSize = ivec2(0,0);

void main()
{	

		vec4 clipPos = mvp * vec4(Position, 1.0);
	 
		float pointSize = scale * outputSize.y * pMatrix[1][1] / clipPos.w; 

		gl_PointSize = pointSize; 
		gl_Position  = clipPos;

		Out.InstancePos = Position;
		Out.PointSize = pointSize; 
		Out.Instance = gl_VertexID;

}
