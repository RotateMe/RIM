#version 450

uniform float scale = 1.0;

uniform mat4  mvp;
uniform mat4  mMatrix;
uniform mat4  vMatrix;
uniform mat4  pMatrix;
uniform ivec2 outputSize = ivec2(0,0);

layout( location = 0 ) in vec3 Position;

out vData
{
	flat int Instance;
} Out;


void main()
{	
		vec3 instancePos = Position;
		vec3 worldPos = instancePos;
		vec4 clipPos = mvp * vec4(worldPos, 1.0);
	 
//		gl_PointSize = scale * outputSize.y * (1.0/tan(verticalFOV_rad*0.5)) / clipPos.w; 
		gl_PointSize = scale * outputSize.y * pMatrix[1][1] / clipPos.w; 

		gl_Position  = clipPos;
		Out.Instance = gl_VertexID;
}
