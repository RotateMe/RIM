#version 430 core
 
layout(triangles) in;
layout (triangle_strip, max_vertices=3) out;

in vData
{
	vec3 fragWsPos;
	vec3 fragWsNormal;
	vec2 fragTexCoord;
	vec3 fragColor;
	flat int fragMaterialId;
} In[3];
 
out fData {
	vec3 fragWsPos;
	vec3 fragWsNormal;
	vec3 fragWsTangent;
	vec2 fragTexCoord;
	vec3 fragColor;
	flat int fragMaterialId;
} Out;

uniform bool useFlatNormals = false;

void main()
{
  
  vec3 deltaPos1 = In[1].fragWsPos - In[0].fragWsPos;
  vec3 deltaPos2 = In[2].fragWsPos - In[0].fragWsPos;
	
  vec2 deltaUV1 = In[1].fragTexCoord - In[0].fragTexCoord;
  vec2 deltaUV2 = In[2].fragTexCoord - In[0].fragTexCoord;

  //float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
  float r = deltaUV1.x * deltaUV2.y > deltaUV1.y * deltaUV2.x ? 1.0f : -1.f;

  vec3 tangent   = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y)*r;
  vec3 bitangent = (deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x)*r;

  vec3 flatNormal = normalize( cross(deltaPos1, deltaPos2) );

  if (dot(cross(tangent, bitangent), In[0].fragWsNormal) < 0.0) tangent *= -1.0;

  for(int index = 0; index < gl_in.length(); index++)
  {
     // copy attributes
    gl_Position = gl_in[index].gl_Position;

	Out.fragWsPos      = In[index].fragWsPos;
	Out.fragWsNormal   = useFlatNormals ? flatNormal : In[index].fragWsNormal;
	Out.fragTexCoord   = In[index].fragTexCoord;
	Out.fragWsTangent  = normalize(tangent); 
	Out.fragColor      = In[index].fragColor; 
	Out.fragMaterialId = In[index].fragMaterialId; 
	
    EmitVertex();
  }
  
  EndPrimitive();

}