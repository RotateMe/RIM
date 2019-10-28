#ifndef CUBEMAP_GLSL
#define CUBEMAP_GLSL

int cubemapFaceIndex(vec3 dir)
{
	vec3 absdir = abs(dir);

	if ( absdir.x > absdir.y && absdir.x > absdir.z )
		return dir.x > 0 ? 0 : 1;
	else if ( absdir.y > absdir.z )
		return dir.y > 0 ? 2 : 3;
	else 
		return dir.z > 0 ? 4 : 5;
}

#endif // CUBEMAP_GLSL