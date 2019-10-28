#include "math.glsl"

///////////////////
///////// CONVERSION
////////////
vec3 ScreenCoordsToViewVector(in vec2 screenCoords, in float verticalFOV, in float aspectRatio, in  bool useCamera = false, in mat4 viewTransform = mat4(1.0)) 
{
	// eye vector to spherical coordinates

	float v_half_fov = verticalFOV / 2.0; // half field of view vertical
	float h_half_fov =  atan(aspectRatio * tan(v_half_fov)); // half field of view horizontal
				
	float x = (screenCoords.x - 0.5) * 2.0; // [-1.0, 1.0]
	float y = (screenCoords.y - 0.5) * 2.0; // [-1.0, 1.0]

	float hangle = atan(x * tan(h_half_fov));
	float vangle = atan(y * cos(hangle) * tan(v_half_fov)); // (cos(hangle) corrects for deformation to the sides

	vec3 rotDir = vec3(0., 0., 1.0);
	rotDir = rotate_vertex_position(rotDir, vec3(vec4(1.0, 0.0, 0.0, 0.0)), vangle * 180.0 / M_PI);		// vertically rotate about x
	rotDir = rotate_vertex_position(rotDir, vec3(vec4(0.0, -1.0, 0.0, 0.0)), hangle * 180.0 / M_PI);		// horizontally rotate about y
	
	if (useCamera) rotDir = rotDir * mat3(viewTransform);												// rotate with camera rotation
	
	return rotDir;
}

vec2 ScreenCoordsToSphereCoords(in vec2 screenCoords, in float verticalFOV, in float aspectRatio) {

	// eye vector to spherical coordinates

	float v_half_fov = verticalFOV / 2.0; // half field of view vertical
	float h_half_fov =  atan(aspectRatio * tan(v_half_fov)); // half field of view horizontal
				
	float x = (screenCoords.x - 0.5) * 2.0; // [-1.0, 1.0]
	float y = (screenCoords.y - 0.5) * 2.0; // [-1.0, 1.0]

	// atan only working in 4th quadratnt
	// atan2 seperates 2,3th quadrant and 1 

	float hangle = atan(x * tan(h_half_fov));
	float vangle = atan(y * cos(hangle) * tan(v_half_fov)); // (cos(hangle) corrects for deformation to the sides
	
	
	return vec2(hangle, vangle);
}

///////////////////
///////// CYLINDRICAL
////////////
vec3 mapCylindricalTexCoordToView(vec2 normTexCoord) {

	float v = M_PI/2;

	float i = (normTexCoord.x - 0.5)*2;
	float j = (normTexCoord.y - 0.5)*2;
	float x = cos(i * M_PI);
	float y = j * tan(v/2);
	float z = sin(i * M_PI);

	vec3 envDir = vec3(x, y, z );

	return envDir;
}

vec4 CylindricalMapLookUp(sampler2D tex, vec3 r)
{
	r = rotate_vertex_position(r, vec3(0,1,0), 180); // correct for 
	float x = r.x;
	float y = r.y;
	float z = r.z;
	float phi_norm = atan(x,z) / (2*M_PI) + 0.5; // [0, 2*M_PI] mapped to [0,1]
	float theta = acos(y) / M_PI; //[0, M_PI] mapped to [0,1]
	vec2 texcoord = vec2(1.0 - phi_norm, theta);
    return texture( tex, texcoord );
}


///////////////////
///////// LAT-LONG / EQUIRECT
////////////
vec3 mapEquirectTexCoordToView(vec2 normTexCoord) {

	float u = normTexCoord.x * 2;
	float v = normTexCoord.y;
	float theta = M_PI * (u - 1.0) ; // [-M_PI, M_PI]	
	float phi = M_PI * v ; // [0, 2 M_PI]
	float x = sin(phi) * sin(theta);
	float y = cos(phi);
	float z = -sin(phi) * cos(theta);

	vec3 envDir = vec3(x, y, z );

	return -envDir;

}

vec2 mapViewToEquirectTexCoord(vec3 dir)
{
	dir = -dir;
	float yaw = .5 - atan(dir.z, -dir.x) / (2.0 * M_PI); // lon
	float pitch = .5 - asin(dir.y) / M_PI;				// lat
	return vec2(yaw, pitch);
}

vec4 EquiRectMapLookUpLod(sampler2D tex, vec3 r, float lod)
{
	vec2 texcoord = mapViewToEquirectTexCoord(r);
    return texture( tex, texcoord, lod );
}

vec4 EquiRectMapLookUp(sampler2D tex, vec3 r)
{
	return EquiRectMapLookUpLod(tex, r, 0);
}

///////////////////
///////// MIRROR SPHERE
////////////
vec3 mapMirrorSphereTexCoordToView(vec2 normTexCoord) {

	// mapping as described in book "High Dynamic Range Image Acquisition, by Sheldon M Ross"
	float u = 1.0-normTexCoord.y;
	float v = 1.0-normTexCoord.x;

	float u_off = 2*u-1;
	float v_off = 2*v-1;
	float r = sqrt(u_off*u_off + v_off*v_off);
	
	if (r > 1.0)
		return vec3(0);

	float theta = atan(u_off,-v_off);
	float phi = 2 * asin(r);
	
	float x = sin(phi) * cos(theta);
	float y = sin(phi) * sin(theta);
	float z = -cos(phi);

	vec3 envDir = vec3(x, y, z );

	return envDir;

}

///////////////////
///////// ANGULAR MAP
////////////
vec3 mapAngularTexCoordToView(vec2 normTexCoord) {

	// mapping as described in book "High Dynamic Range Image Acquisition, by Sheldon M Ross"
	float u = normTexCoord.x;
	float v = normTexCoord.y;

	// compute radius to black out non-valid pixels
	float u_off = 2*u-1;
	float v_off = 2*v-1;
	float r = sqrt(u_off*u_off + v_off*v_off);
	if (r > 1.0)
		return vec3(0);


	float theta = atan(-2*v + 1, 2*u-1);
	float phi = M_PI * sqrt((2*u-1)*(2*u-1) + (2*v-1)*(2*v-1));
	
	float x =  sin(phi) * cos(theta);
	float y =  sin(phi) * sin(theta);
	float z = -cos(phi);

	vec3 envDir = vec3(x, y, z );

	return envDir;

}

///////////////////
///////// CUBE MAP FORMAT
////////////

vec3 mapCubeTexCoordToView(vec2 normTexCoord, int cubeface) {

	vec3 envDir = vec3(0);

	switch (cubeface) {
		case 0: //+X
			envDir = vec3(1, mix(1,-1,normTexCoord.y), mix(1,-1,normTexCoord.x));
			break;
		case 1: // -X
			envDir = vec3(-1, mix(1,-1,normTexCoord.y), mix(-1,1,normTexCoord.x));
			break;
		case 2: // +Y
			envDir = vec3(mix(-1,1,normTexCoord.x), 1, mix(-1,1,normTexCoord.y));
			//upVector = vec3(0,0,1);
			break;
		case 3: // -Y
			envDir = vec3(mix(-1,1,normTexCoord.x), -1, mix(1,-1,normTexCoord.y));
			//upVector = vec3(0,0,-1);
			break;
		case 4: // +Z
			envDir = vec3(mix(-1,1,normTexCoord.x), mix(1,-1,normTexCoord.y), 1);
			break;
		case 5: // -Z
			envDir = vec3(mix(1,-1,normTexCoord.x), mix(1,-1,normTexCoord.y), -1);
			break;
	}
	envDir = normalize(envDir);	
	
	return envDir;

}

// This is for indirect coordinate system cubemap sampling = Direct3D
//  ________ ________ ________ ________ ________ ________
// |     y  |  y     |   ___x |  z     |  y     |     y  |
// |     |  |  |     |  |     |  |     |  |     |     |  |
// | z___|  |  |___z |  |     |  |___x |  |___x | x___|  |
// |        |        |  z     |        |        |        |
// |________|________|________|________|________|________|
//   face X  face -X   face Y  face -Y   face Z   face -Z
//
vec3 mapCubeTexCoordToViewNoFlip(vec2 inputTexcoord, int viewIndex)
{
    vec2 position = 2 * inputTexcoord - 1;

    if (viewIndex == 0)
        return vec3(1, -position.y, -position.x); // face X
    if (viewIndex == 1)
        return vec3(-1, -position.y, position.x); // face -X
    if (viewIndex == 2)
        return vec3(position.x, 1, position.y); // face Y
    if (viewIndex == 3)
        return vec3(position.x, -1, -position.y); // face -Y
    if (viewIndex == 4)
        return vec3(position.x, -position.y, 1); // face Z
    if (viewIndex == 5)
        return vec3(-position.x, -position.y, -1); // face -Z
        
    return vec3(0);
}

vec4 CubeMapLookUpLod(samplerCube tex, vec3 dir, float lod) {
	return textureLod( tex, dir, lod);
}

vec4 CubeMapLookUp(samplerCube tex, vec3 dir) {
	return CubeMapLookUpLod( tex, dir, 0.0);
}

// From https://www.nvidia.com/object/cube_map_ogl_tutorial.html

void mapViewToCubeTexCoord(in vec3 view, out vec2 normTexCoord, out int cubeface) {

	vec3 absView = abs(view);

	if (absView.x > absView.y && absView.x > absView.z) {
		cubeface = view.x > 0 ? 0 : 1;
	} else if (absView.y > absView.z) {
		cubeface = view.y > 0 ? 2 : 3;
	} else {
		cubeface = view.z > 0 ? 4 : 5;
	}

	vec2  st;
	float ma;

	switch (cubeface) {
	case 0: //+X
		st = vec2(-view.z, -view.y);
		ma = absView.x;
		break;
	case 1: // -X
		st = vec2(+view.z, -view.y);
		ma = absView.x;
		break;
	case 2: // +Y
		st = vec2(+view.x, +view.z);
		ma = absView.y;
		break;
	case 3: // -Y
		st = vec2(+view.x, -view.z);
		ma = absView.y;
		break;
	case 4: // +Z
		st = vec2(+view.x, -view.y);
		ma = absView.z;
		break;
	case 5: // -Z
		st = vec2(-view.x, -view.y);
		ma = absView.z;
		break;
	}
	
	
	normTexCoord = (st / ma + 1) * 0.5;

}
