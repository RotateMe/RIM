
// POINT LIGHTS

float attenuation(PointLight light, float dist) {

	float attenuation = 0.0;
	if (dist <= light.InnerRadius) {
		attenuation = 1.0 / (1 + light.Constant * dist + light.Quadratic * dist * dist);
	}
	else  { // linear fall-off
		attenuation = max(0.0, 1.0 - (dist - light.InnerRadius) / (light.OuterRadius - light.InnerRadius)) * light.LinearFallOff;
	}
	
	//attenuation = pow(attenuation, 2.2); // correct for gamma

	return attenuation;
}

vec3 radiance(PointLight light, vec3 frag_albedo, vec3 frag_normal, vec3 frag_pos) {

	vec3 AmbientColor = light.Color * light.AmbientIntensity;
	
	vec3 lightDir = light.Position - frag_pos;
	float dist = length(lightDir);
	lightDir = normalize(lightDir);
	
	float DiffuseFactor = dot(normalize(frag_normal), lightDir);
	DiffuseFactor = max(DiffuseFactor, 0.0);
	vec3 DiffuseColor = light.Color * light.Intensity * DiffuseFactor;

    return (frag_albedo * (AmbientColor + DiffuseColor)) * attenuation(light, dist);
}


// DIRECTIONAL LIGHTS

vec3 radiance(DirectionalLight light, vec3 frag_albedo, vec3 frag_normal) {
	
	vec3 AmbientColor = light.Color * light.AmbientIntensity;
	
	float DiffuseFactor = dot(normalize(frag_normal), -light.Direction);
	DiffuseFactor = max(DiffuseFactor, 0.0);
    vec3 DiffuseColor = light.Color * light.Intensity * DiffuseFactor;
	
	return frag_albedo * (AmbientColor + DiffuseColor);
}

vec3 radiance(DirectionalLight light, vec3 frag_albedo, vec3 frag_normal, float mapped_shadow) {
	
	vec3 AmbientColor = light.Color * light.AmbientIntensity;
	
	float DiffuseFactor = dot(normalize(frag_normal), -light.Direction) * mapped_shadow;
	DiffuseFactor = max(DiffuseFactor, 0.0);
    vec3 DiffuseColor = light.Color * light.Intensity * DiffuseFactor;
	
	return frag_albedo * (AmbientColor + DiffuseColor);
}