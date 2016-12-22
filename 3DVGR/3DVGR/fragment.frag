#version 420

layout(binding = 1, std140) uniform Light
{
	vec3 pos;
	vec3 diffuse;
	vec3 specular;
	float intensity;
	float falloff;
	vec3 cutoff;
	vec3 direction;
	bool shadowMapped;
	mat4 shadowMapProjection;
} light;

layout(binding = 0, std140) uniform Material
{
	bool useAmbientTexture;
	bool useDiffuseTexture;
	bool useSpecularTexture;
	bool useTransparencyTexture;
	bool useBumpTexture;
	bool flipTexture;
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	vec3 emissive;
	float shininess;
	float transparency;
} material;

layout(binding = 1) uniform sampler2D diffuseTex;

uniform vec3 camPos;

in vec3 worldPos;
in vec3 out_normal;
in vec2 out_texcoords;

layout(location = 0)out vec4 finalColor;


void main()
{
	// calculate lighting
	//finalColor = vec4(vec3(0.f), 1.f);
	//
	//vec3 v = camPos - worldPos;
	//camSpaceDist = length(v);
	//v = normalize(v);

	//for (int i = 0; i < 8; ++i)
	//{
	//	vec3 diff = lightPos[i] - worldPos;
	//	vec3 lightDir = normalize(diff);
	//	finalColor += vec4(diffuse * vec3(max(0, dot(lightDir, out_normal))) * lightDiff[i] * lightInt[i] * (useDiffuse > 0 ? texture(diffuseTex, out_texcoords).rgb : vec3(1.f)), 0.f);
	//
	//	vec3 r = reflect(-lightDir, out_normal);
	//	finalColor += vec4(specular * lightSpec[i] * pow(max(0, dot(r, v)), shininess) * lightInt[i], 0.f);
	//}
	//if (noLight > 0)
	//	finalColor = vec4(1.f);

	vec3 lightDir = normalize(camPos - worldPos);
	vec3 r = reflect(-lightDir, out_normal);

	finalColor = vec4(material.diffuse, 1.f) * max(0, dot(lightDir, out_normal)) * (material.useDiffuseTexture ? texture(diffuseTex, out_texcoords).rgba : vec4(1.f));
	finalColor += vec4(material.specular * pow(max(0, dot(r, lightDir)), material.shininess), 0.f);


	//finalPos = worldPos;
	//finalNormal = out_normal;
}