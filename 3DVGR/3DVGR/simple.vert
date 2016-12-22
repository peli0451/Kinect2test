#version 330

uniform mat4 projection;
uniform mat4 modelT;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 texcoords;
layout(location = 3) in vec3 color;

out vec3 worldPos;
out vec3 out_normal;
out vec2 out_texcoords;
out vec3 out_color;

void main()
{
	worldPos = (modelT * vec4(position, 1.f)).xyz;
	gl_Position = projection * vec4(worldPos, 1.f);
	out_normal = normalize((modelT * vec4(normal, 0.f)).xyz);
	out_texcoords = texcoords;
	out_color = color;
}