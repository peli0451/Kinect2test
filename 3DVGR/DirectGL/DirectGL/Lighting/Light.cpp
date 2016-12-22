#include "stdafx.h"
#include "Light.h"
#include "Utility/Globals.h"

#include <algorithm>
#include <DirectGL/DirectGL.h>

namespace
{
	DirectGL::Utility::Global<int> m_showShadowMap("r_showShadowMap", 0);
	const char *fs_shadow = 
		"#version 420\n																													\
																																		\
		layout(binding = 3) uniform sampler2D transparencyTex;																			\
																																		\
		layout(binding = 0, std140) uniform Material																					\
		{																																\
			bool useAmbientTexture;																										\
			bool useDiffuseTexture;																										\
			bool useSpecularTexture;																									\
			bool useTransparencyTexture;																								\
			bool useBumpTexture;																										\
			bool flipTexture;																											\
			vec3 ambient;																												\
			vec3 diffuse;																												\
			vec3 specular;																												\
			vec3 emissive;																												\
			float shininess;																											\
			float transparency;																											\
		} material;																														\
																																		\
		in vec2 fragment_texcoords;																										\
																																		\
		layout(location = 0) out vec2 finalColor;																						\
																																		\
		void main()																														\
		{																																\
			float tr = (material.useTransparencyTexture ? texture(transparencyTex, fragment_texcoords).r : 1) * material.transparency;	\
			if (tr < 0.000001)																											\
				discard;																												\
			float dx = dFdx(gl_FragCoord.z);																							\
			float dy = dFdy(gl_FragCoord.z);																							\
																																		\
			finalColor = vec2(gl_FragCoord.z, gl_FragCoord.z * gl_FragCoord.z + 0.25 * (dx * dx + dy * dy));							\
		}";																																
																																		
	const char *vs_shadow = 
		"#version 330\n																	\
																						\
		uniform mat4 projection;														\
		uniform mat4 modelT;															\
																						\
		uniform float flipTexture;														\
																						\
		layout(location = 0) in vec3 position;											\
		layout(location = 2) in vec2 texcoords;											\
																						\
		out vec2 fragment_texcoords;													\
																						\
		void main()																		\
		{																				\
			vec3 worldPos = (modelT * vec4(position, 1.f)).xyz;							\
			gl_Position = projection * vec4(worldPos, 1.f);								\
			fragment_texcoords = vec2(texcoords.x, abs(flipTexture - texcoords.y));		\
		}";
}

DirectGL::Geometry::PointModel &DirectGL::Lighting::Light::getSpritePoint()
{
	static DirectGL::Geometry::PointModel s_spritePoint;
	return s_spritePoint;
}

void DirectGL::Lighting::Light::drawIcon()
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLuint reference = std::min(viewport[2] - viewport[0], viewport[3] - viewport[1]);
	getSpritePoint().setSize(reference * 0.05f);
	getSpritePoint().draw();
}

void DirectGL::Lighting::Light::draw()
{
	drawIcon();
}

void DirectGL::Lighting::Light::toFileStream(std::ofstream &stream) const
{
	stream.write(reinterpret_cast<const char*>(&m_ambient), sizeof(Texturing::Color3f));
	stream.write(reinterpret_cast<const char*>(&m_diffuse), sizeof(Texturing::Color3f));
	stream.write(reinterpret_cast<const char*>(&m_specular), sizeof(Texturing::Color3f));
	stream.write(reinterpret_cast<const char*>(&m_intensity), sizeof(GLfloat));
	stream.write(reinterpret_cast<const char*>(&m_cosineCutoff), sizeof(GLfloat));
	stream.write(reinterpret_cast<const char*>(&m_falloffExponent), sizeof(GLfloat));

	stream.write(reinterpret_cast<const char*>(&m_transformation), sizeof(Cameras::Transformation));
}

void DirectGL::Lighting::Light::fromFileStream(std::ifstream &stream)
{
	stream.read(reinterpret_cast<char*>(&m_ambient), sizeof(Texturing::Color3f));
	stream.read(reinterpret_cast<char*>(&m_diffuse), sizeof(Texturing::Color3f));
	stream.read(reinterpret_cast<char*>(&m_specular), sizeof(Texturing::Color3f));
	stream.read(reinterpret_cast<char*>(&m_intensity), sizeof(GLfloat));
	stream.read(reinterpret_cast<char*>(&m_cosineCutoff), sizeof(GLfloat));
	stream.read(reinterpret_cast<char*>(&m_falloffExponent), sizeof(GLfloat));

	stream.read(reinterpret_cast<char*>(&m_transformation), sizeof(Cameras::Transformation));
	m_aabb.bound(Position3D::Ones() * -0.01f, Position3D::Ones() * 0.01f);
	m_type = Point;
}

void DirectGL::Lighting::Light::bindToProgram(Shaders::Program &program) 
{
	// copying all data in the uniform buffer
	auto &block(program.getUniformBlock("Light"));
	if (block.isValid())
	{
		if (!m_uniformBuffer.isValid() || block.getBufferSize() > m_uniformBuffer.getSize())
		{

			std::vector<unsigned char> data(block.getBufferSize());
			memcpy(data.data() + block.getUniform("Light.pos").offset, getTransformation().getPosition().data(), 12);
			memcpy(data.data() + block.getUniform("Light.diffuse").offset, m_diffuse.data(), 12);
			memcpy(data.data() + block.getUniform("Light.specular").offset, m_specular.data(), 12);
			memcpy(data.data() + block.getUniform("Light.intensity").offset, &m_intensity, 4);
			memcpy(data.data() + block.getUniform("Light.falloff").offset, &m_falloffExponent, 4);
			Vector3f c(1.f / (m_cosineCutoff - 1.f), m_cosineCutoff - 1.f == 0.f ? 0.f : 1.f, m_cosineCutoff == -1.f ? 0.f : 1.f);
			memcpy(data.data() + block.getUniform("Light.cutoff").offset, c.data(), 12);
			memcpy(data.data() + block.getUniform("Light.direction").offset, getDirection().data(), 12);
			bool is = m_shadowMap != nullptr;
			memcpy(data.data() + block.getUniform("Light.shadowMapped").offset, &is, sizeof(is));
			memcpy(data.data() + block.getUniform("Light.shadowMapProjection").offset, m_lastShadowCamera.getMatrix().data(), 64);
			memcpy(data.data() + block.getUniform("Light.type").offset, reinterpret_cast<int*>(&m_type), sizeof(m_type));

			m_uniformBuffer.layoutFromUniformBlock(block);
			m_uniformBuffer.BufferData(GL_UNIFORM_BUFFER, block.getBufferSize(), data.data(), GL_DYNAMIC_DRAW);
		}

		m_uniformBuffer.bind(1);
	}
	
	if (isShadowMapped())
		m_shadowMap->bind(10);
}

void DirectGL::Lighting::Light::setDirection(const Direction3D &direction)
{
	Direction3D z(direction.normalized()),
				x(-z.z(), 0.f, z.x()); // orthogonal to z

	// if z is equivalent to a scaled y-unit vector, the length of the above x
	// would be zero -> calculation problems. We avoid this by just assuming it to be
	// the unit x vector.
	if (std::abs(x.norm() - 0.000001f) > FLT_EPSILON)
		x = Direction3D(1.f, 0.f, 0.f);

	x.normalize();
	Direction3D y(x.cross(z));
	Matrix3f rot;
	rot.col(0) = x;
	rot.col(1) = y;
	rot.col(2) = -z; // the identity matrix should arise for the standard direction (0, 0, -1) so we need to invert the z vector

	m_transformation.setRotation(Rotation3f(rot));

	auto dir(getDirection());
	m_uniformBuffer.setUniform("Light.direction", sizeof(dir), dir.data());
}

void DirectGL::Lighting::Light::setShadowMapResolution(GLsizei width, GLsizei height)
{
	if (width == 0 || height == 0)
	{
		m_shadowMap.reset();
		m_shadowMapProgram.clear();
		const bool sm = 0;
		m_uniformBuffer.setUniform("Light.shadowMapped", sizeof(sm), &sm);
	}
	else
	{
		m_shadowMap.reset(new Texturing::Texture2D());
		m_depthBuffer.reset(new Buffers::Renderbuffer());
		m_depthBuffer->RenderbufferStorage(GL_DEPTH_COMPONENT24, width, height);
		m_shadowMap->TexImage2D(0, GL_RG32F, width, height, 0, GL_RG, GL_FLOAT, nullptr);
		m_shadowMap->TextureParameteri(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		m_shadowMap->TextureParameteri(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		//m_shadowMap->TextureParameteri(GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
		//m_shadowMapProgram.fromFiles("shadowmapping.vert", "shadowmapping.frag")
		std::vector<std::string> vertShader, fragShader;
		vertShader.push_back(vs_shadow);
		fragShader.push_back(fs_shadow);
		m_shadowMapProgram.fromSources(vertShader, fragShader); // TODO dedicated shadow mapping shaders?
		const bool sm = 1;
		m_uniformBuffer.setUniform("Light.shadowMapped", sizeof(sm), &sm);
	}
}

void DirectGL::Lighting::Light::renderShadowMap(const RenderableList &scene)
{
	// create the shadow map if it wasn't set up before
	if (!isShadowMapped())
	{
		LOGWARNING("<DirectGL::Lighting::Light::renderShadowMap> No shadowMap has been set, using default...");
		setShadowMapResolution();
	}
	
	// we'll use the owner context of the shadowmap texture.
	assert(m_shadowMap->getOwner().isBound());
	auto fbo(m_shadowMap->getOwner().getGlobalFramebuffer());

	DirectGL::ScopedProgram p(m_shadowMapProgram);
	DirectGL::ScopedEnable t(GL_DEPTH_TEST);
	DirectGL::ScopedViewport v(0, 0, m_shadowMap->getWidth(), m_shadowMap->getHeight());
	{
		DirectGL::ScopedColorMask m(GL_TRUE, GL_TRUE, GL_FALSE, GL_FALSE);
		DirectGL::ScopedFramebuffer f(*fbo);
		fbo->attach(GL_DEPTH_ATTACHMENT, *m_depthBuffer);
		fbo->attach(GL_COLOR_ATTACHMENT0, *m_shadowMap);
		// now setting up the perspective camera for the light
		if (m_type == Point)
		{
			float angle = std::acos(getCutoffCosine()) / static_cast<float>(M_PI)* 180.f * 2; // TODO doesn't work for angles >= 180
			m_lastShadowCamera.setProjective(angle, 0.1f, 1000.f, static_cast<float>(m_shadowMap->getWidth()) / m_shadowMap->getHeight());
		}
		else if (m_type == Directional)
		{
			float angle = std::tan(std::acos(getCutoffCosine()));
			m_lastShadowCamera.setOrthogonal(-angle, angle, angle, -angle, 0.1f, 1000.f);
		}
		else
			LOGEXCEPTION("<Lighting::Light> Invalid type set for this light!");
		m_lastShadowCamera.setPosition(m_transformation.getPosition());
		m_lastShadowCamera.setRotation(m_transformation.getRotation());
		Matrix4f mat(m_lastShadowCamera.getMatrix());
		m_uniformBuffer.setUniform("Light.shadowMapProjection", sizeof(mat), mat.data());

		// and finally the program to render
		m_shadowMapProgram.UniformMatrix4fv("projection", 1, GL_FALSE, m_lastShadowCamera.getMatrix().data());

		glClear(GL_DEPTH_BUFFER_BIT);
		for (auto &renderable : scene)
			renderable->draw(m_shadowMapProgram);

		m_shadowMap->GenerateMipmaps();
	}

	if (m_showShadowMap.get<int>() > 0)
	{
		glClear(GL_DEPTH_BUFFER_BIT);
		for (auto &renderable : scene)
			renderable->draw(m_shadowMapProgram);
	}
}

GLfloat DirectGL::Lighting::Light::getFalloffCorrectedIntensity() const
{
	return m_intensity / ((1- m_cosineCutoff) * static_cast<GLfloat>(M_PI));
}

void DirectGL::Lighting::Light::setFalloff(const Direction3D &dir, GLfloat falloffExponent, GLfloat cutoffCosine)
{
	setFalloffExponent(falloffExponent);
	setCutoffCosine(cutoffCosine);
	setDirection(dir);
}

void DirectGL::Lighting::Light::setCutoffCosine(GLfloat cosine)
{
	m_cosineCutoff = std::max(-1.f, std::min(1.f, cosine)); 
	Vector3f cut(1.f / (m_cosineCutoff - 1.f), m_cosineCutoff - 1.f == 0.f ? 0.f : 1.f, m_cosineCutoff == -1.f ? 0.f : 1.f);
	m_uniformBuffer.setUniform("Light.cutoff", sizeof(cut), cut.data());
}

void DirectGL::Lighting::Light::setTransformation(const Cameras::Transformation &transformation)
{
	Renderable::setTransformation(transformation);
	auto &pos(getTransformation().getPosition());
	auto dir(getDirection());
	m_uniformBuffer.setUniform("Light.pos", sizeof(pos), pos.data());
	m_uniformBuffer.setUniform("Light.direction", sizeof(dir), dir.data());
}

void DirectGL::Lighting::Light::clear()
{
	if (isShadowMapped())
	{
		m_shadowMap->clear();
		m_depthBuffer->clear();
	}
	m_shadowMapProgram.clear();
	m_uniformBuffer.clear();
}
