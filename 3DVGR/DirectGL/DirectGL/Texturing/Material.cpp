#include "stdafx.h"
#include "Material.h"

#include "Utility/ImageIO.h"
#include "Utility/GLError.h"

#include <algorithm>

void DirectGL::Texturing::Material::fromMTLMaterial(const Utility::MTLFile::Material &material)
{
	m_name = material.name;
	m_ambient = material.ambient;
	m_diffuse = material.diffuse;
	m_specular = material.specular;
	m_emissive = material.emissive;
	m_shininess = std::max(1.f, material.shininess);
	m_refraction = material.refraction;
	m_transparency = std::max(0.f, std::min(1.f, material.transparency));
	m_mirror = false;
	m_flipTexture = false;
	// TODO some textures might be single channel
	if (material.ambientTexture.size())
		m_ambientTex = Utility::ImageIO::loadImage(material.ambientTexture);
	if (material.diffuseTexture.size())
		m_diffuseTex = Utility::ImageIO::loadImage(material.diffuseTexture);
	if (material.specularTexture.size())
		m_specularTex = Utility::ImageIO::loadImage(material.specularTexture);
	if (material.transparencyTexture.size())
		m_transparencyTex = Utility::ImageIO::loadImage(material.transparencyTexture);
	if (material.bumpTexture.size())
		m_bumpTex = Utility::ImageIO::loadImage(material.bumpTexture);
}

void DirectGL::Texturing::Material::bindToProgram(Shaders::Program &program)
{
	auto &block(program.getUniformBlock("Material"));
	if (block.isValid())
	{
		if (!m_uniformBuffer.isValid() || block.getBufferSize() > m_uniformBuffer.getSize())
		{
			m_uniformBuffer.layoutFromUniformBlock(block);
			std::vector<unsigned char> data(block.getBufferSize());

			const bool settings[] = { m_ambientTex != nullptr,
									m_diffuseTex != nullptr,
									m_specularTex != nullptr,
									m_transparencyTex != nullptr,
									m_bumpTex != nullptr,
									m_flipTexture };

			// this part assumes a std140 (i.e. unoptimized) uniform block layout
			memcpy(data.data() + block.getUniform("Material.useAmbientTexture").offset, settings, sizeof(bool));
			memcpy(data.data() + block.getUniform("Material.useDiffuseTexture").offset, settings + 1, sizeof(bool));
			memcpy(data.data() + block.getUniform("Material.useSpecularTexture").offset, settings + 2, sizeof(bool));
			memcpy(data.data() + block.getUniform("Material.useTransparencyTexture").offset, settings + 3, sizeof(bool));
			memcpy(data.data() + block.getUniform("Material.useBumpTexture").offset, settings + 4, sizeof(bool));
			memcpy(data.data() + block.getUniform("Material.flipTexture").offset, settings + 5, sizeof(bool));
			memcpy(data.data() + block.getUniform("Material.ambient").offset, m_ambient.data(), sizeof(m_ambient));
			memcpy(data.data() + block.getUniform("Material.diffuse").offset, m_diffuse.data(), sizeof(m_diffuse));
			memcpy(data.data() + block.getUniform("Material.specular").offset, m_specular.data(), sizeof(m_specular));
			memcpy(data.data() + block.getUniform("Material.emissive").offset, m_emissive.data(), sizeof(m_emissive));
			memcpy(data.data() + block.getUniform("Material.shininess").offset, &m_shininess, sizeof(m_shininess));
			memcpy(data.data() + block.getUniform("Material.transparency").offset, &m_transparency, sizeof(m_transparency));
			m_uniformBuffer.BufferData(GL_UNIFORM_BUFFER, block.getBufferSize(), data.data(), GL_DYNAMIC_DRAW);
		}

		m_uniformBuffer.bind(0);
	}
	
#define BIND_IF_PRESENT(value, slot) if(value != nullptr) value->bind(slot);

	BIND_IF_PRESENT(m_ambientTex, 0);
	BIND_IF_PRESENT(m_diffuseTex, 1);
	BIND_IF_PRESENT(m_specularTex, 2);
	BIND_IF_PRESENT(m_transparencyTex, 3);
	BIND_IF_PRESENT(m_bumpTex, 4);

#undef BIND_IF_PRESENT
	OPENGL_ERROR_CHECK("Failed to bind material: ");
}

void DirectGL::Texturing::Material::fromFileStream(std::ifstream &stream)
{
	stream.read(reinterpret_cast<char*>(&m_ambient), sizeof(m_ambient));
	stream.read(reinterpret_cast<char*>(&m_diffuse), sizeof(m_diffuse));
	stream.read(reinterpret_cast<char*>(&m_specular), sizeof(m_specular));
	stream.read(reinterpret_cast<char*>(&m_emissive), sizeof(m_emissive));
	stream.read(reinterpret_cast<char*>(&m_transparency), sizeof(m_transparency));
	stream.read(reinterpret_cast<char*>(&m_shininess), sizeof(m_shininess));
	stream.read(reinterpret_cast<char*>(&m_refraction), sizeof(m_refraction));
	stream.read(reinterpret_cast<char*>(&m_flipTexture), sizeof(m_flipTexture));
	std::string::size_type namesize;
	stream.read(reinterpret_cast<char*>(&namesize), sizeof(std::string::size_type));
	char *buff = new char[namesize + 1];
	buff[namesize] = 0;
	stream.read(buff, namesize);
	m_name = buff;
	delete[] buff;

#define PTR_READ(name) \
	stream.read(&flag, 1); \
	if (flag != 0) \
	{ \
		name.reset(new Texturing::Texture2D); \
		name->fromFileStream(stream); \
	}
	
	char flag;
	PTR_READ(m_ambientTex);
	PTR_READ(m_diffuseTex);
	PTR_READ(m_specularTex);
	PTR_READ(m_transparencyTex);
	PTR_READ(m_bumpTex);

#undef ptrRead

	if (m_ambientTex.get()) m_ambientTex->setMipmapped(true);
	if (m_diffuseTex.get()) m_diffuseTex->setMipmapped(true);
	if (m_specularTex.get()) m_specularTex->setMipmapped(true);
	if (m_transparencyTex.get()) m_transparencyTex->setMipmapped(true);
	if (m_bumpTex.get()) m_bumpTex->setMipmapped(true);

	m_transparency = std::max(0.f, std::min(1.f, m_transparency));
	m_shininess = std::max(1.f, m_shininess);
}

void DirectGL::Texturing::Material::toFileStream(std::ofstream &stream)
{
	stream.write(reinterpret_cast<const char*>(&m_ambient), sizeof(m_ambient));
	stream.write(reinterpret_cast<const char*>(&m_diffuse), sizeof(m_diffuse));
	stream.write(reinterpret_cast<const char*>(&m_specular), sizeof(m_specular));
	stream.write(reinterpret_cast<const char*>(&m_emissive), sizeof(m_emissive));
	stream.write(reinterpret_cast<const char*>(&m_transparency), sizeof(m_transparency));
	stream.write(reinterpret_cast<const char*>(&m_shininess), sizeof(m_shininess));
	stream.write(reinterpret_cast<const char*>(&m_refraction), sizeof(m_refraction));
	stream.write(reinterpret_cast<const char*>(&m_flipTexture), sizeof(m_flipTexture));
	const char* name = m_name.c_str();
	std::string::size_type namesize = m_name.size();
	stream.write(reinterpret_cast<const char*>(&namesize), sizeof(std::string::size_type));
	stream.write(name, sizeof(char) * namesize);

#define PTR_WRITE(name) \
	if (name != nullptr) \
	{ \
	stream.write("\1", 1); \
	name->toFileStream(stream); \
	} \
	else \
		stream.write("\0", 1);

	PTR_WRITE(m_ambientTex);
	PTR_WRITE(m_diffuseTex);
	PTR_WRITE(m_specularTex);
	PTR_WRITE(m_transparencyTex);
	PTR_WRITE(m_bumpTex);

#undef ptrWrite
}