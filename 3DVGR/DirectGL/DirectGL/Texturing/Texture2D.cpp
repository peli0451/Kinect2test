#include "stdafx.h"
#include "Texture2D.h"
#include "Utility/GLError.h"
#include "Buffers/Framebuffer.h"

void DirectGL::Texturing::Texture2D::create()
{
	if (m_id)
		clear();
	Object::create();
	glGenTextures(1, &m_id);
	if (!m_id)
	{
		LOGEXCEPTION("Failed to create Texture! (OpenGL Context not active?)");
	}
	TextureParameteri(GL_TEXTURE_MIN_FILTER, m_mipmapped ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
	TextureParameteri(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	TextureParameteri(GL_TEXTURE_WRAP_S, GL_REPEAT);
	TextureParameteri(GL_TEXTURE_WRAP_T, GL_REPEAT);

	if (m_data.size() && m_width > 0 && m_height > 0)
	{
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		TexImage2D(0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, m_data.data());
	}

	OPENGL_ERROR_CHECK("Generic 2D-Texture creation failed: ");
}

void DirectGL::Texturing::Texture2D::create(GLuint width, GLuint height, const std::vector<GLubyte> &imageData)
{
	setImage(width, height, imageData);
	create();
}

void DirectGL::Texturing::Texture2D::TextureParameteri(GLenum paramName, GLuint paramValue)
{
	bind();
	glTexParameteri(GL_TEXTURE_2D, paramName, paramValue);
	OPENGL_ERROR_CHECK("glTextureParameteri failed: ");
}

void DirectGL::Texturing::Texture2D::TexImage2D(GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels)
{
	bind();
	glTexImage2D(GL_TEXTURE_2D, level, internalFormat, width, height, border, format, type, pixels);
	if (m_mipmapped) glGenerateMipmap(GL_TEXTURE_2D);
	m_width = width;
	m_height = height;
	if (pixels != m_data.data())
	{
		m_data.resize(m_width * m_height * sizeof(GLubyte)* 3);
		std::memcpy(&m_data[0], pixels, sizeof(GLubyte)* 3 * m_width * m_height);
	}
	OPENGL_ERROR_CHECK("glTexImage2D failed: ");
}

void DirectGL::Texturing::Texture2D::setImage(GLuint width, GLuint height, const std::vector<GLubyte> &imageData)
{
	clear();
	m_width = width;
	m_height = height;
	m_data = imageData;
}

void DirectGL::Texturing::Texture2D::setImage(GLuint width, GLuint height, const GLubyte *const imageData)
{
	clear();
	m_width = width;
	m_height = height;
	m_data.resize(width * height * 3);
	std::memcpy(m_data.data(), imageData, m_data.size() * sizeof(GLubyte));
}

void DirectGL::Texturing::Texture2D::clear()
{
	if (m_id)
	{
		glDeleteTextures(1, &m_id);
		getOwner().unbindTextureObject(*this);
	}
	m_width = 0;
	m_height = 0;
	m_id = 0;
	m_data.clear();
	Object::clear();
}

void DirectGL::Texturing::Texture2D::bind()
{
	if (!m_id)
		create();
	GLint activeTex;
	glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);
	if (getOwner().bindTextureObject(GL_TEXTURE_2D, activeTex, *this))
		glBindTexture(GL_TEXTURE_2D, m_id);
}

void DirectGL::Texturing::Texture2D::bind(GLuint unit)
{
	if (!m_id) 
		create();
	glActiveTexture(GL_TEXTURE0 + unit);
	if (getOwner().bindTextureObject(GL_TEXTURE_2D, GL_TEXTURE0 + unit, *this))
		glBindTexture(GL_TEXTURE_2D, m_id);
}

void DirectGL::Texturing::Texture2D::toFileStream(std::ofstream &stream)
{
	stream.write(reinterpret_cast<const char*>(&m_width), sizeof(GLuint));
	stream.write(reinterpret_cast<const char*>(&m_height), sizeof(GLuint));
	ByteVector::size_type size = m_data.size();
	stream.write(reinterpret_cast<const char*>(&size), sizeof(ByteVector::size_type));
	//for (auto i = 0; i < size; ++i)
	//{
	//	stream.write(reinterpret_cast<const char*>(&(m_data[i])), sizeof(GLubyte));
	//}
	stream.write(reinterpret_cast<const char*>(m_data.data()), sizeof(ByteVector::value_type) * size);
}

void DirectGL::Texturing::Texture2D::fromFileStream(std::ifstream &stream)
{
	stream.read(reinterpret_cast<char*>(&m_width), sizeof(GLuint));
	stream.read(reinterpret_cast<char*>(&m_height), sizeof(GLuint));
	ByteVector::size_type size;
	stream.read(reinterpret_cast<char*>(&size), sizeof(ByteVector::size_type));
	m_data.resize(size);
	//for (auto i = 0; i < size; ++i)
	//{
	//	stream.read(reinterpret_cast<char*>(&(m_data[i])), sizeof(GLubyte));
	//}
	stream.read(reinterpret_cast<char*>(m_data.data()), sizeof(ByteVector::value_type) * size);
	//TexImage2D(0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, m_data.data());
}

void DirectGL::Texturing::Texture2D::fromFramebuffer(const Buffers::Framebuffer &buffer, GLint x, GLint y, GLint width, GLint height)
{
	TexImage2D(0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
	Buffers::Framebuffer copyFbo;
	copyFbo.attach(GL_COLOR_ATTACHMENT0, *this);
	buffer.BlitFramebuffer(copyFbo, x, y, x + width, y + height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
	OPENGL_ERROR_CHECK("<Texture2D::fromFramebuffer> Failed to copy the buffer to the texture");
}

void DirectGL::Texturing::Texture2D::downloadData()
{
	bind();
	m_data.resize(m_width * m_height * 3);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, m_data.data());
	OPENGL_ERROR_CHECK("<Texture2D::downloadData> Failed to download texture data");
}

void DirectGL::Texturing::Texture2D::setMipmapped(bool mipmapped)
{
	if (mipmapped != m_mipmapped)
	{
		if (isValid())
		{
			if (mipmapped)
			{
				TextureParameteri(GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); // TODO what if the filter has been set to nearest?
				glGenerateMipmap(GL_TEXTURE_2D);
			}
			else
			{
				TextureParameteri(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			}
		}	
		m_mipmapped = m_mipmapped;
	}
}

void DirectGL::Texturing::Texture2D::GenerateMipmaps()
{
	bind();
	glGenerateMipmap(GL_TEXTURE_2D);
}