#include "stdafx.h"
#include "TextureCubeMap.h"
#include "Utility/GLError.h"

void DirectGL::Texturing::TextureCubeMap::create()
{
	clear();
	Object::create();
	glGenTextures(1, &m_id);

	TextureParameteri(GL_TEXTURE_MIN_FILTER, m_mipmapped ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
	TextureParameteri(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	TextureParameteri(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	TextureParameteri(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	TextureParameteri(GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

void DirectGL::Texturing::TextureCubeMap::clear()
{
	if (isValid())
	{
		glDeleteTextures(1, &m_id);
		getOwner().unbindTextureObject(*this);
		m_id = 0;
	}
	Object::clear();
}

void DirectGL::Texturing::TextureCubeMap::bind()
{
	if (!isValid())
		create();
	GLint activeTex;
	glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);
	if (getOwner().bindTextureObject(GL_TEXTURE_CUBE_MAP, activeTex, *this))
		glBindTexture(GL_TEXTURE_CUBE_MAP, m_id);
}


void DirectGL::Texturing::TextureCubeMap::bind(GLuint unit)
{
	if (!isValid())
		create();
	glActiveTexture(GL_TEXTURE0 + unit);
	if (getOwner().bindTextureObject(GL_TEXTURE_CUBE_MAP, GL_TEXTURE0 + unit, *this))
		glBindTexture(GL_TEXTURE_CUBE_MAP, m_id);
}

void DirectGL::Texturing::TextureCubeMap::TextureParameteri(GLenum paramName, GLuint paramValue)
{
	bind();
	glTexParameteri(GL_TEXTURE_CUBE_MAP, paramName, paramValue);
	OPENGL_ERROR_CHECK("glTextureParameteri failed: ");
}

void DirectGL::Texturing::TextureCubeMap::TexImage2D(GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels)
{
	bind();
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, level, internalFormat, width, height, border, format, type, pixels);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, level, internalFormat, width, height, border, format, type, pixels);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, level, internalFormat, width, height, border, format, type, pixels);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, level, internalFormat, width, height, border, format, type, pixels);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, level, internalFormat, width, height, border, format, type, pixels);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, level, internalFormat, width, height, border, format, type, pixels);
	if(m_mipmapped) glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
	OPENGL_ERROR_CHECK("glTexImage2D failed: ");

	m_width = width;
	m_height = height;
}

void DirectGL::Texturing::TextureCubeMap::TexImage2D(GLuint faceIndex, GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels)
{
	bind();
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + faceIndex, level, internalFormat, width, height, border, format, type, pixels);
	if(m_mipmapped) glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
	OPENGL_ERROR_CHECK("glTexImage2D failed: ");
}

void DirectGL::Texturing::TextureCubeMap::setMipmapped(bool mipmapped)
{
	if (mipmapped != m_mipmapped)
	{
		if (isValid())
		{
			if (mipmapped)
			{
				TextureParameteri(GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); // TODO what if the filter has been set to nearest?
				glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
			}
			else
			{
				TextureParameteri(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			}
		}
		m_mipmapped = m_mipmapped;
	}
}

void DirectGL::Texturing::TextureCubeMap::GenerateMipmaps()
{
	bind();
	glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
}