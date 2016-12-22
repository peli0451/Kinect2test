#pragma once

#include <epoxy/gl.h>

#include <DirectGL/Forward.h>
#include <DirectGL/Object.h>

namespace DirectGL
{
	namespace Texturing
	{
		class TextureCubeMap : public DirectGL::Object
		{
		public:
			LOCAL_DECLARE(TextureCubeMap)

			TextureCubeMap(bool mipmapped = true) : m_mipmapped(mipmapped) { clear(); }

			void create();
			void clear();

			GLsizei getWidth() const { return m_width; }
			GLsizei getHeight() const { return m_height; }

			void bind();
			void bind(GLuint unit);
			void TextureParameteri(GLenum paramName, GLuint paramValue);
			void TexImage2D(GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels);
			void TexImage2D(GLuint faceIndex, GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels);
		
			void setMipmapped(bool mipmapped);
			void GenerateMipmaps();

		private:
			GLsizei m_width,
					m_height;
			bool m_mipmapped;
		};

		typedef TextureCubeMap::Ptr TextureCubeMapPtr;
	}
}