#pragma once

#include <epoxy/gl.h>
#include <vector>

#include <DirectGL/Forward.h>
#include "DirectGL/Object.h"
#include "DirectGL/Buffers/Framebuffer.h"

namespace DirectGL
{
	namespace Texturing
	{
		class Texture2D : public DirectGL::Object
		{
		public:
			LOCAL_DECLARE(Texture2D)

			Texture2D(bool mipmapped = true) : m_mipmapped(mipmapped) { clear(); }

			void create();
			void create(GLuint width, GLuint height, const std::vector<GLubyte> &imageData);
			void downloadData();

			void fromFramebuffer(const Buffers::Framebuffer &buffer, GLint x, GLint y, GLint width, GLint height);
			void fromFileStream(std::ifstream &stream);
			void toFileStream(std::ofstream &stream);

			void bind();
			void bind(GLuint unit);
			void TextureParameteri(GLenum paramName, GLuint paramValue);
			void TexImage2D(GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void *pixels);

			inline const GLsizei &getWidth() const { return m_width; }
			inline const GLsizei &getHeight() const { return m_height; }
			inline const std::vector<GLubyte> &getData() { if (m_width * m_height * 3 != m_data.size()) downloadData(); return m_data; }

			void setImage(GLuint width, GLuint height, const std::vector<GLubyte> &imageData);
			void setImage(GLuint width, GLuint height, const GLubyte *const imageData);
			void setMipmapped(bool mipmapped);
			void GenerateMipmaps();

			void clear();

		private:
			typedef std::vector<GLubyte> ByteVector;
			GLsizei m_width,
					m_height;
			ByteVector m_data;
			bool m_mipmapped;

		};
		typedef Texture2D::Ptr Texture2DPtr;
	}
}