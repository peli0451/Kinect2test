/** @file GBuffer.h ============================================================
*
*	Defines a variadic template class for texture pools of a certain class
*	with varying internal storage types.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include "Framebuffer.h"

#include <algorithm>
#include <boost/smart_ptr.hpp>

namespace DirectGL
{
	namespace Buffers
	{
		template<typename BufferType, GLenum... Formats> // TODO what if no formats are given?
		class GBuffer
		{
		public:
		// TYPEDEFS ===================================================================
			using texture_type = BufferType;
			using Ptr = boost::shared_ptr<GBuffer<BufferType, Formats...>>;
		
		// CONSTANTS ======================================================================
			constexpr static const unsigned num_buffers = sizeof...(Formats);

		// DE-/CONSTRUCTORS ===========================================================
			GBuffer();
			~GBuffer() { clear(); }

		// OBJECT OVERRIDES ===========================================================
			void create(GLsizei width, GLsizei height);
			void clear();

		// CONVENIENCE WRAPPER METHODS ================================================
			void bind(const GLint range[num_buffers]);
			void attachToFramebuffer(DirectGL::Buffers::Framebuffer &framebuffer);
			void TextureParameteri(GLenum paramName, GLuint paramValue);

		// GETTERS ====================================================================
			texture_type &getBuffer(std::size_t index) { if (index >= num_buffers) LOGEXCEPTION("<GBuffer::getBuffer> Buffer index out of bounds!"); return m_buffers[index]; }
			const texture_type &getBuffer(std::size_t index) const { return m_buffers[i]; }
			GLsizei getWidth() const { return m_buffers[0].getWidth(); }
			GLsizei getHeight() const { return m_buffers[0].getHeight(); }

		// SETTERS ====================================================================
			void setMipmapped(bool enable);

		private:
		// INTERNAL STATE =============================================================
			texture_type m_buffers[num_buffers];
			static GLenum m_formats[num_buffers];
		};
	}
}

template<typename BufferType, GLenum... Formats>
GLenum DirectGL::Buffers::GBuffer<BufferType, Formats...>::m_formats[DirectGL::Buffers::GBuffer<BufferType, Formats...>::num_buffers] = { Formats... };

template<typename BufferType, GLenum... Formats>
inline DirectGL::Buffers::GBuffer<BufferType, Formats...>::GBuffer()
{
}

template<typename BufferType, GLenum... Formats>
inline void DirectGL::Buffers::GBuffer<BufferType, Formats...>::create(GLsizei width, GLsizei height)
{
	clear();
	width = std::max(2, width);
	height = std::max(2, height);
	for (size_t i = 0; i < num_buffers; ++i)
		m_buffers[i].TexImage2D(0, m_formats[i], width, height, 0, GL_RGBA, GL_UNSIGNED_INT, nullptr);
}

template<typename BufferType, GLenum... Formats>
inline void DirectGL::Buffers::GBuffer<BufferType, Formats...>::clear()
{
	for (size_t i = 0; i < num_buffers; ++i)
		m_buffers[i].clear();
}

template<typename BufferType, GLenum... Formats>
inline void DirectGL::Buffers::GBuffer<BufferType, Formats...>::TextureParameteri(GLenum paramName, GLuint paramValue)
{
	for (size_t i = 0; i < num_buffers; ++i)
		m_buffers[i].TextureParameteri(paramName, paramValue);
}

template<typename BufferType, GLenum... Formats>
inline void DirectGL::Buffers::GBuffer<BufferType, Formats...>::bind(const GLint range[num_buffers])
{
	for (size_t i = 0; i < num_buffers; ++i)
		m_buffers[i].bind(range[i]);
}

template<typename BufferType, GLenum... Formats>
inline void DirectGL::Buffers::GBuffer<BufferType, Formats...>::attachToFramebuffer(DirectGL::Buffers::Framebuffer &framebuffer)
{
	for (size_t i = 0; i < num_buffers; ++i)
		framebuffer.attach(GL_COLOR_ATTACHMENT0 + i, m_buffers[i]);
	framebuffer.detachColorBuffers(GL_COLOR_ATTACHMENT0 + num_buffers);
}

template<typename BufferType, GLenum... Formats>
inline void DirectGL::Buffers::GBuffer<BufferType, Formats...>::setMipmapped(bool enable)
{
	for (size_t i = 0; i < num_buffers; ++i)
		m_buffers[i].setMipmapped(enable);
}