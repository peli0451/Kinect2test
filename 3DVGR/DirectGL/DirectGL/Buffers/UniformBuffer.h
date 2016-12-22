/** @file UniformBuffer.h =====================================================
*
*	Declares UniformBuffer class for handling buffer objects bound to
*	GL_UNIFORM_BUFFER and interoperating with shader uniform blocks.
*
*	@author Julian Meder
* =============================================================================
*/


#pragma once

#include "Buffer.h"
#include <DirectGL/Types.h>
#include <DirectGL/Shaders/UniformBlock.h>

#include <map>

namespace DirectGL
{
	namespace Buffers
	{

		class UniformBuffer : public Buffer
		{
		public:
		// CONSTRUCTORS ===============================================================
			UniformBuffer() : m_blockSize(0) {}
			UniformBuffer(const Shaders::UniformBlock &block);

		// OBJECT OVERRIDES ===========================================================
			void create() override;

		// MEMBER FUNCTIONS ===========================================================
			void layoutFromUniformBlock(const Shaders::UniformBlock &block);
			void setUniform(const std::string &name, GLsizeiptr size, const void *data);

		// OPENGL WRAPPER METHODS =====================================================
			void bind() { Buffer::bind(GL_UNIFORM_BUFFER); }
			void bind(GLuint index);
			void BufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage);
			void BufferSubData(GLintptr offset, GLsizeiptr size, const void* data);

		private:
		// INTERNAL STATE =============================================================
			Shaders::UniformBlock::UniformMap m_uniforms;
			GLuint m_blockSize;
		};
	}
}