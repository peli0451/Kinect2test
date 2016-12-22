#include "stdafx.h"
#include "UniformBuffer.h"
#include <Utility/GLError.h>

using namespace DirectGL::Buffers;
using namespace DirectGL::Shaders;

UniformBuffer::UniformBuffer(const UniformBlock &block)
{
	layoutFromUniformBlock(block);
}

void UniformBuffer::layoutFromUniformBlock(const UniformBlock &block)
{
	clear();
	m_uniforms = block.getUniforms();
	m_blockSize = block.getBufferSize();
}

void UniformBuffer::BufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage)
{
	if (size != m_blockSize)
		LOGEXCEPTION("<Buffers::UniformBuffer::BufferData> Tried to buffer data with different size than defined uniform block size!");
	
	Buffer::BufferData(target, size, data, usage);
}

void UniformBuffer::BufferSubData(GLintptr offset, GLsizeiptr size, const void* data)
{
	bind();
	glBufferSubData(GL_UNIFORM_BUFFER, offset, size, data);
	OPENGL_ERROR_CHECK("<Buffers::UniformBuffer::BufferSubData> failed: ");
}

void UniformBuffer::bind(GLuint index)
{
	// TODO add index range bounding function in context class and check binding by index
	if (!isValid()) create();
	getOwner().bindObject(GL_UNIFORM_BUFFER, *this);
	glBindBufferRange(GL_UNIFORM_BUFFER, index, m_id, 0, m_size);
	OPENGL_ERROR_CHECK("<Buffers::UniformBuffer::bind(GLuint)> failed: ");
}

void UniformBuffer::create()
{
	if (!isValid()) Buffer::create();
	Buffer::BufferData(GL_UNIFORM_BUFFER, m_blockSize, nullptr, GL_DYNAMIC_DRAW);
}

void UniformBuffer::setUniform(const std::string &name, GLsizeiptr size, const void *data)
{
	if (data == nullptr)
		LOGEXCEPTION("<Buffers::UniformBuffer::setUniform> Invalid data given (nullptr)!");

	auto f(m_uniforms.find(name));
	if (f == m_uniforms.end())
		LOGWARNING("<Buffers::UniformBuffer::setUniform> No uniform \"" + name + "\" registered!");
	else
	{
		BufferSubData(f->second.offset, size, data);
	}
}