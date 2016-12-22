#include "stdafx.h"
#include "Buffer.h"
#include "Utility/GLError.h"

void DirectGL::Buffers::Buffer::create()
{
	clear();
	Object::create();
	glGenBuffers(1, &m_id);
	OPENGL_ERROR_CHECK("Failed to create Buffer object: ");
}

void DirectGL::Buffers::Buffer::clear()
{
	m_size = 0;
	if (isValid())
	{
		glDeleteBuffers(1, &m_id);
		m_id = 0;
		OPENGL_ERROR_CHECK("Buffer deletion failed: ");
		Object::clear();
	}
}

void DirectGL::Buffers::Buffer::bind(GLenum target)
{
	if (!isValid())
		create();
	if (getOwner().bindObject(target, *this))
		glBindBuffer(target, m_id);
}

void DirectGL::Buffers::Buffer::BufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage)
{
	bind(target);
	glBufferData(target, size, data, usage);
	auto err = glGetError();
	if (err != GL_NO_ERROR)
	{
		OPENGL_LOG_ERR(err, "Error during data buffering: ");
	}
	else
		m_size = size;

}