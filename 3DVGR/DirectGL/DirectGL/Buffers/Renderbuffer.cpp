#include "stdafx.h"
#include "Renderbuffer.h"
#include "Utility/GLError.h"

void DirectGL::Buffers::Renderbuffer::create()
{
	clear();
	Object::create();
	glGenRenderbuffers(1, &m_id);
	OPENGL_ERROR_CHECK("Renderbuffer creation failed: ");
}

void DirectGL::Buffers::Renderbuffer::clear()
{
	if (m_id)
	{
		glDeleteRenderbuffers(1, &m_id);
		m_id = 0;
		OPENGL_ERROR_CHECK("Renderbuffer deletion failed: ");
		Object::clear();
	}
}

void DirectGL::Buffers::Renderbuffer::RenderbufferStorage(GLenum internalformat, GLsizei width, GLsizei height)
{
	if (!m_id)
		create();
	bind();
	glRenderbufferStorage(GL_RENDERBUFFER, internalformat, width, height);
	m_width = width;
	m_height = height;
	OPENGL_ERROR_CHECK("Setting renderbuffer storage spec failed: ");
}

void DirectGL::Buffers::Renderbuffer::bind() const
{
	if (getOwner().bindObject(GL_RENDERBUFFER, *this))
		glBindRenderbuffer(GL_RENDERBUFFER, m_id);
}