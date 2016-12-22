#include "stdafx.h"
#include "VertexArray.h"
#include "Utility/GLError.h"

void DirectGL::Buffers::VertexArray::create()
{
	clear();
	Object::create();
	glGenVertexArrays(1, &m_id);
	OPENGL_ERROR_CHECK("Failed to create Vertex Array: ");
}

void DirectGL::Buffers::VertexArray::clear()
{
	if (m_id)
	{
		glDeleteVertexArrays(1, &m_id);
		m_id = 0;
		OPENGL_ERROR_CHECK("Vertex Array deletion failed: ");
	}
	Object::clear();
}

void DirectGL::Buffers::VertexArray::bind()
{
	if (!isValid())
		create();
	if (getOwner().bindObject(GL_VERTEX_ARRAY_BINDING, *this))
		glBindVertexArray(m_id);
}

void DirectGL::Buffers::VertexArray::unbind()
{
	if (Context::getCurrent()->unbindPoint(GL_VERTEX_ARRAY_BINDING))
		glBindVertexArray(0);
}