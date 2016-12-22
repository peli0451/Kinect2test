#include "stdafx.h"
#include "PointModel.h"
#include "Utility/GLError.h"

void DirectGL::Geometry::PointModel::clear()
{
	m_vertices.clear();
	glFree();
}

void DirectGL::Geometry::PointModel::glAllocate()
{
	glFree();

	m_array.bind();

#define GEN_BIND_FILL_BUFFER(name, attribIndex, data, dataSize, elemCount) \
	name.BufferData(GL_ARRAY_BUFFER, dataSize, data, GL_STATIC_DRAW); \
	glVertexAttribPointer(attribIndex, elemCount, GL_FLOAT, GL_FALSE, 0, nullptr); \
	glEnableVertexAttribArray(attribIndex);

	if (m_vertices.size())
	{
		GEN_BIND_FILL_BUFFER(m_buffer, 0, m_vertices.data(), sizeof(VertexBuffer::value_type) * m_vertices.size(), 3);
	}
	m_array.unbind();
}

void DirectGL::Geometry::PointModel::glFree()
{
	if (m_array.isValid())
	{
		m_array.clear();
		m_buffer.clear();
	}
}

void DirectGL::Geometry::PointModel::draw()
{
	if (!m_array.isValid())
		glAllocate();
	m_array.bind();
	glPointSize(m_size);
	glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_vertices.size()));
	OPENGL_ERROR_CHECK("Drawing of point model failed with error ");
}

void DirectGL::Geometry::PointModel::draw(unsigned numinstances)
{
	if (!m_array.isValid())
		glAllocate();
	m_array.bind();
	glPointSize(m_size);
	glDrawArraysInstanced(GL_POINTS, 0, static_cast<GLsizei>(m_vertices.size()), numinstances);
	OPENGL_ERROR_CHECK("Drawing of point model failed with error ");
}