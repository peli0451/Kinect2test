#include "stdafx.h"
#include "IndexedLineModel.h"
#include "Utility/GLError.h"

using namespace DirectGL::Geometry;

void IndexedLineModel::operator=(const IndexedLineModel &other)
{
	ModelBase::operator=(other);
	m_lines = other.m_lines;
}

void IndexedLineModel::glFree()
{
	ModelBase::glFree();
	m_glLineBuffer.clear();
}

void IndexedLineModel::clear()
{
	ModelBase::clear();
	m_lines.clear();
}

void IndexedLineModel::glAllocate()
{
	glFree();

	m_glArray.bind();

	// todo normal generation?
	//if (m_normals.empty() && m_vertices.size())
	//	generateNormals();

	ModelBase::glAllocate();

	if (m_lines.size()) // TODO throw error if no triangles present?
	{
		m_glLineBuffer.BufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(LineBuffer::value_type) * m_lines.size(), m_lines.data(), GL_STATIC_DRAW);
	}

	// TODO allocate textures
	m_glArray.unbind();
}

void IndexedLineModel::draw()
{
	if (!isAllocated())
		glAllocate();
	m_glArray.bind();
	glDrawElements(GL_LINES, static_cast<GLsizei>(m_lines.size()) * 2, GL_UNSIGNED_INT, nullptr);
	OPENGL_ERROR_CHECK("Drawing of indexed line model failed with error ");
}