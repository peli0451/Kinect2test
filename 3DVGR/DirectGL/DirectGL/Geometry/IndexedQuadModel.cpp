#include "stdafx.h"
#include "IndexedQuadModel.h"
#include "Utility/GLError.h"

void DirectGL::Geometry::IndexedQuadModel::toFileStream(std::ofstream &stream) const
{

}

void DirectGL::Geometry::IndexedQuadModel::fromFileStream(std::ifstream &stream)
{

}

void DirectGL::Geometry::IndexedQuadModel::draw()
{
	if (!isAllocated())
		glAllocate();
	m_glArray.bind();
	glDrawElements(GL_QUADS, static_cast<GLsizei>(m_quads.size()) * 4, GL_UNSIGNED_INT, nullptr);
	OPENGL_ERROR_CHECK("Drawing of indexed quad model failed with error ");
}

void DirectGL::Geometry::IndexedQuadModel::clear()
{
	ModelBase::clear();
	m_quads.clear();
}

void DirectGL::Geometry::IndexedQuadModel::glAllocate()
{
	glFree();
	m_glArray.bind();
	ModelBase::glAllocate();

	if (m_quads.size()) // TODO throw error if no quads present?
	{
		m_glQuadBuffer.BufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(QuadBuffer::value_type) * m_quads.size(), m_quads.data(), GL_STATIC_DRAW);
	}

	m_glArray.unbind();
}

void DirectGL::Geometry::IndexedQuadModel::glFree()
{
	ModelBase::glFree();
	m_glQuadBuffer.clear();
}

void DirectGL::Geometry::IndexedQuadPatch::draw()
{
	if (!isAllocated())
		glAllocate();
	m_glArray.bind();
	glDrawElements(GL_PATCHES, static_cast<GLsizei>(m_quads.size()) * 4, GL_UNSIGNED_INT, nullptr);
	OPENGL_ERROR_CHECK("Drawing of indexed quad patch failed with error ");
}

void DirectGL::Geometry::IndexedQuadPatch::draw(int numInstances)
{
	if (!isAllocated())
		glAllocate();
	m_glArray.bind();
	glDrawElementsInstanced(GL_PATCHES, static_cast<GLsizei>(m_quads.size()) * 4, GL_UNSIGNED_INT, nullptr, numInstances);
	OPENGL_ERROR_CHECK("Drawing of indexed quad patch failed with error ");
}

void DirectGL::Geometry::IndexedQuadModel::setQuads(const QuadBuffer &quads)
{
	m_quads = quads;
	glFree();
}