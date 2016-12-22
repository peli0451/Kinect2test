#include "stdafx.h"
#include "PostProcessQuad.h"

void DirectGL::Geometry::PostProcessQuad::create()
{
	GLfloat vertices[] = { -1.f, -1.f, 1.f, -1.f, -1.f, 1.f, 1.f, -1.f, 1.f, 1.f, -1.f, 1.f };
	m_array.bind();
	m_vertices.BufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)* 12, vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glEnableVertexAttribArray(0);
}

void DirectGL::Geometry::PostProcessQuad::clear()
{
	m_array.clear();
	m_vertices.clear();
}

void DirectGL::Geometry::PostProcessQuad::draw()
{
	if (!m_array.isValid())
		create();
	m_array.bind();
	glDrawArrays(GL_TRIANGLES, 0, 6);
}

void DirectGL::Geometry::PostProcessQuad::draw(unsigned numinstances)
{
	if (!m_array.isValid())
		create();
	m_array.bind();
	glDrawArraysInstanced(GL_TRIANGLES, 0, 6, numinstances);
}