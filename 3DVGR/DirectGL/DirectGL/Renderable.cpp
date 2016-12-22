#include "stdafx.h"
#include "Renderable.h"

#include "Shaders/Program.h"

void DirectGL::Renderable::draw(Shaders::Program &program)
{
	program.UniformMatrix4fv("modelT2", 1, GL_FALSE, m_transformation.getMatrix().data());
}

void DirectGL::Renderable::clear()
{
	m_transformation.reset();
	m_aabb.clear();
}

void DirectGL::Renderable::operator=(const Renderable &other)
{
	m_transformation = other.m_transformation;
	m_aabb = other.m_aabb;
}