#include "stdafx.h"
#include "SkyBox2D.h"

void DirectGL::Geometry::Skybox2D::create(float depth)
{
	glFree();
	m_vertices.clear();
	m_triangles.clear();

	m_vertices.push_back(Position3D(-1.f, -1.f, depth));
	m_vertices.push_back(Position3D(1.f, -1.f, depth));
	m_vertices.push_back(Position3D(-1.f, 1.f, depth));
	m_vertices.push_back(Position3D(1.f, 1.f, depth));

	m_triangles.push_back(Index3D(0, 1, 2));
	m_triangles.push_back(Index3D(1, 3, 2));

	glAllocate();
}