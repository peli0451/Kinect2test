#pragma once

#include <DirectGL/Geometry/IndexedTriangleModel.h>

class TestTriangle : public DirectGL::Geometry::IndexedTriangleModel
{
public:
	TestTriangle()
	{
		m_vertices.push_back(DirectGL::Position3D(-1.f, -1.f, 0.f));
		m_vertices.push_back(DirectGL::Position3D(1.f, -1.f, 0.f));
		m_vertices.push_back(DirectGL::Position3D(-1.f, 1.f, 0.f));

		m_triangles.push_back(DirectGL::Index3D(0, 1, 2));
	}
};