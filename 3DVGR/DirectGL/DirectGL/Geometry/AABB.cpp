#include "stdafx.h"
#include "AABB.h"

DirectGL::Geometry::AABB::AABB(const Position3D &edge1, const Position3D &edge2) : m_minEdge(Position3D(FLT_MAX, FLT_MAX, FLT_MAX)), m_maxEdge(Position3D(-FLT_MAX, -FLT_MAX, -FLT_MAX))
{
	m_minEdge = Position3D(std::min(edge1.x(), edge2.x()), std::min(edge1.y(), edge2.y()), std::min(edge1.z(), edge2.z()));
	m_maxEdge = Position3D(std::max(edge1.x(), edge2.x()), std::max(edge1.y(), edge2.y()), std::max(edge1.z(), edge2.z()));
	updateVolume();
}

void DirectGL::Geometry::AABB::extend(const Position3D &point)
{
	m_minEdge = Position3D(std::min(m_minEdge.x(), point.x()), std::min(m_minEdge.y(), point.y()), std::min(m_minEdge.z(), point.z()));
	m_maxEdge = Position3D(std::max(m_maxEdge.x(), point.x()), std::max(m_maxEdge.y(), point.y()), std::max(m_maxEdge.z(), point.z()));
	updateVolume();
}

void DirectGL::Geometry::AABB::extend(const AABB &other)
{
	extend(other.m_minEdge);
	extend(other.m_maxEdge);
}

void DirectGL::Geometry::AABB::bound(const VertexBuffer &vertices)
{
	m_minEdge = Position3D(FLT_MAX, FLT_MAX, FLT_MAX);
	m_maxEdge = Position3D(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (VertexBuffer::const_iterator it = vertices.begin(), e = vertices.end(); it != e; ++it)
	{
		const VertexBuffer::value_type &curr = *it;
		m_minEdge = Position3D(std::min(m_minEdge.x(), curr.x()), std::min(m_minEdge.y(), curr.y()), std::min(m_minEdge.z(), curr.z()));
		m_maxEdge = Position3D(std::max(m_maxEdge.x(), curr.x()), std::max(m_maxEdge.y(), curr.y()), std::max(m_maxEdge.z(), curr.z()));
	}

	updateVolume();
}

void DirectGL::Geometry::AABB::bound(const Position3D &p1, const Position3D &p2)
{
	m_maxEdge = Position3D(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()), std::min(p1.z(), p2.z()));
	m_minEdge = Position3D(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()), std::max(p1.z(), p2.z()));
}

bool DirectGL::Geometry::AABB::contains(const Position3D &point) const
{
	return (point.x() >= m_minEdge.x() && point.y() >= m_minEdge.y() && point.z() >= m_minEdge.z() &&
			point.x() <= m_maxEdge.x() && point.y() <= m_maxEdge.y() && point.z() <= m_maxEdge.z());
}

void DirectGL::Geometry::AABB::updateVolume()
{
	Position3D diff = m_maxEdge - m_minEdge;
	m_volume = diff.x() * diff.y() * diff.z();
}

