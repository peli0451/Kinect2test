#include "stdafx.h"
#include "IndexedTriangleModel.h"
#include <DirectGL/Utility/GLError.h>

#include <algorithm>

void DirectGL::Geometry::IndexedTriangleModel::operator=(const IndexedTriangleModel &other)
{
	ModelBase::operator=(other);
	m_triangles = other.m_triangles;
}

void DirectGL::Geometry::IndexedTriangleModel::glAllocate()
{
	glFree();

	m_glArray.bind();

	if (m_normals.empty() && m_vertices.size())
		generateNormals();

	ModelBase::glAllocate();

	if (m_triangles.size()) // TODO throw error if no triangles present?
	{
		m_glTriangleBuffer.BufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(TriangleBuffer::value_type) * m_triangles.size(), m_triangles.data(), GL_STATIC_DRAW);
	}

	// TODO allocate textures
	m_glArray.unbind();
}

void DirectGL::Geometry::IndexedTriangleModel::glFree()
{
	ModelBase::glFree();
	m_glTriangleBuffer.clear();
}

void DirectGL::Geometry::IndexedTriangleModel::draw()
{
	if (!isAllocated())
		glAllocate();
	m_glArray.bind();
	glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_triangles.size()) * 3, GL_UNSIGNED_INT, nullptr);
	OPENGL_ERROR_CHECK("Drawing of indexed triangle model failed with error ");
}

void DirectGL::Geometry::IndexedTriangleModel::generateNormals()
{
	m_normals.clear();
	m_normals.resize(m_vertices.size());
	//for (size_t i = 0; i < m_triangles.size(); ++i)
	//{
	//	Index3D &currIndex(m_triangles[i]);
	//	Position3D &v1(m_vertices[currIndex.x()]),
	//		&v2(m_vertices[currIndex.y()]),
	//		&v3(m_vertices[currIndex.z()]);
	//
	//	Position3D normal = (v2 - v1).cross(v3 - v1);
	//	float norm = normal.norm();
	//	if (norm)
	//		normal /= norm;
	//	m_normals[currIndex.x()] = normal;
	//	m_normals[currIndex.y()] = normal;
	//	m_normals[currIndex.z()] = normal;
	//	
	//}
	std::vector<std::pair<Position3D, float>> normalAccum(m_vertices.size(), std::pair<Position3D, float>(Position3D::Zero(), 0.f));
	for (TriangleBuffer::iterator it = m_triangles.begin(), e = m_triangles.end(); it != e; ++it)
	{
		Index3D &curr = *it;
		Position3D &v1(m_vertices[curr.x()]),
				   &v2(m_vertices[curr.y()]),
				   &v3(m_vertices[curr.z()]);
		Position3D normal = (v2 - v1).cross(v3 - v1);
		float norm = normal.norm();
		float area = 0.5f * norm;
		if (norm)
			normal /= norm;
		for (unsigned char i = 0; i < 3; ++i)
		{
			std::pair<Position3D, float> &p = normalAccum[curr[i]];
			p.first += normal * area;
			p.second++;
		}
	}
	
	for (size_t i = 0, e = normalAccum.size(); i < e; ++i)
	{
		m_normals[i] = (normalAccum[i].first / normalAccum[i].second).normalized();
	}
	// TODO vertices with multiple normals
}

void DirectGL::Geometry::IndexedTriangleModel::transform(const Cameras::Transformation &transform)
{
	if (!m_vertices.empty() && m_normals.empty())
		generateNormals();

	Eigen::Matrix4f transformation = transform.getMatrix();
	for (VertexBuffer::iterator it = m_vertices.begin(), e = m_vertices.end(); it != e; ++it)
	{
		Position3D &curr = *it;
		Eigen::Vector4f tmp = transformation * Eigen::Vector4f(curr.x(), curr.y(), curr.z(), 1.f);
		tmp /= tmp.w();
		curr.x() = tmp.x();
		curr.y() = tmp.y();
		curr.z() = tmp.z();
	}

	transformation = transformation.inverse().transpose();
	for (NormalBuffer::iterator it = m_normals.begin(), e = m_normals.end(); it != e; ++it)
	{
		Position3D &curr = *it;
		Eigen::Vector4f tmp = transformation * Eigen::Vector4f(curr.x(), curr.y(), curr.z(), 0.f);
		tmp.normalize();
		curr.x() = tmp.x();
		curr.y() = tmp.y();
		curr.z() = tmp.z();
	}

	if (isAllocated())
		glFree();
}

void DirectGL::Geometry::IndexedTriangleModel::toFileStream(std::ofstream &stream) const
{
	ModelBase::toFileStream(stream);
#define BUFFER_WRITE(type, source) \
	{ \
		type::size_type size = source.size(); \
		stream.write(reinterpret_cast<const char*>(&size), sizeof(type::size_type)); \
		for (type::const_iterator it = source.begin(), e = source.end(); it != e; ++it) \
		{ \
			stream.write(reinterpret_cast<const char*>(&(*it)), sizeof(type::value_type)); \
		} \
	}

	BUFFER_WRITE(TriangleBuffer, m_triangles);

#undef BUFFER_WRITE

	stream.write(reinterpret_cast<const char*>(&m_transformation), sizeof(Cameras::Transformation));
}

void DirectGL::Geometry::IndexedTriangleModel::fromFileStream(std::ifstream &stream)
{
	ModelBase::fromFileStream(stream);
#define BUFFER_READ(type, target) \
	{ \
		type::size_type size; \
		stream.read(reinterpret_cast<char*>(&size), sizeof(type::size_type)); \
		for (type::size_type i = 0; i < size; ++i) \
		{ \
			type::value_type tmp; \
			stream.read(reinterpret_cast<char*>(&tmp), sizeof(type::value_type)); \
			target.push_back(tmp); \
		} \
	}

	BUFFER_READ(TriangleBuffer, m_triangles);

#undef BUFFER_READ

	stream.read(reinterpret_cast<char*>(&m_transformation), sizeof(Cameras::Transformation));
	m_aabb.bound(m_vertices);
}

void DirectGL::Geometry::IndexedTriangleModel::clear()
{
	ModelBase::clear();
	m_triangles.clear();
}

bool rayIntersectsTriangle(const DirectGL::Position3D &p, const DirectGL::Position3D &d,
	const DirectGL::Position3D &v0, const DirectGL::Position3D &v1, const DirectGL::Position3D &v2, float &dist) {
	
	DirectGL::Position3D e1 = v1 - v0,
								   e2 = v2 - v0;

	DirectGL::Position3D h = d.cross(e2);
	float a = e1.dot(h);

	if (a > -0.00001 && a < 0.00001)
		return(false);

	float f = 1 / a;
	DirectGL::Position3D s = p - v0;
	float u = f * s.dot(h);

	if (u < 0.0 || u > 1.0)
		return(false);

	DirectGL::Position3D q = s.cross(e1);
	float v = f * d.dot(q);

	if (v < 0.0 || u + v > 1.0)
		return(false);

	float t = f * e2.dot(q);
	dist = t;
	if (t > 0.00001)
		return(true);
	else
		return (false);

}

float DirectGL::Geometry::IndexedTriangleModel::raycut(const Position3D &start, const Position3D &ray, Direction3D &normal, const Cameras::Transformation &additionalTF) const
{
	float retVal = FLT_MAX;
	Eigen::Matrix4f myTransInv = (additionalTF.getMatrix() * m_transformation.getMatrix()).inverse();
	Eigen::Vector4f tfStart = myTransInv * Eigen::Vector4f(start.x(), start.y(), start.z(), 1.f);
	Eigen::Vector4f tfRay = myTransInv * Eigen::Vector4f(ray.x(), ray.y(), ray.z(), 0.f);
	Position3D newStart(tfStart.data());
	Position3D newRay(tfRay.data());
	newRay.normalize();
	for (TriangleBuffer::const_iterator it = m_triangles.begin(), e = m_triangles.end(); it != e; ++it)
	{
		const TriangleBuffer::value_type &curr = *it;
		const VertexBuffer::value_type &v0 = m_vertices[curr.x()],
									   &v1 = m_vertices[curr.y()],
									   &v2 = m_vertices[curr.z()];
		float dist;
		if (rayIntersectsTriangle(newStart, newRay, v0, v1, v2, dist))
		{
			if (dist < retVal)
			{
				retVal = dist;
				normal = m_normals[curr.x()]; // TODO inaccurate;
			}
		}
	}

	return retVal;
}