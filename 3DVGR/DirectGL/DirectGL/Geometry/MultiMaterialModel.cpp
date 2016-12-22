#include "stdafx.h"
#include "MultiMaterialModel.h"
#include <set>

void DirectGL::Geometry::MultiMaterialModel::toFileStream(std::ofstream &stream) const
{
	// saving geometry
	MaterialGroups::size_type size = m_groups.size();
	stream.write(reinterpret_cast<const char*>(&size), sizeof(MaterialGroups::size_type));
	for (auto it = m_groups.begin(), e = m_groups.end(); it != e; ++it)
	{
		it->toFileStream(stream);
	}

	stream.write(reinterpret_cast<const char*>(&m_transformation), sizeof(Cameras::Transformation));
}

void  DirectGL::Geometry::MultiMaterialModel::fromFileStream(std::ifstream &stream)
{
	clear();
	MaterialGroups::size_type size;
	stream.read(reinterpret_cast<char*>(&size), sizeof(MaterialGroups::size_type));
	m_groups.resize(size);
	for (auto i = 0; i < size; ++i)
	{	
		m_groups[i].fromFileStream(stream);
		m_aabb.extend(m_groups[i].getAABB());
	}

	stream.read(reinterpret_cast<char*>(&m_transformation), sizeof(Cameras::Transformation));
}

void DirectGL::Geometry::MultiMaterialModel::draw(Shaders::Program &program)
{
	for (MaterialGroups::iterator it = m_groups.begin(), e = m_groups.end(); it != e; ++it)
	{
		program.UniformMatrix4fv("modelT", 1, GL_FALSE, m_transformation.getMatrix().data());
		it->draw(program);
	}
}

void DirectGL::Geometry::MultiMaterialModel::draw()
{
	for (MaterialGroups::iterator it = m_groups.begin(), e = m_groups.end(); it != e; ++it)
	{
		it->draw();
	}
}

void DirectGL::Geometry::MultiMaterialModel::addMaterialGroup(const MaterialGroup &group)
{
	m_groups.push_back(group);
	m_aabb.extend(group.getAABB());
}

float DirectGL::Geometry::MultiMaterialModel::raycut(const Position3D &start, const Position3D &ray, Direction3D &normal) const
{
	float retVal = FLT_MAX;
	for (MaterialGroups::const_iterator it = m_groups.begin(), e = m_groups.end(); it != e; ++it)
	{
		retVal = std::min(it->raycut(start, ray, normal, m_transformation), retVal);
	}

	return retVal;
}

void DirectGL::Geometry::MultiMaterialModel::clear()
{
	m_groups.clear();
	m_aabb = AABB();
	m_transformation.reset();
}

void DirectGL::Geometry::MultiMaterialModel::transform(const Cameras::Transformation &transform)
{
	for (auto &i : m_groups)
	{
		i.transform(transform);
	}
}