#include "stdafx.h"
#include "MaterializedModel.h"

void DirectGL::Geometry::MaterializedModel::toFileStream(std::ofstream &stream) const
{
	IndexedTriangleModel::toFileStream(stream);
	getMaterial()->toFileStream(stream);
}

void DirectGL::Geometry::MaterializedModel::fromFileStream(std::ifstream &stream)
{
	IndexedTriangleModel::fromFileStream(stream);
	Texturing::MaterialPtr mat(new Texturing::Material());
	mat->fromFileStream(stream);
	setMaterial(mat);
}