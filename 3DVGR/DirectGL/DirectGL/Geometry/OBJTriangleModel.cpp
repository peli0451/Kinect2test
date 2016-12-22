#include "stdafx.h"
#include "OBJTriangleModel.h"

#include <Utility/OBJFile.h>
#include <map>

void DirectGL::Geometry::OBJTriangleModel::fromFile(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction))
{
	// TODO error files?
	Utility::OBJFile file(filename, progressCallback);
	const VertexBuffer &vertices(file.getVertexList());
	const NormalBuffer &normals(file.getNormalList());
	const TexCoordBuffer &texCoords(file.getTexCoordList());
	const Utility::OBJFile::ObjectList &objects(file.getObjects());
	clear();
	
	//m_vertices.reserve(vertices.size());
	//m_normals.reserve(normals.size());
	//m_texCoords.reserve(texCoords.size());
	
	struct Triple
	{ 
		Triple(int v, int t, int n) : vertex(v), texCoord(t), normal(n){} 
		bool operator<(const Triple &other) const 
		{ 
			return vertex < other.vertex || 
				(vertex == other.vertex && texCoord < other.texCoord) ||
				(vertex == other.vertex && texCoord == other.texCoord && normal < other.normal);
		} 
		
		int vertex, texCoord, normal; 
	};

	struct ModelStub
	{
		VertexBuffer vertices;
		TriangleBuffer triangles;
		NormalBuffer normals;
		TexCoordBuffer texCoords;
		Texturing::MaterialPtr material;
	};

	std::map<std::string, ModelStub> materialMap;
	size_t pp = 0,
		gCount = 0;
	for (Utility::OBJFile::ObjectList::const_iterator it = objects.begin(), e = objects.end(); it != e; ++it)
	{
		const Utility::OBJFile::Object &currentObject = *it;
		for (Utility::OBJFile::Object::GroupList::const_iterator it2 = currentObject.groups.begin(), e2 = currentObject.groups.end(); it2 != e2; ++it2)
		{
			auto &currentGroup = *it2;
			for (auto it3 = currentGroup.MaterialSubGroups.begin(), e3 = currentGroup.MaterialSubGroups.end(); it3 != e3; ++it3)
				gCount += it3->vertexIndices.size();
		}
	}

	for (Utility::OBJFile::ObjectList::const_iterator it = objects.begin(), e = objects.end(); it != e; ++it)
	{
		const Utility::OBJFile::Object &currentObject = *it;
		for (Utility::OBJFile::Object::GroupList::const_iterator it2 = currentObject.groups.begin(), e2 = currentObject.groups.end(); it2 != e2; ++it2)
		{
			auto &currentGroup = *it2;
			for (auto it3 = currentGroup.MaterialSubGroups.begin(), e3 = currentGroup.MaterialSubGroups.end(); it3 != e3; ++it3)
			{
				std::map<Triple, int> uniqueVertices;
				auto &currentSubGroup = *it3;

				// TODO if the geometry has no materials but is divided in many groups
				// the same default material is allocated for each individual group
				std::map<std::string, ModelStub>::iterator existing = materialMap.find(currentSubGroup.material);
				ModelStub *currentPart;
				if (existing != materialMap.end())
				{
					currentPart = &(existing->second);
				}
				else
				{
					materialMap[currentSubGroup.material] = ModelStub();
					existing = materialMap.find(currentSubGroup.material); // TODO seems awfully unintuitive
					currentPart = &(existing->second);
					currentPart->material.reset(new Texturing::Material(file.getMaterial(currentSubGroup.material))); // TODO link materials in the obj file class
				}
				if (currentSubGroup.textureIndices.empty())
				if (currentSubGroup.normalIndices.empty())
				{
					for (size_t i = 0, j = currentSubGroup.vertexIndices.size(); i < j; ++i)
					{
						const Index3D &currInd = currentSubGroup.vertexIndices[i];
						Index3D finalInd;
						for (int k = 0; k < 3; ++k)
						{
							Triple vert(currInd[k], -1, -1);
							std::map<Triple, int>::iterator found(uniqueVertices.find(vert));
							if (found == uniqueVertices.end())
							{
								finalInd[k] = static_cast<Index3D::Scalar>(currentPart->vertices.size());
								uniqueVertices[vert] = finalInd[k];
								currentPart->vertices.push_back(vertices[currInd[k]]);
							}
							else
								finalInd[k] = found->second;
						}
						currentPart->triangles.push_back(finalInd);
						progressCallback(static_cast<float>(++pp) / gCount * 100.f, "Converting OBJ Objects...");
					}
				}
				else
				{
					for (size_t i = 0, j = currentSubGroup.vertexIndices.size(); i < j; ++i)
					{
						const Index3D &currInd = currentSubGroup.vertexIndices[i],
							&currNorm = currentSubGroup.normalIndices[i];
						Index3D finalInd;
						for (int k = 0; k < 3; ++k)
						{
							Triple vert(currInd[k], -1, currNorm[k]);
							std::map<Triple, int>::iterator found(uniqueVertices.find(vert));
							if (found == uniqueVertices.end())
							{
								finalInd[k] = static_cast<Index3D::Scalar>(currentPart->vertices.size());
								uniqueVertices[vert] = finalInd[k];
								currentPart->vertices.push_back(vertices[currInd[k]]);
								currentPart->normals.push_back(normals[currNorm[k]]);
							}
							else
								finalInd[k] = found->second;
						}
						currentPart->triangles.push_back(finalInd);
						progressCallback(static_cast<float>(++pp) / gCount * 100.f, "Converting OBJ Objects...");
					}
				}
				else if (currentSubGroup.normalIndices.empty())
				{
					for (size_t i = 0, j = currentSubGroup.vertexIndices.size(); i < j; ++i)
					{
						const Index3D &currInd = currentSubGroup.vertexIndices[i],
							&currTex = currentSubGroup.textureIndices[i];
						Index3D finalInd;
						for (int k = 0; k < 3; ++k)
						{
							Triple vert(currInd[k], currTex[k], -1);
							std::map<Triple, int>::iterator found(uniqueVertices.find(vert));
							if (found == uniqueVertices.end())
							{
								finalInd[k] = static_cast<Index3D::Scalar>(currentPart->vertices.size());
								uniqueVertices[vert] = finalInd[k];
								currentPart->vertices.push_back(vertices[currInd[k]]);
								currentPart->texCoords.push_back(texCoords[currTex[k]]);
							}
							else
								finalInd[k] = found->second;
						}
						currentPart->triangles.push_back(finalInd);
						progressCallback(static_cast<float>(++pp) / gCount * 100.f, "Converting OBJ Objects...");
					}
				}
				else
				{
					for (size_t i = 0, j = currentSubGroup.vertexIndices.size(); i < j; ++i)
					{
						const Index3D &currInd = currentSubGroup.vertexIndices[i],
							&currTex = currentSubGroup.textureIndices[i],
							&currNorm = currentSubGroup.normalIndices[i];
						Index3D finalInd;
						for (int k = 0; k < 3; ++k)
						{
							Triple vert(currInd[k], currTex[k], currNorm[k]);
							std::map<Triple, int>::iterator found(uniqueVertices.find(vert));
							if (found == uniqueVertices.end())
							{
								finalInd[k] = static_cast<Index3D::Scalar>(currentPart->vertices.size());
								uniqueVertices[vert] = finalInd[k];
								currentPart->vertices.push_back(vertices[currInd[k]]);
								currentPart->texCoords.push_back(texCoords[currTex[k]]);
								currentPart->normals.push_back(normals[currNorm[k]]);
							}
							else
								finalInd[k] = found->second;
						}
						currentPart->triangles.push_back(finalInd);
						progressCallback(static_cast<float>(++pp) / gCount * 100.f, "Converting OBJ Objects...");
					}
				}
			}
		}
	}
	progressCallback(100.f, "Converting OBJ Objects...");
	pp = 0;
	for (std::map<std::string, ModelStub>::iterator it = materialMap.begin(), e = materialMap.end(); it != e; ++it)
	{
		ModelStub &curr = it->second;
		addMaterialGroup(MaterialGroup(curr.vertices, curr.triangles, curr.material, curr.normals, curr.texCoords));
		progressCallback(static_cast<float>(++pp) / materialMap.size() * 100.f, "Creating material Groups...");
	}
	progressCallback(100.f, "Creating material Groups...");
	progressCallback(100.f, "Done");
}