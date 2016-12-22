#include "stdafx.h"
#include "OBJFile.h"
#include "Logging.h"

#include <fstream>

namespace
{
	std::string nextSplit(const std::string &base, std::string::size_type &pos, const std::string &splitAt)
	{
		std::string retVal;
		if (pos == std::string::npos)
			return retVal;

		std::string::size_type fpos = base.find_first_of(splitAt, pos);

		if (fpos == std::string::npos)
		{
			retVal = base.substr(pos);
			pos = std::string::npos;
		}
		else
		{
			retVal = base.substr(pos, fpos - pos);
			pos = base.find_first_not_of(splitAt, fpos + 1);
		}

		return retVal;
	}

	void replaceFirst(std::string &base, const std::string &replaceAt, const std::string &replacement)
	{
		std::string::size_type pos = base.find(replaceAt);
		if (pos != std::string::npos)
			base.replace(pos, replacement.size(), replacement.c_str());
	}

	void replaceAll(std::string &base, const std::string &replaceAt, const std::string &replacement)
	{
		std::string::size_type pos = base.find(replaceAt);
		while (pos != std::string::npos)
		{
			base.replace(pos, replacement.size(), replacement.c_str());
			pos = base.find(replaceAt, pos + 1);
		}
	}
}

DirectGL::Utility::MTLFile::Material DirectGL::Utility::MTLFile::defaultMaterial;

void DirectGL::Utility::MTLFile::fromFile(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction))
{
	clear();

	std::ifstream file(filename);
	if (file.fail())
		return;
	file.seekg(0, std::ios_base::end);
	std::size_t size = static_cast<std::size_t>(file.tellg());
	file.seekg(0, std::ios_base::beg);
	char *data = new char[size + 1];
	std::memset(data, 0, sizeof(char)* (size + 1));
	file.read(data, size);
	file.close();

	std::string text(data);
	replaceAll(text, "\t", " ");
	Material currentMaterial("");

	std::string::size_type textPos = 0;

	std::string filepath = filename.substr(0, filename.find_last_of("\\/") + 1);

	while (textPos < size)
	{
		std::string::size_type nextPos = text.find_first_of('\n', textPos);
		if (nextPos == std::string::npos)
			break;
		std::string line = text.substr(textPos, nextPos - textPos);
		textPos = nextPos + 1;
		std::string::size_type pos = 0;
		std::string split;
		do
		{
			split = nextSplit(line, pos, " ");
		} while (split.empty() && pos != std::string::npos);

		LineTypeMap::iterator f = m_lineTypes.find(split);

		if (f != m_lineTypes.end())
		switch (f->second)
		{
		case ambient:
			{
				Texturing::Color3f parseColor;
				std::stringstream parser(line.substr(pos));
				parser >> currentMaterial.ambient.r();
				parser >> currentMaterial.ambient.g();
				parser >> currentMaterial.ambient.b();
			}
			break;
		case diffuse:
			{
				Texturing::Color3f parseColor;
				std::stringstream parser(line.substr(pos));
				parser >> currentMaterial.diffuse.r();
				parser >> currentMaterial.diffuse.g();
				parser >> currentMaterial.diffuse.b();
			}
			break;
		case specular:
			{
				Texturing::Color3f parseColor;
				std::stringstream parser(line.substr(pos));
				parser >> currentMaterial.specular.r();
				parser >> currentMaterial.specular.g();
				parser >> currentMaterial.specular.b();
			}
			break;
		case emissive:
			{
				 Texturing::Color3f parseColor;
				 std::stringstream parser(line.substr(pos));
				 parser >> currentMaterial.emissive.r();
				 parser >> currentMaterial.emissive.g();
				 parser >> currentMaterial.emissive.b();
			}
			break;
		case shininess:
			{
				std::stringstream parser(line.substr(pos));
				parser >> currentMaterial.shininess;
			}
			break;
		case transparency:
			{
				std::stringstream parser(line.substr(pos));
				parser >> currentMaterial.transparency;
			}
			break;
		case refraction:
			{
				 std::stringstream parser(line.substr(pos));
				 parser >> currentMaterial.refraction;
			}
			break;
		case materialDef:
			m_materials[currentMaterial.name] = currentMaterial;
			currentMaterial = Material(nextSplit(line, pos, " "));
			break;
		case ambientTex:
			currentMaterial.ambientTexture = filepath + nextSplit(line, pos, " ");
			break;
		case diffuseTex:
			currentMaterial.diffuseTexture = filepath + nextSplit(line, pos, " ");
			break;
		case specularTex:
			currentMaterial.specularTexture = filepath + nextSplit(line, pos, " ");
			break;
		case transparencyTex:
			currentMaterial.transparencyTexture = filepath + nextSplit(line, pos, " ");
			break;
		case bumpTex:
			currentMaterial.bumpTexture = filepath + nextSplit(line, pos, " ");
			break;
		}

		progressCallback(static_cast<float>(textPos) / size * 100.f, "Parsing MTL text...");
	}
	// All materials are assumed to be initialized by newmtl, therefore the first
	// material is invalid
	m_materials.erase("");
	m_materials[currentMaterial.name] = currentMaterial;
}
// TODO line types to lowercase?
void DirectGL::Utility::MTLFile::init()
{
	m_lineTypes["Ka"] = ambient;
	m_lineTypes["Kd"] = diffuse;
	m_lineTypes["Ks"] = specular;
	m_lineTypes["Ke"] = emissive;
	m_lineTypes["Ns"] = shininess;
	m_lineTypes["Ni"] = refraction;
	m_lineTypes["d"] = transparency;
	m_lineTypes["newmtl"] = materialDef;
	m_lineTypes["map_Ka"] = ambientTex;
	m_lineTypes["map_Kd"] = diffuseTex;
	m_lineTypes["map_Ks"] = specularTex;
	m_lineTypes["map_d"] = transparencyTex;
	m_lineTypes["map_bump"] = bumpTex;
}

void DirectGL::Utility::MTLFile::clear()
{
	m_materials.clear();
}

void DirectGL::Utility::OBJFile::fromFile(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction))
{
	clear();

	std::ifstream file(filename);
	if (file.fail())
		return;
	file.seekg(0, std::ios_base::end);
	std::size_t size = static_cast<std::size_t>(file.tellg());
	file.seekg(0, std::ios_base::beg);
	char *data = new char[size + 1];
	std::memset(data, 0, sizeof(char) * (size + 1));
	file.read(data, size);
	file.close();

	m_objects.push_back(Object());
	Object *currentObject = &m_objects[0];
	currentObject->name = "defaultObject";
	currentObject->groups.push_back(Object::Group());
	Object::Group *currentGroup = &(currentObject->groups[0]);
	currentGroup->name = "defaultGroup";
	currentGroup->MaterialSubGroups.push_back(Object::Group::MaterialSubGroup());
	Object::Group::MaterialSubGroup *currentMaterialGroup = &(currentGroup->MaterialSubGroups[0]);
	currentMaterialGroup->material = "default";

	std::string currentMaterial = "";
	std::string mtlFile = "";
	size_t currentVertexIndex[] = { 0, 0, 0 };
	Eigen::Vector3f test;

	std::string::size_type textPos = 0;
	std::string text(data);

	std::string filepath = filename.substr(0, filename.find_last_of("\\/") + 1);

	while (textPos < size)
	{
		std::string::size_type nextPos = text.find_first_of('\n', textPos);
		if (nextPos == std::string::npos)
			nextPos = size;
		std::string line = text.substr(textPos, nextPos - textPos);
		textPos = nextPos + 1;
		std::string::size_type pos = 0;
		std::string split;
		do
		{
			split = nextSplit(line, pos, " \t");
		} while (split.empty() && pos != std::string::npos);

		LineTypeMap::iterator f = m_lineTypes.find(split);

		if (f != m_lineTypes.end())
		switch (f->second)
		{
		case vertex:
			{
			   VertexBuffer::value_type vertex;
			   std::stringstream parser(line.substr(pos));
			   parser >> vertex.x();
			   parser >> vertex.y();
			   parser >> vertex.z();
			   m_vertexList.push_back(vertex);
			   ++currentVertexIndex[0];
			}
			break;
		case normal:
			{
				NormalBuffer::value_type normal;
				std::stringstream parser(line.substr(pos));
				parser >> normal.x();
				parser >> normal.y();
				parser >> normal.z();
				m_normalList.push_back(normal);
				++currentVertexIndex[2];
			}
			break;
		case texCoord:
			{
				TexCoordBuffer::value_type texCoord;
				std::stringstream parser(line.substr(pos));
				parser >> texCoord.x();
				parser >> texCoord.y();
				m_texCoordList.push_back(texCoord);
				++currentVertexIndex[1];
			}
			break;
		case object:
			// current object is finished, creating new one
			m_objects.push_back(Object());
			currentObject = &m_objects[m_objects.size() - 1];
			currentObject->name = nextSplit(line, pos, " ");
			currentObject->groups.push_back(Object::Group());
			currentGroup = &(currentObject->groups[currentObject->groups.size() - 1]);
			currentGroup->name = "defaultGroup";
			currentGroup->MaterialSubGroups.push_back(Object::Group::MaterialSubGroup());
			currentMaterialGroup = &(currentGroup->MaterialSubGroups[0]);
			currentMaterialGroup->material = currentMaterial;
			break;
		case group:
			{
				currentObject->groups.push_back(Object::Group());
				currentGroup = &(currentObject->groups[currentObject->groups.size() - 1]);
				currentGroup->name = nextSplit(line, pos, " ");
				currentGroup->MaterialSubGroups.push_back(Object::Group::MaterialSubGroup());
				currentMaterialGroup = &(currentGroup->MaterialSubGroups[currentGroup->MaterialSubGroups.size() - 1]);
				currentMaterialGroup->material = currentMaterial;
			}
			break;
		case material:
			currentMaterial = nextSplit(line, pos, " ");
			currentGroup->MaterialSubGroups.push_back(Object::Group::MaterialSubGroup());

			currentMaterialGroup = &(currentGroup->MaterialSubGroups[currentGroup->MaterialSubGroups.size() - 1]);
			currentMaterialGroup->material = currentMaterial;
			break;
		case materialFile:
			mtlFile = filepath + nextSplit(line, pos, " ");
			break;
		case face:
			{
				 // TODO different face types, definition types
					 // TODO index parsing still suboptimal
				bool normalPresent = false,
					 texCoordPresent = false;
				std::vector<Index3D> indices;
				while (pos != std::string::npos)
				{
					Index3D parsedInds(Index3D::Zero());
					std::string s(nextSplit(line, pos, " "));
					replaceFirst(s, "/", " ");
					std::stringstream parser(s);
					parser >> parsedInds[0];
					if (!parser.eof())
					{
						parser >> parsedInds[1];
						texCoordPresent = parsedInds[1] != 0;
						parser.seekg(1, std::ios_base::cur);
						if (!parser.eof())
						{
							parser >> parsedInds[2];
							normalPresent = parsedInds[2] != 0;
						}
					}

					for (int i = 0; i < 3; ++i)
						if (parsedInds[i] < 0)
							parsedInds[i] += static_cast<int>(currentVertexIndex[i]);
						else
							--parsedInds[i];
					
					indices.push_back(parsedInds);
				}

				switch (indices.size())
				{
				case 1:
				case 2:
					LOGEXCEPTION("<OBJFile::fromFile> Faces must have at least three vertices!");
					break;
				case 3:
					currentMaterialGroup->vertexIndices.push_back(Index3D(indices[0][0], indices[1][0], indices[2][0]));
					if (texCoordPresent) currentMaterialGroup->textureIndices.push_back(Index3D(indices[0][1], indices[1][1], indices[2][1]));
					if (normalPresent) currentMaterialGroup->normalIndices.push_back(Index3D(indices[0][2], indices[1][2], indices[2][2]));
					break;
				case 4:
					currentMaterialGroup->vertexIndices.push_back(Index3D(indices[0][0], indices[1][0], indices[2][0]));
					currentMaterialGroup->vertexIndices.push_back(Index3D(indices[2][0], indices[3][0], indices[0][0]));
					if (texCoordPresent)
					{
						currentMaterialGroup->textureIndices.push_back(Index3D(indices[0][1], indices[1][1], indices[2][1]));
						currentMaterialGroup->textureIndices.push_back(Index3D(indices[2][1], indices[3][1], indices[0][1]));
					}
					if (normalPresent)
					{
						currentMaterialGroup->normalIndices.push_back(Index3D(indices[0][2], indices[1][2], indices[2][2]));
						currentMaterialGroup->normalIndices.push_back(Index3D(indices[2][2], indices[3][2], indices[0][2]));
					}
					break;
				default:
						assert(false);
				}
			}
			break;
		}
		progressCallback(static_cast<float>(textPos) / size * 100.f, "Parsing OBJ text...");
	}

	for (ObjectList::size_type i = 0; i < m_objects.size(); ++i)
	{
		Object &curr = m_objects[i];

		for (Object::GroupList::size_type j = 0; j < curr.groups.size(); ++j)
		{
			Object::Group &currGroup = curr.groups[j];
			for (Object::Group::SubGroupList::size_type k = 0; k < currGroup.MaterialSubGroups.size(); ++k)
			{
				if (currGroup.MaterialSubGroups[k].vertexIndices.empty())
					currGroup.MaterialSubGroups.erase(currGroup.MaterialSubGroups.begin() + (k--));
			}

			if (currGroup.MaterialSubGroups.empty())
				curr.groups.erase(curr.groups.begin() + (j--));
		}

		if (curr.groups.empty())
			m_objects.erase(m_objects.begin() + (i--));
	}

	if (mtlFile.size())
		m_materialFile.fromFile(mtlFile, progressCallback);
}

void DirectGL::Utility::OBJFile::init()
{
	m_lineTypes["v"] = vertex;
	m_lineTypes["vt"] = texCoord;
	m_lineTypes["vn"] = normal;
	m_lineTypes["o"] = object;
	m_lineTypes["g"] = group;
	m_lineTypes["usemtl"] = material;
	m_lineTypes["mtllib"] = materialFile;
	m_lineTypes["f"] = face;
	m_lineTypes["#"] = comment;
	// TODO shine groups
}

void DirectGL::Utility::OBJFile::clear()
{
	m_vertexList.clear();
	m_normalList.clear();
	m_texCoordList.clear();
	m_objects.clear();
	m_materialFile.clear();
}

void DirectGL::Utility::OBJFile::loadMaterialFile(const std::string &filename)
{
	m_materialFile.fromFile(filename);
}