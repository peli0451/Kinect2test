#pragma once

#include <DirectGL/Types.h>
#include <DirectGL/Texturing/Color.h>

#include <string>
#include <vector>
#include <unordered_map>

namespace DirectGL
{
	namespace Utility
	{
		class MTLFile
		{
		public:
			struct Material
			{
				Material(const std::string &name = std::string("")) :
				name(name), ambient(Texturing::Color3f::Zero()), diffuse(Texturing::Color3f::Ones()),
				specular(Texturing::Color3f::Zero()), emissive(Texturing::Color3f::Zero()),
				shininess(1.f), refraction(1.f), transparency(1.f) {}

				Texturing::Color3f ambient,
								   diffuse,
								   specular,
								   emissive;
				GLfloat shininess,
						refraction,
						transparency;
				std::string ambientTexture,
							diffuseTexture,
							specularTexture,
							transparencyTexture,
							bumpTexture,
							name;
			};

			MTLFile() { init(); }
			MTLFile(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction) = defaultProgress){ init(); fromFile(filename, progressCallback); }

			void fromFile(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction) = defaultProgress);
			const Material &getMaterial(const std::string &name) const { MaterialMap::const_iterator f = m_materials.find(name);  if (f != m_materials.end()) return f->second; else return defaultMaterial; }
			void clear();

		private:
			enum MTLType
			{
				ambient,
				diffuse,
				specular,
				shininess,
				emissive,
				transparency,
				refraction,
				materialDef,
				ambientTex,
				diffuseTex,
				specularTex,
				transparencyTex,
				bumpTex
					
			};
			typedef std::unordered_map<std::string, MTLType> LineTypeMap;
			typedef std::unordered_map<std::string, Material> MaterialMap;

			static Material defaultMaterial;

			void init();
			static void defaultProgress(float progress, const std::string &currentAction) {}

			LineTypeMap m_lineTypes;
			MaterialMap m_materials;
		};

		class OBJFile
		{
		public:
			struct Object
			{
				struct Group
				{
					struct MaterialSubGroup
					{
						Buffer3Di vertexIndices,
						textureIndices,
						normalIndices;
						std::string material;
					};
					typedef std::vector<MaterialSubGroup> SubGroupList;
					SubGroupList MaterialSubGroups;
					std::string name;	
				};
				typedef std::vector<Group> GroupList;
				GroupList groups;
				std::string name;
			};
			typedef std::vector<Object> ObjectList;

			OBJFile(){ init(); }
			OBJFile(const std::string &filename, void (*progressCallback)(float progress, const std::string &currentAction) = defaultProgress) { init(); fromFile(filename, progressCallback); }

			void fromFile(const std::string &filename, void(*progressCallback)(float progress, const std::string &currentAction) = defaultProgress);
			const VertexBuffer &getVertexList() const { return m_vertexList; }
			const NormalBuffer &getNormalList() const { return m_normalList; }
			const TexCoordBuffer &getTexCoordList() const { return m_texCoordList; }
			const ObjectList &getObjects() const { return m_objects; }
			const MTLFile::Material &getMaterial(const std::string &name) const { return m_materialFile.getMaterial(name); }

			void clear();

		private:
			enum OBJType
			{
				vertex,
				normal,
				texCoord,
				object,
				group,
				material,
				face,
				materialFile,
				comment
			};
			typedef std::unordered_map<std::string, OBJType> LineTypeMap;
			
			void init();
			void loadMaterialFile(const std::string &filename);
			static void defaultProgress(float progress, const std::string &currentAction) {}

			VertexBuffer m_vertexList;
			NormalBuffer m_normalList;
			TexCoordBuffer m_texCoordList;
			ObjectList m_objects;
			LineTypeMap m_lineTypes;
			MTLFile m_materialFile;
			// TODO materials;
		};
	}
}