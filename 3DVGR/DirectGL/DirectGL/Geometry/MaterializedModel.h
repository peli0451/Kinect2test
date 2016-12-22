#pragma once

#include <DirectGL/Geometry/IndexedTriangleModel.h>
#include <DirectGL/Shaders/Program.h>
#include <DirectGL/Texturing/Material.h>

namespace DirectGL
{
	namespace Geometry
	{
		class MaterializedModel : public IndexedTriangleModel
		{
		public:
			MaterializedModel() : m_material(new Texturing::Material()) {}
			MaterializedModel(std::ifstream &stream) { fromFileStream(stream); }
			MaterializedModel(const MaterializedModel &other) { *this = other; }
			MaterializedModel(const VertexBuffer &vertices, const TriangleBuffer &indices,
				const Texturing::MaterialPtr material,
				const NormalBuffer &normals = NormalBuffer(), const TexCoordBuffer texCoords = TexCoordBuffer(),
				const Texturing::ColorBuffer &colors = Texturing::ColorBuffer())
				: m_material(material), IndexedTriangleModel(vertices, indices, normals, texCoords, colors) {}

			~MaterializedModel() { clear(); }

			void toFileStream(std::ofstream &stream) const;
			void fromFileStream(std::ifstream &stream);

			inline void draw() { IndexedTriangleModel::draw(); }
			inline void draw(Shaders::Program &program) { Renderable::draw(program); m_material->bindToProgram(program); IndexedTriangleModel::draw(); }
			inline void setMaterial(const Texturing::MaterialPtr &material) { m_material = material; }
			
			inline const Texturing::MaterialPtr &getMaterial() const { return m_material; }
			
			inline void clear() { m_material.reset(); IndexedTriangleModel::clear(); }
			// TODO assignment operator?
		private:
			Texturing::MaterialPtr m_material;
		};
	}
}