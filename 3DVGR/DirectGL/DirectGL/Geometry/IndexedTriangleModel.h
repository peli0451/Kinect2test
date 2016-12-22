#pragma once

#include <DirectGL/Forward.h>
#include <DirectGL/Geometry/ModelBase.h>

namespace DirectGL
{
	namespace Geometry
	{
		class IndexedTriangleModel : public ModelBase
		{
		public:
			LOCAL_DECLARE(IndexedTriangleModel)

			using index_buffer_type = TriangleBuffer;
			IndexedTriangleModel() {}
			IndexedTriangleModel(std::ifstream &stream) { fromFileStream(stream); }
			IndexedTriangleModel(const IndexedTriangleModel &other) { *this = other; }
			IndexedTriangleModel(const VertexBuffer &vertices, const TriangleBuffer &indices,
				const NormalBuffer &normals = NormalBuffer(), const TexCoordBuffer texCoords = TexCoordBuffer(),
				const Texturing::ColorBuffer &colors = Texturing::ColorBuffer())
				: ModelBase(vertices, normals, texCoords, colors), m_triangles(indices) {}

			~IndexedTriangleModel() { clear(); }

			void toFileStream(std::ofstream &stream) const;
			void fromFileStream(std::ifstream &stream);

			void draw();

			const NormalBuffer &getNormals() const { if (!m_vertices.empty() && m_normals.empty()) const_cast<IndexedTriangleModel*>(this)->generateNormals(); return m_normals; }
			const index_buffer_type &getTriangles() const { return m_triangles; }

			void transform(const Cameras::Transformation &transform);
			float raycut(const Position3D &start, const Position3D &ray, Direction3D &normal, const Cameras::Transformation &additionalTF = Cameras::Transformation()) const;

			void clear();

			void operator=(const IndexedTriangleModel &other);

		private:
			Buffers::Buffer	m_glTriangleBuffer;

		protected:
			virtual void glAllocate();
			virtual void glFree();
			void generateNormals();

			index_buffer_type m_triangles;
		};
	}
}