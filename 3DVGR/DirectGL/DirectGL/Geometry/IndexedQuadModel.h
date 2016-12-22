#pragma once

#include <DirectGL/Forward.h>
#include <DirectGL/Geometry/ModelBase.h>

namespace DirectGL
{
	namespace Geometry
	{
		class IndexedQuadModel : public ModelBase
		{
		public:
			LOCAL_DECLARE(IndexedQuadModel)

			using index_buffer_type = QuadBuffer;
			IndexedQuadModel() {}
			IndexedQuadModel(std::ifstream &stream) { fromFileStream(stream); }
			IndexedQuadModel(const VertexBuffer &vertices, const QuadBuffer &indices,
				const NormalBuffer &normals = NormalBuffer(), const TexCoordBuffer texCoords = TexCoordBuffer(),
				const Texturing::ColorBuffer &colors = Texturing::ColorBuffer())
				: ModelBase(vertices, normals, texCoords, colors), m_quads(indices) {}

			~IndexedQuadModel() { clear(); }

			void toFileStream(std::ofstream &stream) const;
			void fromFileStream(std::ifstream &stream);

			void draw();

			const index_buffer_type &getQuads() const { return m_quads; }
			void setQuads(const QuadBuffer &quads); 

			void clear();

		private:
			Buffers::Buffer	m_glQuadBuffer;

		protected:
			virtual void glAllocate();
			virtual void glFree();
			void generateNormals();

			index_buffer_type m_quads;
		};

		class IndexedQuadPatch : public IndexedQuadModel
		{
		public:
			LOCAL_DECLARE(IndexedQuadPatch)

			IndexedQuadPatch() {}
			IndexedQuadPatch(std::ifstream &stream) { fromFileStream(stream); }
			IndexedQuadPatch(const VertexBuffer &vertices, const QuadBuffer &indices,
				const NormalBuffer &normals = NormalBuffer(), const TexCoordBuffer texCoords = TexCoordBuffer(),
				const Texturing::ColorBuffer &colors = Texturing::ColorBuffer())
				: IndexedQuadModel(vertices, indices, normals, texCoords, colors) {}

			void draw();
			void draw(int numInstances);
		};
	}
}