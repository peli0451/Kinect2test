#pragma once

#include <DirectGL/Forward.h>
#include <DirectGL/Geometry/ModelBase.h>
#include <DirectGL/Types.h>

namespace DirectGL
{
	namespace Geometry
	{
		class IndexedLineModel : public ModelBase
		{
		public:
			LOCAL_DECLARE(IndexedLineModel)

			using index_buffer_type = LineBuffer;
			IndexedLineModel() {}
			IndexedLineModel(std::ifstream &stream) { fromFileStream(stream); }
			IndexedLineModel(const VertexBuffer &vertices, const LineBuffer &indices, const NormalBuffer &normals = NormalBuffer(), const TexCoordBuffer texCoords = TexCoordBuffer(),
				const Texturing::ColorBuffer &colors = Texturing::ColorBuffer())
				: ModelBase(vertices, normals, texCoords, colors) {}

			~IndexedLineModel() { clear(); }

			void toFileStream(std::ofstream &stream) const {}
			void fromFileStream(std::ifstream &stream) {}

			void draw();
			void clear();

			const LineBuffer &getIndices() const { return m_lines; }

			void operator=(const IndexedLineModel &other);

		private:
			Buffers::Buffer	m_glLineBuffer;	

		protected:
			virtual void glAllocate();
			virtual void glFree();
			index_buffer_type m_lines;
		};
	}
}