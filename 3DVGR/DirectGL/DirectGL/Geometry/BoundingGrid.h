#pragma once

#include <DirectGL/Geometry/IndexedTriangleModel.h>
#include <DirectGL/Shaders/Program.h>
#include <DirectGL/Buffers/VertexArray.h>
#include <DirectGL/Buffers/Buffer.h>

namespace DirectGL
{
	namespace Geometry
	{
		class BoundingGrid
		{
		public:
			void fromAABB(const AABB &aabb);
			void draw();
			void setTransformation(const Eigen::Matrix4f &transformation) { m_transformation = transformation; }
			void clear();

		private:
			void glAllocate();
			void glFree();
			Shaders::Program m_program;

			VertexBuffer m_vertices;
			LineBuffer m_lines;

			DirectGL::Buffers::Buffer m_vertbuff,
									  m_linebuff;
			DirectGL::Buffers::VertexArray m_array;
			Eigen::Matrix4f m_transformation;
		};
	}
}