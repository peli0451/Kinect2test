#pragma once

#include <epoxy/gl.h>

#include <DirectGL/Types.h>
#include <DirectGL/Buffers/VertexArray.h>
#include <DirectGL/Buffers/Buffer.h>

namespace DirectGL
{
	namespace Geometry
	{
		class PointModel
		{
		public:
			PointModel() : m_size(1.f) { m_vertices.push_back(Position3D::Zero()); }
			PointModel(const VertexBuffer &vertices) : m_vertices(vertices), m_size(1.f) {}
			void draw();
			void draw(unsigned numinstances);
			void setSize(GLfloat size) { m_size = size; }
			void clear();

		protected:
			void glAllocate();
			void glFree();

			VertexBuffer m_vertices;

		private:
			Buffers::VertexArray m_array;
			Buffers::Buffer m_buffer;

			GLfloat m_size;
		};
	}
}