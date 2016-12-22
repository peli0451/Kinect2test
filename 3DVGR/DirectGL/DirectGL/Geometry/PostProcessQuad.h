#pragma once

#include <epoxy/gl.h>

#include <DirectGL/Forward.h>

#include <DirectGL/Buffers/VertexArray.h>
#include <DirectGL/Buffers/Buffer.h>

namespace DirectGL
{
	namespace Geometry
	{
		class PostProcessQuad
		{
		public:
			LOCAL_DECLARE(PostProcessQuad)

			void create();
			void clear();

			void draw();
			void draw(unsigned numinstances);

		private:
			Buffers::VertexArray m_array;
			Buffers::Buffer	m_vertices;
		};
	}
}