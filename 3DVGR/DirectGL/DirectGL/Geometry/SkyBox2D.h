#pragma once

#include "IndexedTriangleModel.h"

#include "DirectGL/Texturing/Texture2D.h"

namespace DirectGL
{
	namespace Geometry
	{
		class Skybox2D : public IndexedTriangleModel
		{
		public:
			void create(float depth = 1.f);
			void setTexture(const Texturing::Texture2DPtr &texture) { m_texture = texture; }

			void draw() { m_texture->bind(0); IndexedTriangleModel::draw(); }

		private:
			Texturing::Texture2DPtr m_texture;

		};
	}
}