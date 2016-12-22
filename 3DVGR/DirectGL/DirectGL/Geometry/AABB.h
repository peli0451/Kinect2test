#pragma once

#include <epoxy/gl.h>
#include <DirectGL/Types.h>

namespace DirectGL
{
	namespace Geometry
	{
		class AABB
		{
		public:
			AABB() : m_minEdge(Position3D(FLT_MAX, FLT_MAX, FLT_MAX)), m_maxEdge(Position3D(-FLT_MAX, -FLT_MAX, -FLT_MAX)) { }
			AABB(const Position3D &edge1, const Position3D &edge2);

			void extend(const Position3D &point);
			void extend(const AABB &other);
			void bound(const VertexBuffer &vertices);
			void bound(const Position3D &p1, const Position3D &p2);
			bool contains(const Position3D &point) const;

			inline const Position3D &getMinEdge() const { return m_minEdge; }
			inline const Position3D &getMaxEdge() const { return m_maxEdge; }
			inline float getVolume() const { return m_volume; }

			inline void clear() { m_minEdge = Position3D(FLT_MAX, FLT_MAX, FLT_MAX); m_maxEdge = Position3D(-FLT_MAX, -FLT_MAX, -FLT_MAX); }

		private:
			void updateVolume();

			Position3D m_minEdge,
					   m_maxEdge;
			float m_volume;
		};
	}
}