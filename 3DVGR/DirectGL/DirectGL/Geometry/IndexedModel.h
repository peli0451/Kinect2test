#pragma once

#include <epoxy/gl.h>

#include <DirectGL/Types.h>
#include <DirectGL/Renderable.h>
#include <DirectGL/Buffers/Buffer.h>
#include <DirectGL/Buffers/VertexArray.h>

namespace DirectGL
{
	class ModelBase : public Renderable
	{
	public:
		ModelBase() {}
		ModelBase(std::ifstream &stream) { fromFileStream(stream); }
		ModelBase(const VertexBuffer &vertices, const NormalBuffer &normals = NormalBuffer(), const TexCoordBuffer texCoords = TexCoordBuffer(),
			const Texturing::ColorBuffer &colors = Texturing::ColorBuffer())
			: m_vertices(vertices), m_normals(normals), m_texCoords(texCoords), m_colors(colors)
		{
			m_aabb.bound(vertices);
		}

		virtual void toFileStream(std::ofstream &stream) const;
		virtual void fromFileStream(std::ifstream &stream);

		void clear();
		virtual void draw() = 0;

		const VertexBuffer &getVertices() const { return m_vertices; }
		const NormalBuffer &getNormals() const { return m_normals; }
		const TexCoordBuffer &getTexCoords() const { return m_texCoords; }
		const ColorBuffer &getColors() const { return m_colors; }

		void setVertices(const VertexBuffer &vertices);
		void setNormals(const NormalBuffer &normals);
		void setTexCoords(const TexCoordBuffer &texCoords);
		void setColors(const ColorBuffer &colors);

		void operator=(const ModelBase &other);

	protected:
		virtual void glAllocate();
		virtual void glFree();
		inline bool isAllocated() { return m_glArray.isValid(); }

		VertexBuffer	m_vertices;
		NormalBuffer	m_normals;
		TexCoordBuffer	m_texCoords;
		ColorBuffer		m_colors;
		Buffers::VertexArray m_glArray;

	private:
		Buffers::Buffer	m_glVertexBuffer,
						m_glNormalBuffer, 
						m_glTexCoordBuffer,
						m_glColorBuffer;

	};
}