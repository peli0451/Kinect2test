#include "stdafx.h"
#include "ModelBase.h"

void DirectGL::ModelBase::glAllocate()
{
	// TODO add isBound check for m_glArray
	#define GEN_BIND_FILL_BUFFER(name, attribIndex, data, dataSize, elemCount) \
			name.BufferData(GL_ARRAY_BUFFER, dataSize, data, GL_STATIC_DRAW); \
			glVertexAttribPointer(attribIndex, elemCount, GL_FLOAT, GL_FALSE, 0, NULL); \
			glEnableVertexAttribArray(attribIndex);

	if (m_vertices.size())
	{
		GEN_BIND_FILL_BUFFER(m_glVertexBuffer, 0, m_vertices.data(), sizeof(VertexBuffer::value_type) * m_vertices.size(), 3);
	}
	if (m_normals.size())
	{
		GEN_BIND_FILL_BUFFER(m_glNormalBuffer, 1, m_normals.data(), sizeof(NormalBuffer::value_type) * m_normals.size(), 3);
	}
	if (m_texCoords.size())
	{
		GEN_BIND_FILL_BUFFER(m_glTexCoordBuffer, 2, m_texCoords.data(), sizeof(TexCoordBuffer::value_type) * m_texCoords.size(), 2);
	}
	if (m_colors.size())
	{
		GEN_BIND_FILL_BUFFER(m_glColorBuffer, 3, m_colors.data(), sizeof(Texturing::ColorBuffer::value_type) * m_colors.size(), 3);
	}
}

void DirectGL::ModelBase::glFree()
{
	if (isAllocated())
	{
		m_glArray.unbind();
		m_glVertexBuffer.clear();
		m_glNormalBuffer.clear();
		m_glTexCoordBuffer.clear();
		m_glArray.clear();
	}
}

void DirectGL::ModelBase::toFileStream(std::ofstream &stream) const 
{
#define BUFFER_WRITE(type, source) \
			{ \
			type::size_type size = source.size(); \
			stream.write(reinterpret_cast<const char*>(&size), sizeof(type::size_type)); \
			for (type::const_iterator it = source.begin(), e = source.end(); it != e; ++it) \
			{ \
			stream.write(reinterpret_cast<const char*>(&(*it)), sizeof(type::value_type)); \
		} \
		}

	BUFFER_WRITE(VertexBuffer, m_vertices);
	BUFFER_WRITE(NormalBuffer, m_normals);
	BUFFER_WRITE(TexCoordBuffer, m_texCoords);

#undef BUFFER_WRITE
}

void DirectGL::ModelBase::fromFileStream(std::ifstream &stream)
{
	clear();
	#define BUFFER_READ(type, target) \
		{ \
		type::size_type size; \
		stream.read(reinterpret_cast<char*>(&size), sizeof(type::size_type)); \
		for (type::size_type i = 0; i < size; ++i) \
		{ \
		type::value_type tmp; \
		stream.read(reinterpret_cast<char*>(&tmp), sizeof(type::value_type)); \
		target.push_back(tmp); \
	} \
	}

	BUFFER_READ(VertexBuffer, m_vertices);
	BUFFER_READ(NormalBuffer, m_normals);
	BUFFER_READ(TexCoordBuffer, m_texCoords);
	//TODO color buffer missing !

#undef BUFFER_READ
}

void DirectGL::ModelBase::clear()
{
	Renderable::clear();
	glFree();
	m_vertices.clear();
	m_normals.clear();
	m_texCoords.clear();
	m_colors.clear();
}

void DirectGL::ModelBase::setVertices(const VertexBuffer &vertices)
{
	glFree();
	m_vertices = vertices;
}

void DirectGL::ModelBase::setNormals(const NormalBuffer &normals)
{
	glFree();
	m_normals = normals;
}

void DirectGL::ModelBase::setTexCoords(const TexCoordBuffer &texCoords)
{
	glFree();
	m_texCoords = texCoords;
}

void DirectGL::ModelBase::setColors(const ColorBuffer &colors)
{
	glFree();
	m_colors = colors;
}

void DirectGL::ModelBase::operator=(const ModelBase &other)
{
	Renderable::operator=(other);
	if (m_glArray != other.m_glArray && isAllocated())
		glFree();
	m_vertices = other.m_vertices;
	m_normals = other.m_normals;
	m_texCoords = other.m_texCoords;
	m_colors = other.m_colors;
}