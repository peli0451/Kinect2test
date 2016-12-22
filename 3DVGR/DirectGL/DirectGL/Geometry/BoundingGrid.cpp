#include "stdafx.h"
#include "BoundingGrid.h"

void DirectGL::Geometry::BoundingGrid::fromAABB(const AABB &aabb)
{
	clear();

	const Position3D &ll = aabb.getMinEdge();
	const Position3D &tr = aabb.getMaxEdge();

	m_vertices.push_back(ll);
	m_vertices.push_back(Position3D(ll.x(), ll.y(), tr.z()));
	m_vertices.push_back(Position3D(tr.x(), ll.y(), ll.z()));
	m_vertices.push_back(Position3D(tr.x(), ll.y(), tr.z()));
	m_vertices.push_back(Position3D(ll.x(), tr.y(), ll.z()));
	m_vertices.push_back(Position3D(ll.x(), tr.y(), tr.z()));
	m_vertices.push_back(Position3D(tr.x(), tr.y(), ll.z()));
	m_vertices.push_back(Position3D(tr.x(), tr.y(), tr.z()));

	m_lines.push_back(Index2D(0, 1));
	m_lines.push_back(Index2D(0, 2));
	m_lines.push_back(Index2D(0, 4));
	m_lines.push_back(Index2D(1, 3));
	m_lines.push_back(Index2D(1, 5));
	m_lines.push_back(Index2D(2, 3));
	m_lines.push_back(Index2D(2, 6));
	m_lines.push_back(Index2D(3, 7));
	m_lines.push_back(Index2D(4, 5));
	m_lines.push_back(Index2D(4, 6));
	m_lines.push_back(Index2D(5, 7));
	m_lines.push_back(Index2D(6, 7));
}

void DirectGL::Geometry::BoundingGrid::glAllocate()
{
	const char *vert = "#version 330 \n uniform mat4 transformation; layout(location = 0) in vec3 position; out float id; void main(){ gl_Position = transformation * vec4(position, 1.f);  id = gl_VertexID; }";
	const char *frag = "#version 330 \n in float id; layout(location = 0) out vec4 finalColor; void main() { if( abs(id - round(id)) > 0.2) discard; finalColor = vec4(0.f, 1.f, 0.5f, 1.f); }";

	DirectGL::Shaders::Shader::Ptr vertShader(new DirectGL::Shaders::Shader(GL_VERTEX_SHADER));
	vertShader->addSource(vert);
	m_program.attachShader(vertShader);

	DirectGL::Shaders::Shader::Ptr fragShader(new DirectGL::Shaders::Shader(GL_FRAGMENT_SHADER));
	fragShader->addSource(frag);
	m_program.attachShader(fragShader);
	m_program.link();

	m_array.bind();
	if (m_vertices.size())
	{
		m_vertbuff.BufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(VertexBuffer::value_type), m_vertices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
		glEnableVertexAttribArray(0);
	}

	if (m_lines.size())
	{
		m_linebuff.BufferData(GL_ELEMENT_ARRAY_BUFFER, m_lines.size() * sizeof(LineBuffer::value_type), m_lines.data(), GL_STATIC_DRAW);
	}
	m_array.unbind();
}

void DirectGL::Geometry::BoundingGrid::glFree()
{
	m_program.clear();
	m_vertbuff.clear();
	m_linebuff.clear();
}


void DirectGL::Geometry::BoundingGrid::draw()
{
	if (!m_program.isValid())
		glAllocate();
	m_program.use();
	m_array.bind();
	m_program.UniformMatrix4fv("transformation", 1, GL_FALSE, m_transformation.data());
	glDrawElements(GL_LINES, static_cast<GLsizei>(m_lines.size() * 2), GL_UNSIGNED_INT, nullptr);
}
void DirectGL::Geometry::BoundingGrid::clear()
{
	m_vertices.clear();
	m_lines.clear();
	glFree();
}