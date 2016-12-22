#include "stdafx.h"
#include "UniformBlock.h"
#include "Program.h"
#include "DirectGL.h"

#include "Utility\Logging.h"

#include <vector>

using namespace DirectGL::Shaders;

UniformBlock::UniformBlock(Program &program, GLuint index) : m_program(&program), m_index(index)
{
	program.link();
	GLint uniformCount;
	glGetActiveUniformBlockiv(program, index, GL_UNIFORM_BLOCK_DATA_SIZE, reinterpret_cast<GLint*>(&m_size));
	glGetActiveUniformBlockiv(program, index, GL_UNIFORM_BLOCK_ACTIVE_UNIFORMS, &uniformCount);
	
	std::vector<GLint> uniforms(uniformCount),
					   offsets(uniformCount),
					   types(uniformCount);
	glGetActiveUniformBlockiv(program, index, GL_UNIFORM_BLOCK_ACTIVE_UNIFORM_INDICES, uniforms.data());
	glGetActiveUniformsiv(program, uniformCount, reinterpret_cast<GLuint*>(uniforms.data()), GL_UNIFORM_OFFSET, offsets.data());
	glGetActiveUniformsiv(program, uniformCount, reinterpret_cast<GLuint*>(uniforms.data()), GL_UNIFORM_TYPE, types.data());
	for (GLint i = 0; i < uniformCount; ++i)
	{
		char buff[1024];
		ZeroMemory(buff, 1024);
		glGetActiveUniformName(program, uniforms[i], 1024, nullptr, buff);
		m_uniforms[buff] = BlockUniform(types[i], offsets[i]);
	}
}

const UniformBlock::BlockUniform &UniformBlock::getUniform(const std::string &name) const 
{
	auto f(m_uniforms.find(name));
	if (f == m_uniforms.end())
	{
		LOGEXCEPTION(("<Shaders::UniformBlock::getUniform> Uniform \"" + name + "\" not found!").c_str());
	}
	else
		return f->second;
}