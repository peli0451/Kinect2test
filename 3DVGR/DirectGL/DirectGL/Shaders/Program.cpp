#include "stdafx.h"
#include "Program.h"
#include "Utility/GLError.h"

void DirectGL::Shaders::Program::fromForeignProgram(GLuint program)
{
	assert(program != 0);
	if (program == 0)
		LOGEXCEPTION("<DirectGL::Shaders::Program::Program(GLuint)> Invalid foreign id passed!");
	
	clear();
	m_id = program;
	m_idOwned = false;
}

void DirectGL::Shaders::Program::create()
{
	clear();
	Object::create();
	m_id = glCreateProgram();
	m_idOwned = true;
	OPENGL_ERROR_CHECK("Failed to create shader program: ");
}

void DirectGL::Shaders::Program::clear()
{
	if (m_id && m_idOwned)
	{
		glDeleteProgram(m_id);
	}
	m_id = 0;
	m_idOwned = false;
	m_linked = false;
	m_vertexShader.reset();
	m_fragmentShader.reset();
	m_tessControlShader.reset();
	m_tessEvalShader.reset();
	m_geometryShader.reset();
	Object::clear();
}

void DirectGL::Shaders::Program::use() const
{
	if (!m_linked) const_cast<Program*>(this)->link();
	if (m_linked && getOwner().bindObject(GL_CURRENT_PROGRAM, *this))
		glUseProgram(m_id);
}

void DirectGL::Shaders::Program::attachShader(const Shader::Ptr &shader)
{
	if (!m_id)
		create();

	if (!m_idOwned)
	{
		LOGWARNING(">DirectGL::Shaders::Program::attachShader(Shader)> Tried to attach shader to foreign program");
		return;
	}

	shader->compile();
	glAttachShader(m_id, *shader);
	OPENGL_ERROR_CHECK("Shader attaching failed: ");
	switch (shader->getType())
	{
	case GL_VERTEX_SHADER:
		m_vertexShader = shader;
		break;
	case GL_FRAGMENT_SHADER:
		m_fragmentShader = shader;
		break;
	case GL_TESS_CONTROL_SHADER:
		m_tessControlShader = shader;
		break;
	case GL_TESS_EVALUATION_SHADER:
		m_tessEvalShader = shader;
		break;
	case GL_GEOMETRY_SHADER:
		m_geometryShader = shader;
		break;
	}
	m_linked = false;
}

void DirectGL::Shaders::Program::link()
{
	if (!m_id)
	{
		LOGERROR("Program not initialized!");
		return;
	}

	assert(getOwner().isBound());

	glLinkProgram(m_id);

	GLint status = 0;
	glGetProgramiv(m_id, GL_LINK_STATUS, &status);

	if (status == GL_FALSE)
	{
		status = 0;
		glGetProgramiv(m_id, GL_INFO_LOG_LENGTH, &status);

		if (status > 0)
		{
			GLchar *errorLog = new GLchar[status];
			glGetProgramInfoLog(m_id, status, &status, errorLog);
			LOGERROR("Program linking failed:\n" + std::string(errorLog));
			delete[] errorLog;
		}
	}
	else
		m_linked = true;

	OPENGL_ERROR_CHECK("Program linking failed: ");
	m_uniformMap.clear(); // Uniforms could have changed, map will be rebuild by the following uniform setters
}

void DirectGL::Shaders::Program::BindAttribLocation(GLuint index, const std::string &name)
{
	if (!m_id)
		create();
	assert(getOwner().isBound());
	glBindAttribLocation(m_id, index, name.c_str());
	OPENGL_ERROR_CHECK("Failed to bind vertex attribute location: ");
	m_linked = false;
}

void DirectGL::Shaders::Program::BindFragDataLocation(GLuint colorNumber, const std::string &name)
{
	if (!m_id)
		create();
	assert(getOwner().isBound());
	glBindFragDataLocation(m_id, colorNumber, name.c_str());
	OPENGL_ERROR_CHECK("Fragment data location binding failed: ");
	m_linked = false;
}

void DirectGL::Shaders::Program::fromFiles(const std::string &vertexShader, const std::string &fragmentShader, const std::string &tessControlShader, const std::string &tessEvalShader, const std::string &geometryShader)
{
	clear();
	Shader::Ptr vShader(new Shader(vertexShader, GL_VERTEX_SHADER)),
				fShader(new Shader(fragmentShader, GL_FRAGMENT_SHADER));
	attachShader(vShader);
	attachShader(fShader);

	if (tessControlShader.size() > 0)
	{
		Shader::Ptr tcShader(new Shader(tessControlShader, GL_TESS_CONTROL_SHADER));
		attachShader(tcShader);
	}
	if (tessEvalShader.size() > 0)
	{
		Shader::Ptr teShader(new Shader(tessEvalShader, GL_TESS_EVALUATION_SHADER));
		attachShader(teShader);
	}
	if (geometryShader.size() > 0)
	{
		Shader::Ptr gShader(new Shader(geometryShader, GL_GEOMETRY_SHADER));
		attachShader(gShader);
	}
}

void DirectGL::Shaders::Program::fromSources(const std::vector<std::string> &vertexShader, const std::vector<std::string> &fragmentShader, const std::vector<std::string> &tessControlShader, const std::vector<std::string> &tessEvalShader, const std::vector<std::string> &geometryShader)
{
	clear();
	Shader::Ptr vShader(new Shader(vertexShader, GL_VERTEX_SHADER)),
		fShader(new Shader(fragmentShader, GL_FRAGMENT_SHADER));
	attachShader(vShader);
	attachShader(fShader);

	if (tessControlShader.size() > 0)
	{
		Shader::Ptr tcShader(new Shader(tessControlShader, GL_TESS_CONTROL_SHADER));
		attachShader(tcShader);
	}
	if (tessEvalShader.size() > 0)
	{
		Shader::Ptr teShader(new Shader(tessEvalShader, GL_TESS_EVALUATION_SHADER));
		attachShader(teShader);
	}
	if (geometryShader.size() > 0)
	{
		Shader::Ptr gShader(new Shader(geometryShader, GL_GEOMETRY_SHADER));
		attachShader(gShader);
	}
}

GLint DirectGL::Shaders::Program::GetUniformLocation(const std::string &name)
{
	std::unordered_map<std::string, GLint>::const_iterator f = m_uniformMap.find(name);
	if (f == m_uniformMap.end())
	{
		assert(getOwner().isBound());
		GLint loc = glGetUniformLocation(m_id, name.c_str());
		m_uniformMap[name] = loc; // silently ignoring missing uniforms
#ifdef _DEBUG
		if (loc < 0)
			LOGWARNING("Program does not contain uniform <" + name + ">");
#endif
		return loc;
	}
	else
		return f->second;
}

const DirectGL::Shaders::UniformBlock &DirectGL::Shaders::Program::getUniformBlock(const std::string &name)
{
	auto f = m_uniformBlockMap.find(name);
	if (f == m_uniformBlockMap.end())
	{
		assert(getOwner().isBound());
		auto loc = glGetUniformBlockIndex(m_id, name.c_str());
		if (loc != GL_INVALID_INDEX)
		{
			m_uniformBlockMap[name] = UniformBlock(*this, loc);
			return m_uniformBlockMap[name];
		}
		else
		{
#ifdef _DEBUG
			LOGWARNING("Program does not contain uniform block <" + name + ">");
#endif
			return UniformBlock::Null();
		}
	}
	else
		return f->second;
}

void DirectGL::Shaders::Program::UniformSamplerRange(size_t count, const std::string *names, const GLint *indices)
{
	for (size_t i = 0; i < count; ++i)
		Uniform1i(names[i], indices[i]);
}

void DirectGL::Shaders::Program::setDefine(const std::string &name, const std::string &value)
{
#define SET_AND_COMPILE(shader) if(shader.get()) { shader->setDefine(name, value); shader->compile(); }

	SET_AND_COMPILE(m_vertexShader)
	SET_AND_COMPILE(m_fragmentShader)
	SET_AND_COMPILE(m_tessControlShader)
	SET_AND_COMPILE(m_tessEvalShader)
	SET_AND_COMPILE(m_geometryShader)

	m_linked = false; // we'll relink on the next shader usage
}

void DirectGL::Shaders::Program::unuse()
{
	if (Context::getCurrent()->unbindPoint(GL_CURRENT_PROGRAM))
		glUseProgram(0);
}

void DirectGL::Shaders::Program::UniformBlockBinding(const std::string &blockname, GLuint binding)
{
	auto ind(getUniformBlock(blockname).getIndex());
	if (ind != GL_INVALID_INDEX)
	{
		glUniformBlockBinding(m_id, ind, binding);	
	}
}

// ==========================================================================================================

DirectGL::Shaders::Program::ScopedUse::ScopedUse(Program &program) : m_oldProgram(dynamic_cast<const Program*>(Context::getCurrent()->getBoundObject(GL_CURRENT_PROGRAM)))
{
	program.use();
}

DirectGL::Shaders::Program::ScopedUse::~ScopedUse()
{
	if (m_oldProgram != nullptr)
		m_oldProgram->use();
	else
		Context::getCurrent()->unbindPoint(GL_CURRENT_PROGRAM);
}
