#include "stdafx.h"
#include "Shader.h"
#include "Utility/GLError.h"

DirectGL::Shaders::Shader::PathList DirectGL::Shaders::Shader::m_searchPaths;

void DirectGL::Shaders::Shader::create()
{
	if (m_id)
		clear();
	Object::create();
	m_id = glCreateShader(m_type);
	OPENGL_ERROR_CHECK("Shader creation failed: ");
}

void DirectGL::Shaders::Shader::fromFile(const std::string &filename, GLenum type)
{
	std::vector<std::string> tmp(1, filename);
	fromFiles(tmp, type);
}

void DirectGL::Shaders::Shader::fromFiles(const std::vector<std::string> &filenames, GLenum type)
{
	std::vector<std::string> sources;
	sources.reserve(filenames.size());
	for (std::vector<std::string>::const_iterator it = filenames.begin(), e = filenames.end(); it != e; ++it)
	{
		sources.push_back(loadShader(*it));
	}
	fromSources(sources, type);
}

void DirectGL::Shaders::Shader::clear()
{
	if (m_id)
	{
		glDeleteShader(m_id);
		m_id = 0;
	}
	m_type = GL_INVALID_ENUM;
	m_sources.clear();
	Object::clear();
}

void DirectGL::Shaders::Shader::setType(GLenum type)
{
	switch (type)
	{
	case GL_VERTEX_SHADER:
	case GL_TESS_CONTROL_SHADER:
	case GL_TESS_EVALUATION_SHADER:
	case GL_GEOMETRY_SHADER:
	case GL_FRAGMENT_SHADER:
		m_type = type;
		return;
	}

	LOGERROR("Invalid shader type!");
	throw std::exception("Invalid shader type!");
}

std::string DirectGL::Shaders::Shader::loadShader(const std::string &filename)
{
	std::ifstream file(filename);
	if (file.fail())
	{
		// the file was not found on the given path, trying all of the search paths
#ifdef _DEBUG
		LOGWARNING("Failed to open file <" + filename + ">, trying all search paths...");
#endif
		auto pos = filename.find_last_of("/\\");
		std::string strippedName = pos != std::string::npos ? filename.substr(pos + 1) : filename;
		for (auto i : m_searchPaths)
		{
			file.clear();
			file.open(i + strippedName);
			if (!file.fail())
				break;
		}

		if (file.fail())
		{
			LOGERROR("Failed to open shader file <" + filename + ">!");
			return "";
		}
	}
	file.seekg(0, std::ios_base::end);
	std::size_t fileSize = file.tellg();
	file.seekg(0, std::ios_base::beg);
	char *buff = new char[fileSize + 1];
	std::memset(buff, 0, fileSize + 1);
	file.read(buff, fileSize);
	std::string retVal(buff);
	delete[] buff;
	return retVal;
}

void DirectGL::Shaders::Shader::compile()
{
	if (m_sources.empty())
	{
		LOGWARNING("No shader sources present for shader compilation!");
		return;
	}
	const char **sources = new const char*[m_sources.size()];
	GLint *length = new GLint[m_sources.size()];
	for (std::size_t i = 0, e = m_sources.size(); i < e; ++i)
	{
		sources[i] = m_sources[i].c_str();
		length[i] = static_cast<GLint>(m_sources[i].size());
	}

	// the shader type might have been changed to something invalid so let's check it
	setType(m_type);
	if (!m_id)
		create();
	getOwner().bind();

	glShaderSource(m_id, static_cast<GLsizei>(m_sources.size()), sources, length);
	glCompileShader(m_id);

	delete[] sources;
	delete[] length;

	GLint status = 0;
	glGetShaderiv(m_id, GL_COMPILE_STATUS, &status);

	if (status == GL_FALSE)
	{
		status = 0;
		glGetShaderiv(m_id, GL_INFO_LOG_LENGTH, &status);

		if (status > 0)
		{
			GLchar *errorLog = new GLchar[status];
			glGetShaderInfoLog(m_id, status, &status, errorLog);
			LOGERROR("Shader compilation failed: " + std::string(errorLog));
			delete[] errorLog;
		}
	}

	OPENGL_ERROR_CHECK("General error during shader compilation: ")
}

std::vector<std::string> DirectGL::Shaders::Shader::getUniforms() const
{
	std::vector<std::string> retVal;
	for (std::vector<std::string>::const_iterator it = m_sources.begin(), e = m_sources.end(); it != e; ++it)
	{
		const std::string &curr = *it;
		std::string::size_type pos = curr.find("uniform ");
		while (pos != std::string::npos)
		{
			pos = curr.find(";", pos);
			do --pos; while (curr[pos] == ' ');
			std::string::size_type end = pos + 1;
			do --pos; while (curr[pos] != ' ');
			++pos;
			retVal.push_back(curr.substr(pos, end - pos));

			pos = curr.find("uniform ", pos);
		}
	}
	return retVal;
}

void DirectGL::Shaders::Shader::addSearchPath(const std::string &path)
{
	auto &lastChar = path[path.size() - 1];
	if (lastChar != '\\' && lastChar != '/')
		m_searchPaths.push_back(path + "/");
	else
		m_searchPaths.push_back(path);
}

void DirectGL::Shaders::Shader::setDefine(const std::string &name, const std::string &value)
{
	for (auto &e : m_sources)
	{
		auto pos = e.find(name + " "); // if there is no space after the define, it's not a value define
		
		if (pos == std::string::npos)
			continue;
		
		pos += 1 + name.size();
		auto pos2 = e.find_first_of("\n", pos);
		
		if (pos == std::string::npos)
			continue;

		auto newString = e.substr(0, pos) + value + e.substr(pos2);

		e = newString;
	} // TODO automatic recompile ? what about relinking all attached programs
}