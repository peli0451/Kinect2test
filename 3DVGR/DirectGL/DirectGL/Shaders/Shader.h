/** @file Shader.h ===========================================================
*
*	Contains the Shader OpenGL wrapper class handling OpenGL shader objects.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <epoxy/gl.h>
#include <vector>
#include <string>

#include <DirectGL/Forward.h>
#include <DirectGL/Object.h>

namespace DirectGL
{
	namespace Shaders
	{
		class Shader : public DirectGL::Object
		{
		public:
			LOCAL_DECLARE(Shader)

		// TYPEDEFS
			using UniformNameList = std::vector<std::string>;

		// CONSTRUCTORS ===============================================================
			Shader(GLenum type) { setType(type); }
			Shader(const std::string &filename, GLenum type) { fromFile(filename, type); }
			Shader(const std::vector<std::string> &sources, GLenum type) { fromSources(sources, type); }

		// FROM-CONVERTERS ============================================================
			void fromFile(const std::string &filename, GLenum type);
			void fromFiles(const std::vector<std::string> &filenames, GLenum type);
			void fromSource(const std::string &source, GLenum type) { clear(); m_sources.push_back(source); setType(type); };
			void fromSources(const std::vector<std::string> &sources, GLenum type) { clear(); m_sources = sources; setType(type); }

		// SETTERS ====================================================================
			void addFile(const std::string &filename) { m_sources.push_back(loadShader(filename)); }
			void addSource(const std::string &source){ m_sources.push_back(source); }
			void setDefine(const std::string &name, const std::string &value);
			void setType(GLenum type);

		// GETTERS ====================================================================
			GLenum getType() const { return m_type; }
			std::vector<std::string> getUniforms() const;

		// OBJECT OVERRIDES ===========================================================
			void clear() override;
			void create() override;

		// OPENGL WRAPPERS ============================================================
			void compile();

		// GLOBAL METHODS =============================================================
			static void addSearchPath(const std::string &path);

		private:
		// INTERNAL TYPEDEFS ==========================================================
			using PathList = std::vector<std::string>;

		// INTERNAL UTILITY METHODS ===================================================
			std::string loadShader(const std::string &filename);
		
		// INTERNAL STATE =============================================================
			GLenum m_type;
			std::vector<std::string> m_sources;

		// GLOBAL INTERNALS ===========================================================
			static PathList m_searchPaths;
		};
	}
}