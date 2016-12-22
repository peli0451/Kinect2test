/** @file Program.h ===========================================================
*
*	This file declares the Program class, a OpenGL wrapper for the program
*	object. 
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <DirectGL/Forward.h>
#include <DirectGL/Object.h>
#include <DirectGL/Shaders/Shader.h>
#include <DirectGL/Shaders/UniformBlock.h>

#include <unordered_map>
#include <vector>

namespace DirectGL
{
	namespace Shaders
	{
		class Program : public DirectGL::Object
		{
		public:
			LOCAL_DECLARE(Program)

		// CLASSES ====================================================================
			class ScopedUse
			{
			public:
				ScopedUse(Program &program);
				~ScopedUse();
			private:
				const Program *m_oldProgram;
			};

		// CONSTRUCTORS ===============================================================
			Program() : m_linked(false), m_idOwned(false) {}
			Program(const std::string &vertexShader, const std::string &fragmentShader, const std::string &tessControlShader = std::string(""), const std::string &tessEvalShader = std::string(""), const std::string &geometryShader = std::string("")) : m_linked(false), m_idOwned(false) { fromFiles(vertexShader, fragmentShader, tessControlShader, tessEvalShader, geometryShader); }
			Program(GLuint foreignID) : m_linked(false), m_idOwned(false) { fromForeignProgram(foreignID); }
			
		// OBJECT OVERRIDES ===========================================================
			void create() override;
			void clear() override;

		// FROM-CONVERTERS ============================================================
			void fromFiles(const std::string &vertexShader, const std::string &fragmentShader, const std::string &tessControlShader = std::string(""), const std::string &tessEvalShader = std::string(""), const std::string &geometryShader = std::string(""));
			void fromSources(const std::vector<std::string> &vertexShader, const std::vector<std::string> &fragmentShader, const std::vector<std::string> &tessControlShader = std::vector<std::string>(), const std::vector<std::string> &tessEvalShader = std::vector<std::string>(), const std::vector<std::string> &geometryShader = std::vector<std::string>());
			void fromForeignProgram(GLuint program);
			
		// OPENGL WRAPPERS ============================================================
			void attachShader(const Shader::Ptr &shader);
			void BindAttribLocation(GLuint index, const std::string &name);
			void BindFragDataLocation(GLuint colorNumber, const std::string &name);
			GLint GetUniformLocation(const std::string &name);
			const UniformBlock &getUniformBlock(const std::string &name);
			void link();
			inline void use() const;
			
#define UNIFORM_SETTER(method, ...) \
	void method(const std::string &name, ##__VA_ARGS__)
#define UNIFORM_SETTER_BODY(method, ...) \
			{ gl ## method(GetUniformLocation(name), ##__VA_ARGS__); }

			//void Uniform1i(const std::string &name, GLint v0) { glUniform1i(m_uniformMap[name], v0); }

			UNIFORM_SETTER(Uniform1i, GLint v0) UNIFORM_SETTER_BODY(Uniform1i, v0)
			UNIFORM_SETTER(Uniform1ui, GLint v0) UNIFORM_SETTER_BODY(Uniform1ui, v0)
			UNIFORM_SETTER(Uniform1f, GLfloat v0) UNIFORM_SETTER_BODY(Uniform1f, v0)
			UNIFORM_SETTER(Uniform1d, GLdouble v0) UNIFORM_SETTER_BODY(Uniform1d, v0)
			UNIFORM_SETTER(Uniform2i, GLint v0, GLint v1) UNIFORM_SETTER_BODY(Uniform2i, v0, v1)
			UNIFORM_SETTER(Uniform2f, GLfloat v0, GLfloat v1) UNIFORM_SETTER_BODY(Uniform2f, v0, v1)
			UNIFORM_SETTER(Uniform2d, GLdouble v0, GLdouble v1) UNIFORM_SETTER_BODY(Uniform2d, v0, v1)
			UNIFORM_SETTER(Uniform2fv, GLsizei count, const GLfloat *value) UNIFORM_SETTER_BODY(Uniform2fv, count, value)
			UNIFORM_SETTER(Uniform3i, GLint v0, GLint v1, GLint v2) UNIFORM_SETTER_BODY(Uniform3i, v0, v1, v2)
			UNIFORM_SETTER(Uniform3f, GLfloat v0, GLfloat v1, GLfloat v2) UNIFORM_SETTER_BODY(Uniform3f, v0, v1, v2)
			UNIFORM_SETTER(Uniform3fv, GLsizei count, const GLfloat *value) UNIFORM_SETTER_BODY(Uniform3fv, count, value)
			UNIFORM_SETTER(Uniform3d, GLdouble v0, GLdouble v1, GLdouble v2) UNIFORM_SETTER_BODY(Uniform3d, v0, v1, v2)
			UNIFORM_SETTER(UniformMatrix4fv, GLsizei count, GLboolean transpose, const GLfloat *value) UNIFORM_SETTER_BODY(UniformMatrix4fv, count, transpose, value)
#undef UNIFORM_SETTER_BODY
#undef UNIFORM_SETTER

			void UniformSamplerRange(size_t count, const std::string *names, const GLint *indices);
			void UniformBlockBinding(const std::string &blockName, GLuint binding);
			static void unuse();
			
		// METHODS ====================================================================
			void setDefine(const std::string &name, const std::string &value);

		private:
		// INTERNAL TYPEDEFS ==========================================================
			using UniformMap = std::unordered_map<std::string, GLint>;
			using UniformBlockMap = std::unordered_map<std::string, UniformBlock>;

		// INTERNAL STATE =============================================================
			Shader::Ptr m_vertexShader,
						m_fragmentShader,
						m_tessControlShader,
						m_tessEvalShader,
						m_geometryShader;

			UniformMap m_uniformMap;
			UniformBlockMap m_uniformBlockMap;

			bool m_linked,
				 m_idOwned;
		};
	}
}