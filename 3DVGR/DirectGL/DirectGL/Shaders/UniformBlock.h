#pragma once

#include <DirectGL/Types.h>
#include <DirectGL/Forward.h>

#include <map>
#include <string>

namespace DirectGL
{
	namespace Shaders
	{
		class UniformBlock
		{
		public:
			struct BlockUniform
			{
				BlockUniform() {}
				BlockUniform(GLenum dataType, GLenum offset) : dataType(dataType), offset(offset) {}
				GLenum dataType;
				GLint offset;
			};
			using UniformMap = std::map<std::string, BlockUniform>;

			UniformBlock() : m_index(0), m_program(nullptr) {}
			UniformBlock(Program &program, GLuint index);

			const BlockUniform &getUniform(const std::string &name) const;
			GLuint getIndex() const { return m_index; }
			GLuint getBufferSize() const { return m_size; }
			Program &getProgram() const { return *m_program; }
			const UniformMap &getUniforms() const { return m_uniforms; }

			bool isValid() const { return m_program != nullptr; }

			static const UniformBlock &Null() { static auto s_ublock = UniformBlock(); return s_ublock; }

		private:
			UniformMap m_uniforms;
			GLuint m_index,
				   m_size;
			Program *m_program;
		};
	}
}