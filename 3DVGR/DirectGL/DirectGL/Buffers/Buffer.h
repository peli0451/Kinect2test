/** @file Buffer.h ============================================================
*
*	Class encapsulating a general OpenGL buffer object and wrapping some GL
*	functionality into member methods
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include "DirectGL/Object.h"

namespace DirectGL
{
	namespace Buffers
	{
		class Buffer : public DirectGL::Object
		{
		public:
		// OPENGL WRAPPER METHODS =====================================================
			void bind(GLenum target);
			void BufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage);
		
		// OBJECT OVERRIDES ===========================================================
			void create();
			void clear();

		// GETTERS ====================================================================
			GLsizeiptr getSize() const { return m_size; }

		// GLOBAL METHODS =============================================================
			static void unbind(GLenum target) { glBindBuffer(target, 0); }

		protected:
		// PASSED STATE ===============================================================
			GLsizeiptr m_size;
		};
	}
}