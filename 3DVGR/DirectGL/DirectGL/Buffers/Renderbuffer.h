/** @file Renderbuffer.h ======================================================
*
*	Provides Renderbuffer wrapper class for OpenGL Renderbuffer objects.
*
*	@author Julian Meder
* =============================================================================
*/


#pragma once

#include <epoxy/gl.h>

#include <DirectGL/Forward.h>
#include <DirectGL/Object.h>

namespace DirectGL
{
	namespace Buffers
	{
		class Renderbuffer : public DirectGL::Object
		{
		public:
			LOCAL_DECLARE(Renderbuffer);
		// OBJECT OVERRIDES ===========================================================
			void create();
			void clear();

		// OPENGL WRAPPER METHODS =====================================================
			inline void bind() const;
			void RenderbufferStorage(GLenum internalformat, GLsizei width, GLsizei height);

		// GETTERS ====================================================================
			GLuint getWidth() const { return m_width; }
			GLuint getHeight() const{ return m_height; }
		
		private:
		// INTERNAL STATE =============================================================
			GLuint m_width,
				   m_height;
		};
	}
}