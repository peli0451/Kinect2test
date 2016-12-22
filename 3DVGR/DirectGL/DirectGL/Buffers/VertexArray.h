/** @file VertexArray.h =======================================================
*
*	The VertexArray class corresponding to an OpenGL vertex array object is
*	defined here.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <epoxy/gl.h>
#include <DirectGL/Object.h>

namespace DirectGL
{
	namespace Buffers
	{
		class VertexArray : public DirectGL::Object
		{
		public:
		// OBJECT OVERRIDES ===========================================================
			void create();
			void clear();
			
		// OPENGL WRAPPER METHODS =====================================================
			void bind();
			static void unbind();
		};
	}
}