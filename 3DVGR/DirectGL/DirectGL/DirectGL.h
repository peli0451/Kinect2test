/** @file DirectGL.h ===========================================================
*
*	This file is the main library header, providing convenience include
*	for all participating classes etc., and defining the "scoped" classes,
*	which give scoped OpenGL state changes with automatic reversal on
*	destruction.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <epoxy/gl.h>

#ifdef WIN32
#include <epoxy/wgl.h>
#else
#include <epoxy/glx.h>
#endif

#include <DirectGL/Types.h>
#include <DirectGL/Renderable.h>
#include <DirectGL/Object.h>
#include <DirectGL/Buffers/All>
#include <DirectGL/Cameras/All>
#include <DirectGL/Geometry/All>
#include <DirectGL/Lighting/All>
#include <DirectGL/Shaders/All>
#include <DirectGL/Texturing/All>
#include <DirectGL/Utility/All>
#include <DirectGL/Pipelining/All>

namespace DirectGL
{
	class ScopedPush
	{
	public:
		inline ScopedPush(GLenum what) noexcept { glPushAttrib(what); }
		inline ~ScopedPush() noexcept { glPopAttrib(); }
	};

	class ScopedEnable
	{
	public:
		inline ScopedEnable(GLenum what) noexcept : m_enum(what)  { GLint e = 0; glGetIntegerv(what, &e); m_wasDisabled = e == 0; glEnable(m_enum); }
		inline ~ScopedEnable() noexcept { if (m_wasDisabled) glDisable(m_enum); }

	private:
		GLenum m_enum;
		bool m_wasDisabled;
	};

	class ScopedDisable
	{
	public:
		inline ScopedDisable(GLenum what) noexcept : m_enum(what) { GLint e = 0; glGetIntegerv(what, &e); m_wasEnabled = e > 0; glDisable(m_enum); }
		inline ~ScopedDisable() { if(m_wasEnabled) glEnable(m_enum); }

	private:
		GLenum m_enum;
		bool m_wasEnabled;
	};

	class ScopedViewport
	{
	public:
		inline ScopedViewport(GLint x, GLint y, GLsizei width, GLsizei height) noexcept
		{
			glGetIntegerv(GL_VIEWPORT, m_savedVP);
			glViewport(x, y, width, height);
		}
		inline ~ScopedViewport() noexcept { glViewport(m_savedVP[0], m_savedVP[1], m_savedVP[2], m_savedVP[3]); }

	private:
		GLint m_savedVP[4];
	};

	class ScopedPolygonMode
	{
	public:
		inline ScopedPolygonMode(GLenum mode) noexcept
		{
			glGetIntegerv(GL_POLYGON_MODE, &m_savedMode);
			m_needRestore = mode != m_savedMode;
			if(m_needRestore) glPolygonMode(GL_FRONT_AND_BACK, mode);
		}
		inline ~ScopedPolygonMode() noexcept
		{
			if (m_needRestore) glPolygonMode(GL_FRONT_AND_BACK, m_savedMode);
		}
		
	private:
		GLint m_savedMode;
		bool m_needRestore;
	};

	class ScopedColorMask
	{
	public:
		inline ScopedColorMask(GLboolean red, GLboolean green, GLboolean blue, GLboolean alpha) noexcept
		{
			glGetBooleanv(GL_COLOR_WRITEMASK, m_savedMask);
			m_needRestore = m_savedMask[0] != red || m_savedMask[1] != green || m_savedMask[2] != blue || m_savedMask[3] != alpha;
			if (m_needRestore) glColorMask(red, green, blue, alpha);
		}
		inline ~ScopedColorMask() noexcept
		{
			if (m_needRestore) glColorMask(m_savedMask[0], m_savedMask[1], m_savedMask[2], m_savedMask[3]);
		}

	private:
		GLboolean m_savedMask[4];
		bool m_needRestore;
	};

	class ScopedDepthMask
	{
	public:
		inline ScopedDepthMask(GLboolean depth) noexcept
		{
			glGetBooleanv(GL_DEPTH_WRITEMASK, &m_savedMask);
			m_needRestore = m_savedMask != depth;
			if (m_needRestore) glDepthMask(depth);
		}
		inline ~ScopedDepthMask() noexcept
		{
			if (m_needRestore) glDepthMask(m_savedMask);
		}

	private:
		GLboolean m_savedMask;
		bool m_needRestore;
	};

// CONVENIENCE TYPEDEFS =======================================================
	using ScopedFramebuffer = Buffers::Framebuffer::ScopedBind;
	using ScopedProgram = Shaders::Program::ScopedUse;
}