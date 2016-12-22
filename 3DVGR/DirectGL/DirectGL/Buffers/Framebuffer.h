/** @file Framebuffer.h =======================================================
*
*	This file provides wrapper and helper classes for Framebuffer Objects.
*
*	@author Julian Meder
* =============================================================================
*/


#pragma once

#include <DirectGL/Forward.h>
#include <DirectGL/Object.h>
#include <DirectGL/Buffers/Renderbuffer.h>
#include <DirectGL/Utility/Logging.h>

#include <set>

namespace DirectGL
{
	namespace Texturing
	{
		class Texture2D;
		class TextureCubeMap;
	}

	namespace Buffers
	{
		class Framebuffer: public DirectGL::Object
		{
		public:
			LOCAL_DECLARE(Framebuffer)

		// CLASSES ====================================================================
			class ScopedBind
			{
			public:
				ScopedBind(const Framebuffer &framebuffer, GLenum attachment = GL_FRAMEBUFFER);
				~ScopedBind();

			private:
				const Framebuffer *m_oldReadBuffer,
								  *m_oldDrawBuffer;
			};

		// DE-/CONSTRUCTORS ===========================================================
			Framebuffer() { clear(); }
			Framebuffer(GLuint foreignID) { fromForeignID(foreignID); }
			~Framebuffer() { clear(); }
			void fromForeignID(GLuint id);

		// OBJECT OVERRIDES ===========================================================
			void create();
			void clear();

		// OPENGL WRAPPER METHODS =====================================================
			void FramebufferRenderbuffer(GLenum attachment, const Renderbuffer &renderbuffer);
			void FramebufferTexture(GLenum attachment, const Texturing::Texture2D &texture, GLint level);
			void FramebufferTexture2D(GLenum attachment, GLenum textarget, const Texturing::TextureCubeMap &texture, GLint level);

			void bind(GLenum target) const;
			void BlitFramebuffer(const Framebuffer &other, GLuint srcX0, GLuint srcY0, GLuint srcX1, GLuint srcY1, GLuint dstX0, GLuint dstY0, GLuint dstX1, GLuint dstY1, GLenum mask, GLenum filter) const;
			void BlitToWindowFramebuffer(GLuint srcX0, GLuint srcY0, GLuint srcX1, GLuint srcY1, GLuint dstX0, GLuint dstY0, GLuint dstX1, GLuint dstY1, GLenum mask, GLenum filter) const;
			
		// CONVENIENCE METHODS ========================================================
			void attach(GLenum attachment, const Renderbuffer &renderbuffer) { FramebufferRenderbuffer(attachment, renderbuffer); }
			void attach(GLenum attachment, const Texturing::Texture2D &texture, GLint level = 0) { FramebufferTexture(attachment, texture, level); }
			void attach(GLenum attachment, GLenum textarget, const Texturing::TextureCubeMap &texture, GLint level = 0) { FramebufferTexture2D(attachment, textarget, texture, level); }
			
			void detach(GLenum attachment);
			void detachAll();
			void detachColorBuffers(GLenum start = GL_COLOR_ATTACHMENT0);

			void setColorDrawRange(GLsizei start, GLsizei count);

		// GLOBAL OPENGL WRAPPERS =====================================================
			static void bindWindowFramebuffer(GLenum target = GL_FRAMEBUFFER);
			static Framebuffer::Ptr getCurrentDrawFramebuffer();
			static Framebuffer::Ptr getCurrentReadFramebuffer();
			static Framebuffer &getNull();

		private:
		// INTERNAL ENUMS =============================================================
			enum AttachType
			{
				Texture,
				Renderbuffer
			};

		// INTERNAL UTILITY FUNCTIONS =================================================
			void addActiveAttachment(GLenum attachment, AttachType type);
			void removeActiveAttachment(GLenum attachment);	

		// INTERNAL TYPEDEFS ==========================================================
			using AttachmentVector = std::vector<GLenum>;
			using AttachmentTypeVector = std::vector<AttachType>;

		// INTERNAL STATE =============================================================
			AttachmentVector m_activeAttachments;
			AttachmentTypeVector m_activeAttachmentTypes;
			GLuint m_numColorBuffers;
			bool m_depthAttached,
				 m_stencilAttached,
				 m_owned;
		};

		class FBOSharer
		{
		public:
		// TYPEDEFS ===============================================================
			using FramebufferType = Framebuffer;

		// CONSTRUCTORS ===========================================================
			FBOSharer(Framebuffer &sharedFBO) : m_fbo(nullptr) { setSharedFBO(sharedFBO); }

		// GETTERS ================================================================
			Framebuffer &getSharedFBO() { return *m_fbo; }
			const Framebuffer &getSharedFBO() const { return *m_fbo; }

		// SETTERS ================================================================
			void setSharedFBO(Framebuffer &sharedFBO) { m_fbo = &sharedFBO; }

		private:
		// INTERNAL STATE =========================================================
			Framebuffer *m_fbo;
		};
	}
}