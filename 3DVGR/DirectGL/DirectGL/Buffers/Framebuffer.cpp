#include "stdafx.h"
#include "Framebuffer.h"
#include "Utility/GLError.h"
#include "Texturing/Texture2D.h"
#include "DirectGL/Texturing/TextureCubeMap.h"

void DirectGL::Buffers::Framebuffer::fromForeignID(GLuint id)
{
	clear();
	m_id = id;
	m_owned = false;
	m_owner = Context::getCurrent();
	 // TODO get all attachments
	// TODO foreign objects do not belong to the owning context
}

void DirectGL::Buffers::Framebuffer::create()
{
	clear();
	Object::create();
	glGenFramebuffers(1, &m_id);
	OPENGL_ERROR_CHECK("Failed to create Framebuffer Object: ");
}

void DirectGL::Buffers::Framebuffer::clear()
{
	if (m_id && m_owned)
	{
		glDeleteFramebuffers(1, &m_id);
	}

	m_id = 0;
	m_numColorBuffers = 0;
	m_activeAttachments.clear();
	m_depthAttached = false;
	m_stencilAttached = false;
	m_owned = true;
	Object::clear();
}

void DirectGL::Buffers::Framebuffer::bind(GLenum target) const 
{
	if (!m_id && m_owned)
		const_cast<Framebuffer*>(this)->create();
	
	switch (target)
	{
	case GL_FRAMEBUFFER:
		if (getOwner().bindObject(GL_DRAW_FRAMEBUFFER, *this) | getOwner().bindObject(GL_READ_FRAMEBUFFER, *this))
			glBindFramebuffer(GL_FRAMEBUFFER, m_id);
		break;
	case GL_DRAW_FRAMEBUFFER:
	case GL_READ_FRAMEBUFFER:
		if (getOwner().bindObject(target, *this))
			glBindFramebuffer(target, m_id);
		break;
	}

#ifdef _DEBUG
	GLint fb;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &fb);
	assert(fb == m_id); // TODO this assert seems to be wrong
#endif
	OPENGL_ERROR_WARN("Encountered error during framebuffer binding: ");
}

void DirectGL::Buffers::Framebuffer::FramebufferTexture(GLenum attachment, const Texturing::Texture2D &texture, GLint level)
{
	if (!m_owned)
		return;

	if (!m_id)
		create();
	bind(GL_FRAMEBUFFER);
	glFramebufferTexture(GL_FRAMEBUFFER, attachment, texture, level);
	GLenum err = glGetError();
	if (!err)
		addActiveAttachment(attachment, Texture);
	else
		OPENGL_LOG_ERR(err, "Failed to attach Texture to framebuffer: ");
}

void DirectGL::Buffers::Framebuffer::FramebufferTexture2D(GLenum attachment, GLenum textarget, const Texturing::TextureCubeMap &texture, GLint level)
{
	if (!m_owned)
		return;

	if (!m_id)
		create();
	bind(GL_FRAMEBUFFER);
	glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, textarget, texture, level);
	GLenum err = glGetError();
	if (!err)
		addActiveAttachment(attachment, Texture);
	else
		OPENGL_LOG_ERR(err, "Failed to attach Texture to framebuffer: ");
	OPENGL_ERROR_CHECK("FramebufferTexture2D failed with error: ");
}

void DirectGL::Buffers::Framebuffer::FramebufferRenderbuffer(GLenum attachment, const DirectGL::Buffers::Renderbuffer &renderbuffer)
{
	if (!m_owned)
		return;

	if (!m_id)
		create();
	bind(GL_FRAMEBUFFER);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, renderbuffer);
	GLenum err = glGetError();
	if (!err)
		addActiveAttachment(attachment, Renderbuffer);
	else
		OPENGL_LOG_ERR(err, "Failed to attach renderbuffer to framebuffer: ");
	OPENGL_ERROR_CHECK("Attaching renderbuffer failed: ");
}

void DirectGL::Buffers::Framebuffer::BlitFramebuffer(const Framebuffer &other, GLuint srcX0, GLuint srcY0, GLuint srcX1, GLuint srcY1, GLuint dstX0, GLuint dstY0, GLuint dstX1, GLuint dstY1, GLenum mask, GLenum filter) const
{
	ScopedBind read(*this, GL_READ_FRAMEBUFFER),
				draw(other, GL_DRAW_FRAMEBUFFER);

	glBlitFramebuffer(srcX0, srcY0, srcX1, srcY1, dstX0, dstY0, dstX1, dstY1, mask, filter);
	OPENGL_ERROR_CHECK("Framebuffer blitting failed: ");
}

void DirectGL::Buffers::Framebuffer::BlitToWindowFramebuffer(GLuint srcX0, GLuint srcY0, GLuint srcX1, GLuint srcY1, GLuint dstX0, GLuint dstY0, GLuint dstX1, GLuint dstY1, GLenum mask, GLenum filter) const
{
	ScopedBind read(*this, GL_READ_FRAMEBUFFER),
				draw(0, GL_DRAW_FRAMEBUFFER);
	glBlitFramebuffer(srcX0, srcY0, srcX1, srcY1, dstX0, dstY0, dstX1, dstY1, mask, filter);
	OPENGL_ERROR_CHECK("Framebuffer blitting failed: ");
}

void DirectGL::Buffers::Framebuffer::detach(GLenum attachment)
{
	if (!m_owned)
		return;

	bind(GL_FRAMEBUFFER);
	glFramebufferTexture(GL_FRAMEBUFFER, attachment, 0, 0);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, 0);
	GLenum err = glGetError();
	if (!err)
		removeActiveAttachment(attachment);
	else
		OPENGL_LOG_ERR(err, "Encountered error during detachment: ");
}

void DirectGL::Buffers::Framebuffer::detachAll()
{
	if (!m_owned)
		return;

	bind(GL_FRAMEBUFFER);
	for (auto i = 0; i < m_activeAttachments.size(); ++i)
	{
		auto &attachment(m_activeAttachments[i]);
		auto &type(m_activeAttachmentTypes[i]);
		switch (type)
		{
		case Texture:
			glFramebufferTexture(GL_FRAMEBUFFER, attachment, 0, 0);
			break;
		case Renderbuffer:
			glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, 0);
			break;
		}
		
		
	}
	m_depthAttached = false;
	m_stencilAttached = false;
	m_numColorBuffers = 0;
	m_activeAttachments.clear();
	m_activeAttachmentTypes.clear();
	GLenum err = glGetError();
	if (err)
		OPENGL_LOG_ERR(err, "Encountered error during detachment: ");
}

void DirectGL::Buffers::Framebuffer::detachColorBuffers(GLenum start)
{
	if (!m_owned)
		return;

	bind(GL_FRAMEBUFFER);
	bool buffersChanged = false;
	for (auto j = 0; j < m_activeAttachments.size(); ++j)
	{
		auto &attachment(m_activeAttachments[j]);
		auto &type(m_activeAttachmentTypes[j]);
		if (attachment >= start && attachment < GL_DEPTH_ATTACHMENT)
		{
			switch (type)
			{
			case Texture:
				glFramebufferTexture(GL_FRAMEBUFFER, attachment, 0, 0);
				break;
			case Renderbuffer:
				glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, 0);
				break;
			}

			buffersChanged = true;
			m_activeAttachments.erase(m_activeAttachments.begin() + j); // TODO not very efficient
			m_activeAttachmentTypes.erase(m_activeAttachmentTypes.begin() + j); // TODO not very efficient
			--m_numColorBuffers;
			--j;
		}
	}
	// setting the apropriate draw buffers to stay consistent to the current attachments
	if (buffersChanged && m_numColorBuffers) glDrawBuffers(static_cast<GLsizei>(m_numColorBuffers), m_activeAttachments.data());
	GLenum err = glGetError();
	if (err)
		OPENGL_LOG_ERR(err, "Encountered error during detachment: ");
}

void DirectGL::Buffers::Framebuffer::addActiveAttachment(GLenum attachment, AttachType type)
{
	auto f = std::lower_bound(m_activeAttachments.begin(), m_activeAttachments.end(), attachment);
	if (f == m_activeAttachments.end() || *f != attachment)
	{
		auto offset = f - m_activeAttachments.begin();
		m_activeAttachments.insert(f, attachment);
		m_activeAttachmentTypes.insert(m_activeAttachmentTypes.begin() + offset, type);
		if (attachment < GL_DEPTH_ATTACHMENT)
			m_numColorBuffers++;
		else
			switch (attachment)
			{
			case GL_DEPTH_ATTACHMENT:
				m_depthAttached = true;
				break;
			case GL_STENCIL_ATTACHMENT: // TODO what about Depth_stencil_attachment?
				m_stencilAttached = true;
				break;
			}
		
		if(m_numColorBuffers) glDrawBuffers(static_cast<GLsizei>(m_numColorBuffers), m_activeAttachments.data());
	}
}

void DirectGL::Buffers::Framebuffer::removeActiveAttachment(GLenum attachment)
{
	AttachmentVector::iterator f = std::lower_bound(m_activeAttachments.begin(), m_activeAttachments.end(), attachment);
	if (f != m_activeAttachments.end() && *f == attachment)
	{
		auto offset = f - m_activeAttachments.begin();
		m_activeAttachments.erase(f);
		m_activeAttachmentTypes.erase(m_activeAttachmentTypes.begin() + offset);
		if (attachment < GL_DEPTH_ATTACHMENT)
			m_numColorBuffers--;
		else
			switch (attachment)
			{
			case GL_DEPTH_ATTACHMENT:
				m_depthAttached = false;
				break;
			case GL_STENCIL_ATTACHMENT:
				m_stencilAttached = false;
				break;
			}

		if(m_numColorBuffers) glDrawBuffers(static_cast<GLsizei>(m_numColorBuffers), m_activeAttachments.data());
	}	
}

void DirectGL::Buffers::Framebuffer::setColorDrawRange(GLsizei start, GLsizei count)
{
	if (m_activeAttachments.size() > 0)
	{
		start = std::max(start, 0);
		GLsizei end = std::min(start + count, static_cast<GLsizei>(m_numColorBuffers));

		assert(getOwner().isBound());
		count = std::max(0, end - start);
		glDrawBuffers(count, &m_activeAttachments[start]);
	}
}

DirectGL::Buffers::Framebuffer::ScopedBind::ScopedBind(const DirectGL::Buffers::Framebuffer &framebuffer, GLenum attachment) :
m_oldDrawBuffer(object_cast<Framebuffer>(Context::getCurrent()->getBoundObject(GL_DRAW_FRAMEBUFFER))),
m_oldReadBuffer(object_cast<Framebuffer>(Context::getCurrent()->getBoundObject(GL_READ_FRAMEBUFFER)))
{
	// TODO replace by owner->getCurrentFramebuffer
#ifdef _DEBUG
	GLint oldDrawBuffer, oldReadBuffer;
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &oldDrawBuffer);
	glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &oldReadBuffer);
	assert(m_oldDrawBuffer->getId() == oldDrawBuffer || m_oldReadBuffer->getId() == oldReadBuffer);
#endif
	framebuffer.bind(attachment);
}

DirectGL::Buffers::Framebuffer::ScopedBind::~ScopedBind()
{
	if (m_oldDrawBuffer == m_oldReadBuffer)
		m_oldDrawBuffer->bind(GL_FRAMEBUFFER);
	else
	{
		m_oldDrawBuffer->bind(GL_DRAW_FRAMEBUFFER);
		m_oldReadBuffer->bind(GL_READ_FRAMEBUFFER);
	}
}

DirectGL::Buffers::Framebuffer::Ptr DirectGL::Buffers::Framebuffer::getCurrentDrawFramebuffer()
{
	GLint id;
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &id);
	return Framebuffer::Ptr(new Framebuffer(id));
}

DirectGL::Buffers::Framebuffer::Ptr DirectGL::Buffers::Framebuffer::getCurrentReadFramebuffer()
{
	GLint id;
	glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &id);
	return Framebuffer::Ptr(new Framebuffer(id));
}

void DirectGL::Buffers::Framebuffer::bindWindowFramebuffer(GLenum target)
{
	switch (target)
	{
	case GL_FRAMEBUFFER:
	{
		auto curr(Context::getCurrent());
		if (curr->unbindPoint(GL_DRAW_FRAMEBUFFER) | curr->unbindPoint(GL_READ_FRAMEBUFFER))
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
		break;
	}
	case GL_DRAW_FRAMEBUFFER:
	case GL_READ_FRAMEBUFFER:
		if (Context::getCurrent()->unbindPoint(target))
			glBindFramebuffer(target, 0);
		break;
	}
}

DirectGL::Buffers::Framebuffer &DirectGL::Buffers::Framebuffer::getNull()
{
	static boost::scoped_ptr<DirectGL::Buffers::Framebuffer> s_nullObject;
	if (s_nullObject.get() == nullptr)
		s_nullObject.reset(new Framebuffer(0));
	return *s_nullObject;
}