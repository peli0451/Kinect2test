#include "stdafx.h"
#include <Context.h>
#include <Object.h>
#include <Buffers/Framebuffer.h>
#include <Geometry/PostProcessQuad.h>
#include <Utility/Logging.h>

#include <boost/lexical_cast.hpp>


#ifdef WIN32
#include <epoxy/wgl.h>
#endif

// TODO needs multithread safety

DirectGL::Context *DirectGL::Context::s_current = nullptr;
DirectGL::Context::ContextMap DirectGL::Context::s_contexts;
DirectGL::Context::ContextBoundSet DirectGL::Context::s_boundContexts;
//boost::mutex DirectGL::Context::s_contextMutex;

DirectGL::Context::Ptr DirectGL::Context::getCurrent()
{
#ifdef WIN32
	auto handle = wglGetCurrentContext();
	//assert(s_current == nullptr || s_current->getHandle() == handle);
#else
	// TODO other platform code
#endif

	if (s_current != nullptr && handle == s_current->getHandle())
	{
		//boost::mutex::scoped_lock lock(s_contextMutex);
		return Ptr(s_contexts[s_current->m_handle]);
	}
	else
	{
		auto dc = wglGetCurrentDC();
		if (handle == 0 || dc == 0)
		{
			LOGEXCEPTION("No current assignable OpenGL context found!");
		}
		
		// if a context has been made current in the meantime, a context object might already exist
		if (s_contexts.find(handle) != s_contexts.end())
		{
			//boost::mutex::scoped_lock lock(s_contextMutex);
			Ptr ptr(s_contexts[handle]);
			ptr->bind(dc);
			return ptr;
		}
		else
		{
			//boost::mutex::scoped_lock lock(s_contextMutex);
			Ptr newContext;
			newContext.reset(new Context(handle, dc));
			s_contexts[handle] = boost::weak_ptr<Context>(newContext);
			newContext->bind();
			return newContext;
		}
	}
}

DirectGL::Context::Context(void *handle, void *dc) : m_handle(handle), m_dc(dc)
{
	assert(handle != 0 && dc != 0);
	if (handle == 0 || dc == 0)
		LOGEXCEPTION("Invalid null handle!");

	assert(s_contexts.find(handle) == s_contexts.end());
	if (s_contexts.find(handle) != s_contexts.end())
	{
		LOGEXCEPTION(("Context object for handle <" + boost::lexical_cast<std::string>(handle) + "> already exists!").c_str());
	}

	GLint maxTexunits;
	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxTexunits);
	m_textureBindings.resize(maxTexunits);
	m_textureObjectBindings.resize(maxTexunits);

	m_static_framebuffer.reset(new Buffers::Framebuffer());
	m_static_ppQuad.reset(new Geometry::PostProcessQuad());
}

DirectGL::Context::~Context()
{
	auto found = s_contexts.find(m_handle);
	assert(found != s_contexts.end());
	if (found != s_contexts.end())
		s_contexts.erase(found);

	if (s_current->getHandle() == m_handle)
		unbind();
}

void DirectGL::Context::bind(void *dc)
{
	void *tmpDC = dc;
	if ((dc == 0 || dc == m_dc) && s_current == this)
		return;
	else if (dc == 0)
		tmpDC = m_dc;

	if (s_current != this && s_boundContexts.find(this) != s_boundContexts.end())
		LOGEXCEPTION("This context is already bound in another thread!");

#ifdef WIN32
	auto result = wglMakeCurrent(static_cast<HDC>(tmpDC), static_cast<HGLRC>(m_handle));
	if (result == FALSE)
	{
		LOGWARNING(("Failed to make context <" + boost::lexical_cast<std::string>(m_handle)+"> current for dc <" + boost::lexical_cast<std::string>(dc)+">!").c_str());
		return;
	}
#endif
	s_boundContexts.erase(s_current);
	s_current = this;
	s_boundContexts.insert(this);
	m_dc = tmpDC;
}

void DirectGL::Context::unbind() noexcept
{
	if (s_current != nullptr)
	{
#ifdef WIN32
		auto result = wglMakeCurrent(nullptr, nullptr);
		if (result == FALSE)
		{
			LOGWARNING(("Failed to make current context <" + boost::lexical_cast<std::string>(s_current->m_handle) + "> uncurrent!").c_str());
			return;
		}
#endif
		s_boundContexts.erase(s_current);
		s_current = nullptr;
	}
}

void DirectGL::Context::deallocateAllObjects() noexcept
{
	// When calling the clearing function of an object, it will deregister
	// from its' current owner context automatically, so the list will shrink
	// on its' own
	while (m_objects.size() > 0)
		(*m_objects.begin())->clear();
}

bool DirectGL::Context::bindObject(GLenum bindingPoint, const Object &object) noexcept
{
	if (s_current != this)
		bind();

	if (object.getId() == 0) // binding null objects is equivalent to an unbind operation
		return unbindPoint(bindingPoint);

	auto found(m_bindings.find(bindingPoint));
	if (found != m_bindings.end() && found->second == &object)
			return false;

	m_bindings[bindingPoint] = &object;
	m_objectBindings[&object].insert(bindingPoint);
	return true;
}

bool DirectGL::Context::unbindPoint(GLenum bindingPoint) noexcept
{
	auto found(m_bindings.find(bindingPoint));
	if (found != m_bindings.end())
	{
		m_objectBindings[found->second].erase(bindingPoint);
		m_bindings.erase(found);
		return true;
	}

	return false;
}

void DirectGL::Context::unbindObject(const Object &object) noexcept
{
	auto found(m_objectBindings.find(&object));
	if (found != m_objectBindings.end())
	{
		for (auto &i : found->second)
		{
			m_bindings.erase(i);
		}
		m_objectBindings.erase(found);
	}
}

const DirectGL::Object *DirectGL::Context::getBoundObject(GLenum bindingPoint) noexcept
{
	auto found(m_bindings.find(bindingPoint));
	if (found != m_bindings.end())
		return found->second;
	return nullptr;
}

bool DirectGL::Context::isBound() const noexcept
{
	return s_current == this;
}

bool DirectGL::Context::bindTextureObject(GLenum bindingPoint, GLenum texunit, const Object &object)
{
	int index(texunit - GL_TEXTURE0);
	if (index >= m_textureBindings.size())
		LOGEXCEPTION("Texture unit index out of bounds!");

	auto &binding(m_textureBindings[index]);
	auto found(binding.find(bindingPoint));
	if (found != binding.end() && found->second == &object)
		return false;

	if (found != binding.end())
		m_textureObjectBindings[index][found->second].erase(bindingPoint);

	binding[bindingPoint] = &object;
	auto &texBinding(m_textureObjectBindings[index]);
	texBinding[&object].insert(bindingPoint);
	return true;
}

void DirectGL::Context::unbindTextureObject(const Object &object) noexcept
{
	for (std::size_t i = 0; i < m_textureObjectBindings.size(); ++i)
	{
		auto &curr(m_textureObjectBindings[i]);
		auto found(curr.find(&object));
		if (found != curr.end())
		{
			for (auto &b : found->second)
				m_textureBindings[i].erase(b);
			curr.erase(found);
		}

	}
}

bool DirectGL::Context::unbindTexturePoint(GLenum bindingPoint, GLenum texunit)
{
	auto index(texunit - GL_TEXTURE0);
	if (index >= m_textureBindings.size())
		LOGEXCEPTION("Texture unit index out of bounds!");

	auto &binding(m_textureBindings[index]);
	auto found(binding.find(bindingPoint));
	if (found != binding.end())
	{
		m_textureObjectBindings[index][found->second].erase(bindingPoint);
		binding.erase(found);
		return true;
	}

	return false;
}

const DirectGL::Buffers::FramebufferPtr &DirectGL::Context::getGlobalFramebuffer() const noexcept
{
	assert(isBound());
	if (!isBound())
		LOGWARNING("<DirectGL::Context::getGlobalFramebuffer> Context not currently bound!");

	return m_static_framebuffer;
}

const DirectGL::Geometry::PostProcessQuadPtr &DirectGL::Context::getGlobalPostProcessingQuad() const noexcept
{
	assert(isBound());
	if (!isBound())
		LOGWARNING("<DirectGL::Context::getGlobalPostProcessingQuad> Context not currently bound!");

	return m_static_ppQuad;
}