#include "stdafx.h"
#include "Object.h"

boost::scoped_ptr<DirectGL::Object> DirectGL::Object::s_nullObject;

DirectGL::Object::Object() : m_id(0)
{
}

DirectGL::Object::~Object()
{
	clear();
}

void DirectGL::Object::create()
{
	m_owner = Context::getCurrent();
	assert(m_owner != nullptr);
	m_owner->registerObject(*this);
}

void DirectGL::Object::clear()
{
	if(m_owner.get() != nullptr) m_owner->unregisterObject(*this);
	m_owner.reset();
}

DirectGL::Object &DirectGL::Object::getNull()
{
	if (s_nullObject.get() == nullptr)
		s_nullObject.reset(new DirectGL::Object());
	return *s_nullObject;
}