/** @file Object.h ============================================================
*
*	Contains the object base class for all further wrapped OpenGL objects
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <epoxy/gl.h>

#include <DirectGL/Forward.h>
#include <DirectGL/Context.h>
#include <set>

#include <boost/noncopyable.hpp>

namespace DirectGL
{
	class Object : private boost::noncopyable
	{
	public:
		LOCAL_DECLARE(Object)

	// DE-/CONSTRUCTORS ===========================================================
		Object();
		~Object();

	// OPERATORS ==================================================================
		operator GLuint() const noexcept { return m_id; }

	// GETTERS ====================================================================
		inline bool isValid() const noexcept { return m_id != 0; }
		inline GLuint getId() const noexcept { return m_id; }
		inline Context &getOwner() const noexcept { return *m_owner; }

	// BASE MEMBER FUNCTIONS ======================================================
		virtual void create();
		virtual void clear();

	// STATIC HELPERS =============================================================
		static Object &getNull();

	protected:
	// PASSED STATE ===============================================================
		Context::Ptr m_owner;
		GLuint m_id;

	private:
	// INTERNAL STATE =============================================================
		static boost::scoped_ptr<Object> s_nullObject;
	};

	// HELPER TEMPLATES ===========================================================
	template<typename T>
	const T *object_cast(const Object *object) noexcept
	{
		if (object == nullptr)
			return &T::getNull();

		return dynamic_cast<const T*>(object);
	}

	template<typename T>
	T *object_cast(Object *object) noexcept
	{
		if (object == nullptr)
			return &T::getNull();

		return dynamic_cast<T*>(object);
	}
}