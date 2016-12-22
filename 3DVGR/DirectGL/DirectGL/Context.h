/** @file Context.h ===========================================================
*
*	Contains the Context class handling application known OpenGL state caching
*	to avoid unnecessary OpenGL commands.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <epoxy/GL.h>

#include <DirectGL/Forward.h>

#include <unordered_map>
#include <unordered_set>

#include <boost/smart_ptr.hpp>
//#include <boost/thread/mutex.hpp>

namespace DirectGL
{
	class Context
	{
	public:
		LOCAL_DECLARE(Context)

	// DESTRUCTORS ================================================================
		~Context();

	// GETTERS ====================================================================
		void *getHandle() const noexcept { return m_handle; }
		void *getDC() const noexcept { return m_dc; }
		const Object *getBoundObject(GLenum bindingPoint) noexcept;
		std::size_t getMaxTextureUnits() const noexcept { return m_textureBindings.size(); }
		bool isBound() const noexcept;

		static Ptr getCurrent();
		static Ptr fromContext(void *handle);

		const Buffers::FramebufferPtr &getGlobalFramebuffer() const noexcept;
		const Geometry::PostProcessQuadPtr &getGlobalPostProcessingQuad() const noexcept;

	// SETTERS ====================================================================
		void registerObject(Object &object) noexcept { m_objects.insert(&object); }
		void unregisterObject(Object &object) noexcept { unbindObject(object); m_objects.erase(&object); }

	// CACHED BINDING METHODS =====================================================
		bool bindObject(GLenum bindingPoint, const Object &object) noexcept;
		bool unbindPoint(GLenum bindingPoint) noexcept;
		void unbindObject(const Object &object) noexcept;

		bool bindTextureObject(GLenum bindingPoint, GLenum texunit, const Object &object);
		void unbindTextureObject(const Object &object) noexcept;
		bool unbindTexturePoint(GLenum bindingPoint, GLenum texunit);

	// OPENGL WRAPPER METHODS =====================================================
		void bind(void *dc = 0);
		static void unbind() noexcept;
		void deallocateAllObjects() noexcept;

	private:
	// FACTORY SEAL ===============================================================
		Context(void *handle, void *dc);
		Context(const Context &other){}

	// INTERNAL TYPEDEFS ==========================================================
		using ContextMap = std::unordered_map<void*, boost::weak_ptr<Context>>;
		using ContextBoundSet = std::unordered_set<Context*>;
		using ObjectSet = std::unordered_set<Object*>;
		using BindingMap = std::unordered_map<GLenum, const Object*>;
		using ObjectBindingMap = std::unordered_map<const Object*, std::unordered_set<GLenum>>;

	// STATIC VARIABLES ===========================================================
		static __declspec(thread) Context *s_current;
		static ContextMap s_contexts;
		static ContextBoundSet s_boundContexts;
		//static boost::mutex s_contextMutex;
		
	// OBJECT VARIABLES ===========================================================
		void *m_handle;
		void *m_dc;
		ObjectSet m_objects;

		// OpenGL Object
		//ObjectSet m_objects;
		BindingMap m_bindings;
		ObjectBindingMap m_objectBindings;
		std::vector<BindingMap> m_textureBindings;
		std::vector<ObjectBindingMap> m_textureObjectBindings;

		// some context local objects to prevent too many local creations
		Buffers::FramebufferPtr m_static_framebuffer;
		Geometry::PostProcessQuadPtr m_static_ppQuad;
	};
}