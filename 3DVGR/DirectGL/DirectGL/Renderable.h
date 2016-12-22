/** @file Renderable.h ========================================================
*
*	Declares the Renderable base class for all further classes providing draw
*	call functionality.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <DirectGL/Forward.h>
#include <DirectGL/Cameras/Transformation.h>
#include <DirectGL/Geometry/AABB.h>

#include <boost/noncopyable.hpp>

namespace DirectGL
{
	class Renderable : public boost::noncopyable
	{
	public:
		LOCAL_DECLARE(Renderable)

// DESCTRUCTOR ================================================================
		virtual ~Renderable() { clear(); }

// BASE METHODS ===============================================================
		virtual void draw() = 0;
		virtual void draw(Shaders::Program &program);
		virtual void clear();

// GETTERS ====================================================================
		const Geometry::AABB &getAABB() const { return m_aabb; }
		const Cameras::Transformation &getTransformation() const { return m_transformation; }
		Cameras::Transformation &getTransformation() { return m_transformation; }

// SETTERS ====================================================================
		void setTransformation(const Cameras::Transformation &transformation) { m_transformation = transformation; }
		
// OPERATORS ==================================================================
		void operator=(const Renderable &other);

// PASSED =====================================================================
	protected:
		Cameras::Transformation m_transformation;
		Geometry::AABB m_aabb;
	};

}