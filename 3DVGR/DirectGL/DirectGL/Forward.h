/** @file Forward.h ===========================================================
*
*	Contains forward declarations for some classes (on demand basis) and
*	defines the overall used LOCAL_DECLARE macro for easy Ptr and List 
*	typedefs.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <boost/shared_ptr.hpp>

#include <vector>

#define FORWARD_ENTRY(type) class type; \
							using type ## Ptr = boost::shared_ptr<type>; \
							using type ## List = std::vector<type ## Ptr>;
#define LOCAL_DECLARE(type) using Ptr = boost::shared_ptr<type>; \
							using List = std::vector<Ptr>;

namespace DirectGL
{
	FORWARD_ENTRY(Object)
	FORWARD_ENTRY(Renderable)
	namespace Buffers
	{
		FORWARD_ENTRY(Framebuffer)
	}
	namespace Geometry
	{
		FORWARD_ENTRY(PostProcessQuad)
	}
	namespace Shaders
	{
		FORWARD_ENTRY(Shader)
		FORWARD_ENTRY(Program)
	}
}